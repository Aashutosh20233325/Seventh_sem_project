# Stage C plan: live PPO episodes with different target positions

**Status (2026-10-25):** the mechanism is implemented and tested offline (steps 1 and 4 below; see
"Implementation"). No live run has used a variant yet. Passive readiness runs and flights need separate approval.

**Goal:** show live that the policy finds and reaches the target where it is, not only at the one position used in
all 7 live episodes so far (NED (−3, 3)). Stages A and B showed repeatability and search from turned start headings.

## Where the target position lives today

`sim/worlds/sar_walls.sdf`, model `target`, is the single source of ground truth. It is read by:
- **`sar_task.load_target()`** (`WORLD_SDF`): used by `PX4UAVEnv` (success radius, distance, privileged info) and the stand-in;
- **`tools/isolated_sitl.py`:** profile `sar_oakd` launches world `sar_walls` from `sim/worlds/` (`PX4_GZ_WORLDS`);
- **`tools/verify_world_target.py`:** the readiness gate that compares the live Gazebo pose with that file.

The perception node detects the red sphere in the image. It does not read the target position, and neither does the
observation, so the policy never sees the target's position directly.

## Proposed mechanism (small, reviewable)

1. **World variants:** `sim/worlds/variants/<variant>/sar_walls.sdf`, each a copy of `sar_walls.sdf` with **only the
   target pose changed**. The world name stays `sar_walls`, so every topic name, `SAR_LIDAR_TOPIC` and the GZ
   partition are unchanged.
2. **One selector:** `sar_task` resolves `WORLD_SDF` from an optional `SAR_WORLD_VARIANT` (default: the current file),
   refusing unknown names.
   - The launcher gets `--world-variant <name>`, sets `PX4_GZ_WORLDS` to that directory, exports `SAR_WORLD_VARIANT`
     to the stack, and records the variant and the world file's sha256 in `processes.json`.
   - The world-target gate and the runner read the same variant. The runner logs the variant and the sha256 it loaded.
3. **New consistency gate (fail closed):** the world file sha256 must be the same in the launcher manifest, the
   world-target gate and the runner. A mismatch means no flight.
4. **Tests:**
   - variant resolution;
   - only the target pose differs from the base world (structural diff);
   - refusal of unknown variants;
   - the gate and the env agree on the target;
   - the default behaviour is byte-identical without a variant.

## Implementation (offline, 2026-10-25)

- **Selector:** `sar_task.selected_world_sdf()` / `selected_world_variant()` read `SAR_WORLD_VARIANT` at call time;
  `load_target()` without a file uses the selection. Names must match `[a-z0-9][a-z0-9_]*`, and unknown or empty
  values raise.
- **Variants:** `sim/worlds/variants/{c1_ne,c2_sw,c5_w}/sar_walls.sdf`, written by `tools/make_world_variant.py`. It
  changes only the target pose and adds a header comment, refuses targets that fail `valid_random_target`, and never
  overwrites. A test regenerates each file and requires a byte-identical result.
- **Launcher:** `tools/isolated_sitl.py start --world-variant <name>` refuses the start before creating anything unless
  `SAR_WORLD_VARIANT` names the same variant (or both are unset), and only for a project-asset profile. It points
  `PX4_GZ_WORLDS` (server and GUI env files) at the variant directory, and records `world_variant`, `world_sdf` and
  `world_sdf_sha256` in `processes.json`. Without the flag the generated files are identical to before.
- **Gates:** `tools/verify_world_target.py` compares the live pose with the selected world and names the file and
  hash. The new `tools/verify_world_consistency.py --run-dir <run>` fails closed unless the manifest's variant, file
  and sha256 equal the selection in the calling shell, and the file is unchanged.
- **Runner:** logs a `world` event (variant, path, sha256, target NED) before creating the env.
- **Tests:** `test_world_variants.py`, 38 tests; full suite 507 passed.

## Candidate target positions (stand-in check, 2026-10-25, offline)

All are clear of walls by the stand-in's own validity rule (`valid_random_target`), from the default start (facing east):

| Variant | Target NED | Distance | Where from the start | Stand-in PPO (30 episodes) | Stand-in baseline |
|---|---|---|---|---|---|
| C1 | (2.5, 2.5) | 3.5 m | 45° left (camera edge) | 30/30, median 148 steps | 30/30, 272 |
| C2 | (−4.5, −2.5) | 5.2 m | 119° right | 30/30, median 81 | 30/30, 139 |
| C5 | (−1.0, −4.5) | 4.6 m | 167° right (behind) | 30/30, median 92 | 30/30, 161 |

All three lie inside the E1R training distribution (nearest training target within 0.1 m), so this tests the live
system at new positions, not extrapolation. Two other candidates, (2.0, −3.5) and (−5.5, 1.0), failed the validity rule.

## Proposed flights (separate approval)

1. Offline first: implement the mechanism and tests; review and commit.
2. One passive readiness run per variant (no flight) to check the world-target and consistency gates live.
3. Then one PPO flight per variant (C2, C5, C1), plus one baseline reference (C2). Same procedure and stop rules as
   Stage B, with the consistency gate added.

## Risks and open points

- **Target near the FOV edge (C1):** live behaviour may differ from the stand-in, as in stageb1.
- **Field-of-view mismatch:** optionally widen the stand-in camera model to match the live detection edge
  (about 46°), and re-evaluate, before trusting predictions for edge cases.
- **Interior obstacles** (the policy's known weakness) stay out of Stage C.
- **Restore the default world** after the stage: variants are opt-in, so not passing `--world-variant` restores it.
