"""Strict parser/classifier for the /fmu/in readiness gate (guardcheck3). Standard library only; never raises a
"safe" verdict on doubt.

  fmu_in_gate_parse.py attempt <topic> <attempt> <rc> <elapsed_s> <stdout> <stderr> <parsed.jsonl>
      Classifies one `ros2 topic info --no-daemon --spin-time 5 -v <topic>` attempt; prints one line starting with
      "safe", "publisher" or "unknown", and appends the parsed record (counts, endpoint identities) to parsed.jsonl.
  fmu_in_gate_parse.py final <parsed.jsonl>
      Cross-topic check over the last record of each topic: all three safe, and their PX4-side readers share at
      least one DDS participant. Prints "safe ..." or "unknown ...".

Verdicts (in this order):
  publisher  any well-formed "Publisher count: N" with N > 0, or any PUBLISHER endpoint record (even if the command
             failed or the rest of the output is malformed) - the gate fails at once, without retry.
  unknown    non-zero exit; not exactly one Type / Publisher count / Subscription count line; malformed count or
             endpoint record; unexpected topic type; endpoint counts not matching the printed counts; or no
             PX4-side reader.
  safe       0 publishers and >= 1 SUBSCRIPTION endpoint from a non-ROS DDS participant: node name and namespace
             both "_CREATED_BY_BARE_DDS_APP_" (rmw_dds_common's placeholder for a participant that publishes no ROS
             discovery info) AND topic type hash "INVALID" (no ROS 2 type-hash user data). A ROS 2 Jazzy node whose
             discovery info is not yet known can appear with the placeholder name, but it still carries an RIHS01
             hash, so it is not accepted. Endpoint records are classified by their own fields, never by position.
"""
import json, re, sys

BARE = "_CREATED_BY_BARE_DDS_APP_"
EXPECTED_TYPE = {"/fmu/in/offboard_control_mode": "px4_msgs/msg/OffboardControlMode",
                 "/fmu/in/vehicle_command": "px4_msgs/msg/VehicleCommand",
                 "/fmu/in/trajectory_setpoint": "px4_msgs/msg/TrajectorySetpoint"}
FIELDS = ("Node name", "Node namespace", "Topic type", "Topic type hash", "Endpoint type", "GID")
GID_RE = re.compile(r"[0-9a-f]{2}(\.[0-9a-f]{2}){15}")


def parse(text):
    """Return (problems, counts, endpoints, publisher_evidence)."""
    problems, endpoints, block = [], [], None
    lines = {"Type": [], "Publisher count": [], "Subscription count": []}
    pub_positive = []
    for raw in text.splitlines():
        line = raw.rstrip()
        if line.startswith("Type: "):
            lines["Type"].append(line[6:])
        elif line.startswith("Publisher count:") or line.startswith("Subscription count:"):
            key, _, value = line.partition(":")
            m = re.fullmatch(r" (\d+)", value)
            lines[key].append(int(m.group(1)) if m else None)
            if m is None:
                problems.append(f"malformed {key!r}: {line!r}")
            elif key == "Publisher count" and int(m.group(1)) > 0:
                pub_positive.append(int(m.group(1)))
            block = None                                            # a count line ends any endpoint record
        elif line.startswith("Node name: "):
            block = {"Node name": line[len("Node name: "):]}
            endpoints.append(block)
        elif block is not None and ": " in line and not line.startswith(" "):
            key, _, value = line.partition(": ")
            if key in FIELDS:
                if key in block:
                    problems.append(f"duplicate field {key!r} in endpoint record")
                block[key] = value
    for k, v in lines.items():
        if len(v) != 1:
            problems.append(f"{len(v)} {k!r} lines (expected 1)")
    for ep in endpoints:
        missing = [f for f in FIELDS if f not in ep]
        if missing:
            problems.append(f"endpoint record missing {missing}")
        if ep.get("Endpoint type") not in ("PUBLISHER", "SUBSCRIPTION"):
            problems.append(f"endpoint type {ep.get('Endpoint type')!r}")
        if "GID" in ep and not GID_RE.fullmatch(ep["GID"]):
            problems.append(f"malformed GID {ep['GID']!r}")
    pub_endpoints = [e for e in endpoints if e.get("Endpoint type") == "PUBLISHER"]
    counts = {"type": lines["Type"], "publisher_count": lines["Publisher count"],
              "subscription_count": lines["Subscription count"]}
    return problems, counts, endpoints, (pub_positive, len(pub_endpoints))


def is_px4_reader(ep):
    return (ep.get("Endpoint type") == "SUBSCRIPTION" and ep.get("Node name") == BARE
            and ep.get("Node namespace") == BARE and ep.get("Topic type hash") == "INVALID")


def classify(topic, rc, text):
    problems, counts, endpoints, (pub_positive, n_pub_ep) = parse(text)
    info = {"counts": counts, "endpoints": [{"node": e.get("Node name"), "namespace": e.get("Node namespace"),
                                             "endpoint_type": e.get("Endpoint type"), "type": e.get("Topic type"),
                                             "type_hash": e.get("Topic type hash"), "gid": e.get("GID"),
                                             "participant": e.get("GID", "")[:35] or None,
                                             "px4_side_reader": is_px4_reader(e)} for e in endpoints],
            "problems": problems}
    info["px4_reader_participants"] = sorted({e["participant"] for e in info["endpoints"] if e["px4_side_reader"]})
    if pub_positive or n_pub_ep:
        return f"publisher count={max(pub_positive, default=0)} endpoints={n_pub_ep}", info
    if rc != 0:
        return f"unknown rc={rc}", info
    if problems:
        return "unknown " + "; ".join(problems), info
    if counts["type"][0] != EXPECTED_TYPE.get(topic):
        return f"unknown unexpected type {counts['type'][0]!r}", info
    if any(e.get("Topic type") != counts["type"][0] for e in endpoints):
        return "unknown endpoint topic type differs from Type line", info
    n_sub = counts["subscription_count"][0]
    n_sub_ep = sum(1 for e in endpoints if e.get("Endpoint type") == "SUBSCRIPTION")
    if counts["publisher_count"][0] != 0 or n_sub_ep != n_sub:
        return f"unknown endpoint mismatch: subscription_count={n_sub} subscription_endpoints={n_sub_ep}", info
    readers = [e for e in info["endpoints"] if e["px4_side_reader"]]
    if not readers:
        others = ",".join(f"{e['node']}/{e['type_hash'][:6] if e['type_hash'] else None}" for e in info["endpoints"])
        return f"unknown px4_side_reader_not_discovered (subscribers: {others or 'none'})", info
    return f"safe publishers=0 subscribers={n_sub} px4_side_readers={len(readers)}", info


def main(argv):
    if argv[1] == "attempt":
        topic, attempt, rc, elapsed, out, err, jsonl = argv[2], int(argv[3]), int(argv[4]), argv[5], *argv[6:9]
        text, stderr = open(out, errors="replace").read(), open(err, errors="replace").read()
        verdict, info = classify(topic, rc, text)
        rec = {"topic": topic, "attempt": attempt, "rc": rc, "elapsed_s": float(elapsed), "result": verdict,
               "stderr": stderr, **info}
        with open(jsonl, "a") as f:
            f.write(json.dumps(rec) + "\n")
        print(verdict)
    elif argv[1] == "final":
        last = {}
        for line in open(argv[2]):
            r = json.loads(line)
            last[r["topic"]] = r
        if set(last) != set(EXPECTED_TYPE) or any(not r["result"].startswith("safe") for r in last.values()):
            print(f"unknown not all topics safe: {sorted((t, r['result'][:20]) for t, r in last.items())}")
            return
        common = set.intersection(*(set(r["px4_reader_participants"]) for r in last.values()))
        if not common:
            print("unknown PX4-side readers do not share a DDS participant: "
                  + json.dumps({t: r["px4_reader_participants"] for t, r in last.items()}))
            return
        print(f"safe common PX4-side participant {sorted(common)}")
    else:
        print("unknown bad parser invocation")


if __name__ == "__main__":
    try:
        main(sys.argv)
    except Exception as exc:                                         # any parser fault is inconclusive, never safe
        print(f"unknown parser_error {type(exc).__name__}: {exc}")
