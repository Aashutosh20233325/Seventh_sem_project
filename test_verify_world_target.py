import sar_task
from tools import verify_world_target as vwt

POSE_TEXT = """
pose {
  name: "x500_lidar_2d_oakd_1"
  position {
    x: 0.1
    z: -0.013
  }
}
pose {
  name: "target"
  id: 12
  position {
    x: 3
    y: -3
    z: 0.055
  }
}
"""


def test_live_target_matching_versioned_world_passes():
    found = vwt.parse_pose_info(POSE_TEXT)
    assert found == [(3.0, -3.0, 0.055)]
    assert vwt.check(found, sar_task.load_target()) == []


def test_moved_missing_or_duplicate_target_fails():
    target = sar_task.load_target()
    assert vwt.check([(3.2, -3.0, 0.055)], target)
    assert vwt.check([], target)
    assert vwt.check([(3, -3, 0.055)] * 2, target)


def test_zero_fields_omitted_by_protobuf_parse_as_zero():
    assert vwt.parse_pose_info('pose {\n  name: "target"\n  position {\n    y: -3\n  }\n}') == [(0.0, -3.0, 0.0)]
