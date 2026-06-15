"""Route inspection. Wraps openpilot.tools.lib.route."""
from openpilot.tools.lib.route import Route


def route_info(route: str, data_dir: str | None = None) -> dict:
  """Return segment count and per-segment file availability for a route."""
  r = Route(route, data_dir=data_dir)

  segments = []
  for s in r.segments:
    segments.append({
      "segment": s.name.segment_num,
      "name": s.name.canonical_name,
      "files": {
        "rlog": s.log_path is not None,
        "qlog": s.qlog_path is not None,
        "fcamera": s.camera_path is not None,
        "ecamera": s.ecamera_path is not None,
        "dcamera": s.dcamera_path is not None,
        "qcamera": s.qcamera_path is not None,
      },
    })

  return {
    "route": r.name.canonical_name,
    "dongle_id": r.name.dongle_id,
    "log_id": r.name.log_id,
    "num_segments": len(r.segments),
    "max_segment": r.max_seg_number,
    "segments": segments,
  }
