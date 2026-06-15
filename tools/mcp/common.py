"""Shared helpers for the openpilot MCP server."""
import capnp

from openpilot.tools.lib.logreader import LogReader, ReadMode

# capnp dynamic reader types used for JSON serialization
_DynamicStructReader = capnp.lib.capnp._DynamicStructReader
_DynamicListReader = capnp.lib.capnp._DynamicListReader
_DynamicEnum = capnp.lib.capnp._DynamicEnum

READ_MODES = {
  "rlog": ReadMode.RLOG,
  "qlog": ReadMode.QLOG,
  "auto": ReadMode.AUTO,
}

# guardrail so a single call can't try to return an unbounded log
DEFAULT_MAX_MESSAGES = 10000


def make_log_reader(route: str, mode: str = "rlog") -> LogReader:
  if mode not in READ_MODES:
    raise ValueError(f"unknown mode {mode!r}, expected one of {sorted(READ_MODES)}")
  return LogReader(route, default_mode=READ_MODES[mode], sort_by_time=True)


def _jsonify(obj):
  """Recursively make a plain python object (from capnp .to_dict()) JSON-serializable."""
  if isinstance(obj, (bytes, bytearray)):
    return obj.hex()
  if isinstance(obj, dict):
    return {k: _jsonify(v) for k, v in obj.items()}
  if isinstance(obj, (list, tuple)):
    return [_jsonify(v) for v in obj]
  return obj


def to_jsonable(v):
  """Convert a capnp value (struct/list/enum/data/primitive) into a JSON-serializable value."""
  if isinstance(v, (bytes, bytearray)):
    return v.hex()
  if isinstance(v, _DynamicEnum):
    return str(v)
  if isinstance(v, _DynamicListReader):
    return [to_jsonable(x) for x in v]
  if isinstance(v, _DynamicStructReader):
    return _jsonify(v.to_dict())
  return v


def get_field(obj, path: str):
  """Resolve a dotted field path, e.g. 'wheelSpeeds.fl' or 'leadsV3.0.x'."""
  cur = obj
  for part in path.split("."):
    if part.lstrip("-").isdigit():
      cur = cur[int(part)]
    else:
      cur = getattr(cur, part)
  return cur


def rel_time(mono: int, t0: int) -> float:
  return (mono - t0) * 1e-9
