"""Cereal schema introspection. Wraps cereal/log.capnp, car.capnp and custom.capnp."""
from cereal import log, car, custom

MODULES = {"log": log, "car": car, "custom": custom}

NO_DISCRIMINANT = 0xffff


def _is_union_field(field) -> bool:
  return field.proto.discriminantValue != NO_DISCRIMINANT


def _short_name(schema) -> str:
  # displayName looks like 'car.capnp:CarState.ButtonEvent'
  return schema.node.displayName.split(":")[-1]


def _resolve_struct_schema(name: str, schema: str = "log"):
  if schema not in MODULES:
    raise ValueError(f"unknown schema {schema!r}, expected one of {sorted(MODULES)}")

  # the common case: a top-level message, i.e. a member of the Event union
  ev_fields = log.Event.schema.fields
  if name in ev_fields:
    f = ev_fields[name]
    if f.proto.which() == "slot":
      t = f.proto.slot.type
      if t.which() == "struct":
        return f.schema
      # some messages are lists of structs (e.g. 'can' is List(CanData))
      if t.which() == "list" and t.list.elementType.which() == "struct":
        return f.schema.elementType

  # otherwise a named struct in the requested module (e.g. CanData, CarParams)
  module = MODULES[schema]
  if hasattr(module, name):
    return getattr(module, name).schema

  raise ValueError(f"could not resolve {name!r} as a message or a struct in {schema}.capnp")


def _describe_field(field, max_depth: int, depth: int) -> dict:
  proto = field.proto
  if proto.which() == "group":
    return {"kind": "group", "fields": _describe_struct(field.schema, max_depth, depth)}

  t = proto.slot.type
  tw = t.which()
  entry: dict = {"kind": tw}

  if tw == "enum":
    entry["values"] = [e.name for e in field.schema.node.enum.enumerants]
  elif tw == "struct":
    entry["struct"] = _short_name(field.schema)
    if depth < max_depth:
      entry["fields"] = _describe_struct(field.schema, max_depth, depth + 1)
  elif tw == "list":
    et = t.list.elementType.which()
    entry["element"] = et
    if et == "struct":
      elem = field.schema.elementType
      entry["struct"] = _short_name(elem)
      if depth < max_depth:
        entry["fields"] = _describe_struct(elem, max_depth, depth + 1)
    elif et == "enum":
      try:
        entry["values"] = [e.name for e in field.schema.elementType.node.enum.enumerants]
      except Exception:
        pass

  return entry


def _describe_struct(schema, max_depth: int, depth: int) -> dict:
  return {f.proto.name: _describe_field(f, max_depth, depth) for f in schema.fields_list}


def list_log_messages() -> dict:
  """List the message types (members of the Event union) available in a log."""
  ev = log.Event.schema
  messages = [
    {"name": f.proto.name, "deprecated": "DEPRECATED" in f.proto.name}
    for f in ev.fields_list if _is_union_field(f)
  ]
  return {"count": len(messages), "messages": messages}


def describe_message(message: str, schema: str = "log", max_depth: int = 2) -> dict:
  """Describe the fields and types of a message or struct.

  message: an Event member (e.g. 'carState', 'can') or a struct name (e.g. 'CanData', 'CarParams').
  schema: which capnp file to look up named structs in ('log', 'car', 'custom').
  max_depth: how many levels of nested structs to expand.
  """
  s = _resolve_struct_schema(message, schema)
  return {
    "message": message,
    "struct": _short_name(s),
    "fields": _describe_struct(s, max_depth, 0),
  }
