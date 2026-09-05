"""Label vocabularies: SysNav (YOLOE prompts, VLM room types) -> ProcTHOR-style names.

Object labels arriving from ``semantic_mapping`` are the YOLOE *prompt strings* listed in
``src/semantic_mapping/semantic_mapping/config/objects.yaml`` (e.g. ``tv_monitor``, ``trash can``,
``coffee machine``, ``bulletin board``). Room labels are the closed candidate list of
``src/vlm_node/vlm_node/vlm_reasoning_node.py`` (``Office Room``, ``Corridor``, ...), lower-cased by
the VLM node. Both are normalised (lower-case, ``_``/space/``-`` collapsed to one space) before lookup.
"""

from __future__ import annotations

import re
from pathlib import Path

from tools.sempath_export import spb  # noqa: F401  (sys.path bootstrap for the embedded checkout)

from scripts.make_maps.procthor.convert_procthor_scene import NAV_OBJECT_PRIORITY
from scripts.make_maps.procthor.transform_procthor_to_map import _normalize_simple_category

# YOLOE label (normalised) -> ProcTHOR-style objectType (PascalCase, as in ProcTHOR metadata).
DEFAULT_OBJECT_LABEL_ALIASES: dict[str, str] = {
    "chair": "Chair", "armchair": "ArmChair", "office chair": "Chair", "stool": "Stool",
    "desk": "Desk", "table": "DiningTable", "dining table": "DiningTable", "coffee table": "CoffeeTable",
    "side table": "SideTable",
    "sofa": "Sofa", "couch": "Sofa",
    "tv": "Television", "television": "Television", "tv monitor": "Television", "monitor": "Television",
    "trash can": "GarbageCan", "garbage can": "GarbageCan", "garbage bin": "GarbageCan", "trash bin": "GarbageCan",
    "bin": "GarbageCan",
    "plant": "HousePlant", "house plant": "HousePlant", "potted plant": "HousePlant",
    "fridge": "Fridge", "refrigerator": "Fridge",
    "cabinet": "Cabinet", "shelf": "Shelf", "shelving unit": "Shelf", "bookshelf": "Shelf",
    "bed": "Bed", "door": "Doorway", "doorway": "Doorway", "door frame": "Doorframe",
    "sink": "Sink", "toilet": "Toilet", "bathtub": "Bathtub", "shower": "ShowerHead",
    "printer": "Printer", "coffee machine": "CoffeeMachine", "microwave": "Microwave",
    "microwave oven": "Microwave", "vase": "Vase", "book": "Book", "cup": "Cup", "mug": "Mug",
    "laptop": "Laptop", "keyboard": "Keyboard", "mouse": "Mouse", "clock": "Clock",
    "phone": "CellPhone", "cell phone": "CellPhone", "painting": "Painting", "picture": "Painting",
    "bulletin board": "Whiteboard", "whiteboard": "Whiteboard", "sculpture": "Statue", "statue": "Statue",
    "suitcase": "Suitcase", "shoe": "Shoe", "person": "Person", "unknown": "Object",
    "lamp": "FloorLamp", "floor lamp": "FloorLamp", "desk lamp": "DeskLamp", "curtain": "Curtain",
    "water dispenser": "WaterDispenser", "vending machine": "VendingMachine", "bag": "Bag",
    "backpack": "Backpack", "box": "Box", "cardboard box": "Box", "fire extinguisher": "FireExtinguisher",
    "cart": "Cart", "window": "Window", "pillow": "Pillow", "bottle": "Bottle", "pen": "Pen",
    "pencil": "Pencil", "remote": "RemoteControl", "remote control": "RemoteControl", "poster": "Poster",
    "locker": "Locker", "drawer": "Drawer", "dresser": "Dresser", "mirror": "Mirror", "towel": "Towel",
    "ladder": "Ladder", "computer": "Computer", "speaker": "Speaker", "plate": "Plate", "bowl": "Bowl",
}

# Priority used to resolve overlapping footprints per cell (ProcTHOR's NAV_OBJECT_PRIORITY + extras).
SYSNAV_OBJECT_PRIORITY: dict[str, int] = {
    **NAV_OBJECT_PRIORITY,
    "Television": 70, "Printer": 70, "CoffeeMachine": 60, "Microwave": 60, "WaterDispenser": 80,
    "VendingMachine": 90, "Whiteboard": 40, "Painting": 20, "Poster": 20, "Curtain": 20, "Window": 20,
    "Doorway": 80, "Doorframe": 80, "ArmChair": 90, "CoffeeTable": 100, "SideTable": 90, "Stool": 80,
    "Locker": 80, "Dresser": 80, "Person": 0,
}
DEFAULT_OBJECT_PRIORITY = 50

# VLM room label (normalised) -> simple-demo room category (snake_case).
DEFAULT_ROOM_LABEL_ALIASES: dict[str, str] = {
    "office room": "office", "office": "office", "meeting room": "meeting_room", "conference room": "meeting_room",
    "classroom": "classroom", "laboratory": "laboratory", "lab": "laboratory", "computer lab": "computer_lab",
    "restroom": "bathroom", "bathroom": "bathroom", "toilet": "bathroom", "storage room": "storage_room",
    "storage": "storage_room", "copy room": "copy_room", "student lounge": "lounge", "lounge": "lounge",
    "reception": "reception", "lobby": "reception", "corridor": "hallway", "hallway": "hallway",
    "kitchen": "kitchen", "bedroom": "bedroom", "living room": "living_room", "dining room": "dining_room",
    "": "unknown_room", "unknown": "unknown_room",
}

_WS = re.compile(r"[\s_\-]+")


def normalize_label(label: object) -> str:
    """Lower-case and collapse whitespace/underscores/dashes so 'tv_monitor' == 'TV monitor'."""
    return _WS.sub(" ", str(label or "").strip().lower()).strip()


def _pascal_case(label: str) -> str:
    parts = [part for part in _WS.split(label.strip()) if part]
    return "".join(part[:1].upper() + part[1:] for part in parts) or "Object"


def alias_object_label(label: object, aliases: dict[str, str] | None = None) -> str:
    """YOLOE label -> ProcTHOR-style objectType (PascalCase). Unknown labels are PascalCased."""
    table = DEFAULT_OBJECT_LABEL_ALIASES if aliases is None else aliases
    key = normalize_label(label)
    if key in table:
        return table[key]
    return _pascal_case(key)


def alias_room_label(label: object, aliases: dict[str, str] | None = None) -> str:
    """VLM room label -> simple-demo room category (snake_case)."""
    table = DEFAULT_ROOM_LABEL_ALIASES if aliases is None else aliases
    key = normalize_label(label)
    if key in table:
        return table[key]
    return _normalize_simple_category(key, fallback="unknown_room")


def object_priority(object_type: str, priorities: dict[str, int] | None = None) -> int:
    table = SYSNAV_OBJECT_PRIORITY if priorities is None else priorities
    return int(table.get(object_type, DEFAULT_OBJECT_PRIORITY))


def load_exclude_labels(path: str | Path) -> tuple[str, ...]:
    """Load an object exclusion list ``{exclude: [label, ...]}`` -> normalized label tuple.

    Feeds ``ConvertOptions.drop_labels``: listed objects are excluded from map building
    entirely (no instance, no occupancy blocking, no cell ownership). Entries match both the
    raw detector label ("trash can") and the aliased SemPathBench type ("GarbageCan").
    """
    import yaml  # local import: optional dependency

    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    entries = payload.get("exclude")
    if entries is None:
        entries = []
    if not isinstance(entries, list):
        raise ValueError(f"{path}: `exclude` must be a list of object labels")
    return tuple(dict.fromkeys(normalize_label(e) for e in entries if normalize_label(e)))


def load_label_aliases(path: str | Path | None) -> tuple[dict[str, str], dict[str, str]]:
    """Merge a user yaml ``{objects: {label: ObjectType}, rooms: {label: category}}`` over the defaults."""
    objects = dict(DEFAULT_OBJECT_LABEL_ALIASES)
    rooms = dict(DEFAULT_ROOM_LABEL_ALIASES)
    if path is None:
        return objects, rooms
    import yaml  # local import: optional dependency

    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    for key, value in (payload.get("objects") or {}).items():
        objects[normalize_label(key)] = str(value)
    for key, value in (payload.get("rooms") or {}).items():
        rooms[normalize_label(key)] = str(value)
    return objects, rooms
