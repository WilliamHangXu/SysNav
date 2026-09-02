"""sys.path bootstrap for the embedded SemPathBench checkout.

SysNav embeds a full clone of SemPathBench at ``<SysNav root>/SemPathBench`` (its own git repo,
ignored by SysNav's git; override the location with the ``SEMPATHBENCH_ROOT`` env var). Every
SysNav module that imports SemPathBench code (``scripts.make_maps...``, ``scripts.methods...``,
``scripts.evaluation...``) must import this module first::

    from tools.sempath_export import spb  # noqa: F401  (sys.path bootstrap)
    from scripts.make_maps.procthor.convert_procthor_scene import world_to_grid

Importing this module inserts the clone root at the front of ``sys.path`` and fails with a clear
error if the checkout is missing. ``SEMPATHBENCH_ROOT`` is exported for path construction (e.g.
``resources/maps/real``).
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

_SYSNAV_ROOT = Path(__file__).resolve().parents[2]
SEMPATHBENCH_ROOT = Path(os.environ.get("SEMPATHBENCH_ROOT", _SYSNAV_ROOT / "SemPathBench")).resolve()

# A file every usable checkout must have; guards against pointing at an empty/wrong directory.
_SENTINEL = SEMPATHBENCH_ROOT / "scripts" / "make_maps" / "procthor" / "convert_procthor_scene.py"
if not _SENTINEL.is_file():
    raise ImportError(
        f"SemPathBench checkout not found at {SEMPATHBENCH_ROOT} (missing {_SENTINEL}). "
        f"Clone it there (git clone <remote> {_SYSNAV_ROOT / 'SemPathBench'}) or point the "
        "SEMPATHBENCH_ROOT env var at an existing checkout."
    )

if str(SEMPATHBENCH_ROOT) not in sys.path:
    sys.path.insert(0, str(SEMPATHBENCH_ROOT))
