"""Guards the upstream symbols annotator_ui.py builds on (part of the post-pull compatibility gate).

    python3 -m unittest discover src/sempath_planner/test    (run from the SysNav root)
"""

import sys
import unittest
from pathlib import Path

SYSNAV_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(SYSNAV_ROOT / "src" / "sempath_planner"))


class AnnotatorUpstreamSurfaceTest(unittest.TestCase):
    def test_upstream_symbols_used_by_annotator_ui(self):
        from sempath_planner.annotator_ui import _import_upstream

        mi = _import_upstream(SYSNAV_ROOT)
        for name in ("InstructionAnnotatorHandler", "build_client_state", "load_map_state",
                     "load_annotation_bundle", "make_sample", "validate_sample",
                     "normalize_map_key", "iso_now", "HTML_PAGE"):
            self.assertTrue(hasattr(mi, name), f"upstream make_instruction lost `{name}`")
        self.assertIn("__INITIAL_STATE__", mi.HTML_PAGE,
                      "upstream HTML_PAGE no longer carries the __INITIAL_STATE__ marker")
        self.assertTrue(hasattr(mi.InstructionAnnotatorHandler, "_send_bytes"),
                        "upstream handler lost `_send_bytes`")
        sample = mi.make_sample("real/compat_check_train")
        self.assertIn("expert_route", sample)
        self.assertIn("sample_id", sample)


if __name__ == "__main__":
    unittest.main()
