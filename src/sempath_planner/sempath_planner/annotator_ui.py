"""In-process SemPathBench annotator UI with deep links and live-plan preview.

Runs the upstream instruction annotator (``scripts/make_instruction/make_instruction.py``) inside
the planner node — the upstream module is imported, never modified. A small handler subclass adds:

  GET /?map=real/sim/<stamp>_train           open directly on that map (upstream opens its default)
  GET /?map=real/sim/<stamp>_train&plan=live additionally inject the node's current GroundPlan
                                             trajectory as the ACTIVE sample, drawn immediately

The injected sample lives only in the served page — nothing is written to the checkout's
instruction files (unless the user explicitly saves it in the UI, which is their call).
"""

from __future__ import annotations

import json
import subprocess
import sys
import threading
import webbrowser
from http import HTTPStatus
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, quote, urlparse


def _import_upstream(sysnav_root: Path):
    if str(sysnav_root) not in sys.path:
        sys.path.insert(0, str(sysnav_root))
    from tools.sempath_export import spb  # noqa: F401  (sys.path bootstrap for the embedded checkout)
    from scripts.make_instruction import make_instruction as mi
    return mi


class AnnotatorUI:
    """Owns the HTTP server thread and the current live plan shown by ``&plan=live``."""

    def __init__(self, sysnav_root: Path, host: str, port: int, logger):
        self._sysnav_root = sysnav_root
        self._host = host
        self._port = port
        self._log = logger
        self._lock = threading.Lock()
        self._live_plan: dict | None = None
        self._mi = None
        self._server: ThreadingHTTPServer | None = None
        self.url: str | None = None

    def start(self) -> str:
        """Import upstream, bind the server (fallback to an ephemeral port), serve in a daemon thread."""
        self._mi = _import_upstream(self._sysnav_root)
        ui = self

        class DeepLinkHandler(ui._mi.InstructionAnnotatorHandler):
            def do_GET(self) -> None:  # noqa: N802
                parsed = urlparse(self.path)
                if parsed.path in ("/", "/index.html") and parsed.query:
                    params = parse_qs(parsed.query)
                    map_id = (params.get("map") or [""])[0]
                    if map_id:
                        try:
                            html = ui._build_deeplink_html(map_id, (params.get("plan") or [""])[0])
                        except Exception as exc:
                            self.send_error(HTTPStatus.BAD_REQUEST, str(exc))
                            return
                        self._send_bytes(html, "text/html; charset=utf-8")
                        return
                super().do_GET()

        try:
            self._server = ThreadingHTTPServer((self._host, self._port), DeepLinkHandler)
        except OSError as exc:
            self._log.warning(f"annotator UI: port {self._port} unavailable ({exc}), using an ephemeral port")
            self._server = ThreadingHTTPServer((self._host, 0), DeepLinkHandler)
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True, name="sempath_annotator_ui").start()
        host, port = self._server.server_address[:2]
        display_host = "127.0.0.1" if host in ("0.0.0.0", "::") else host
        self.url = f"http://{display_host}:{port}"
        return self.url

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server = None

    # ------------------------------------------------------------------ live plan

    def set_live_plan(self, map_key: str, instruction: str, start_pose: dict, trajectory: list) -> None:
        with self._lock:
            self._live_plan = {
                "map_key": map_key,
                "instruction": instruction,
                "start_pose": dict(start_pose),
                "trajectory": [[int(r), int(c)] for r, c in trajectory],
            }

    def clear_live_plan(self) -> None:
        with self._lock:
            self._live_plan = None

    def _build_deeplink_html(self, map_id: str, plan_param: str) -> bytes:
        mi = self._mi
        map_key = mi.normalize_map_key(map_id)
        map_state = mi.load_map_state(map_key)
        annotation_state = None
        with self._lock:
            plan = dict(self._live_plan) if self._live_plan else None
        if plan_param and plan and plan["map_key"] == map_key:
            grid_size = int(map_state["grid_size"])
            bundle = mi.load_annotation_bundle(map_key, grid_size)
            index = len(bundle["samples"]) + 1
            sample = mi.make_sample(map_key, index=index)
            sample["name"] = "LIVE PLAN (sempath_planner)"
            sample["instruction"] = plan["instruction"]
            sample["start_pose"] = plan["start_pose"]
            sample["expert_route"] = plan["trajectory"]
            sample["notes"] = ("GroundPlan output for the live robot session; shown as a preview, "
                               "not saved to the instruction files.")
            sample["created_by"] = "sempath_planner"
            bundle["samples"].append(mi.validate_sample(sample, map_key, index, grid_size))
            bundle["active_sample_id"] = sample["sample_id"]
            bundle["updated_at"] = mi.iso_now()
            annotation_state = bundle
        state = mi.build_client_state(map_key, map_state=map_state, annotation_state=annotation_state)
        html = mi.HTML_PAGE.replace("__INITIAL_STATE__", json.dumps(state, ensure_ascii=False))
        return html.encode("utf-8")

    # ------------------------------------------------------------------ browser

    def map_url(self, map_key: str, *, plan: bool = False) -> str | None:
        if self.url is None:
            return None
        suffix = "&plan=live" if plan else ""
        return f"{self.url}/?map={quote(map_key, safe='')}{suffix}"

    def open_in_browser(self, url: str) -> None:
        """Open the URL in the user's browser; never raises (logs on failure)."""
        try:
            if webbrowser.open(url, new=2):
                return
        except Exception:
            pass
        try:  # webbrowser can miss the desktop default under ros2 launch; xdg-open is the fallback
            subprocess.Popen(["xdg-open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:
            self._log.warning(f"could not open a browser ({exc}); open manually: {url}")
