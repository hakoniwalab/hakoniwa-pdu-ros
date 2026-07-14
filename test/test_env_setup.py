from __future__ import annotations

import os
import sys
import tempfile
import unittest
from pathlib import Path

from hakoniwa_pdu_ros.env_setup import _prepend_endpoint_paths_from_env


class EndpointImportPathTest(unittest.TestCase):
    def test_build_variant_python_path_adds_endpoint_source_path(self) -> None:
        original_path = list(sys.path)
        original_env = os.environ.get("HAKONIWA_PDU_ENDPOINT_PYTHON_PATH")
        original_module = sys.modules.pop("hakoniwa_pdu_endpoint", None)
        try:
            with tempfile.TemporaryDirectory() as tmpdir:
                repo = Path(tmpdir) / "hakoniwa-pdu-endpoint"
                build_python = repo / "build-zenoh-shared" / "python"
                build_package = build_python / "hakoniwa_pdu_endpoint"
                source_python = repo / "python"
                source_package = source_python / "hakoniwa_pdu_endpoint"
                build_package.mkdir(parents=True)
                source_package.mkdir(parents=True)
                (build_package / "__init__.py").write_text("", encoding="utf-8")
                (build_package / "_c_endpoint_ffi.py").write_text("", encoding="utf-8")
                (source_package / "__init__.py").write_text("", encoding="utf-8")
                (source_package / "c_endpoint.py").write_text("", encoding="utf-8")

                os.environ["HAKONIWA_PDU_ENDPOINT_PYTHON_PATH"] = str(build_python)
                _prepend_endpoint_paths_from_env("HAKONIWA_PDU_ENDPOINT_PYTHON_PATH")

                import hakoniwa_pdu_endpoint

                self.assertIn(str(build_python.resolve()), sys.path)
                self.assertIn(str(source_python.resolve()), sys.path)
                self.assertIn(str(build_package.resolve()), hakoniwa_pdu_endpoint.__path__)
                self.assertIn(str(source_package.resolve()), hakoniwa_pdu_endpoint.__path__)
        finally:
            sys.path[:] = original_path
            if original_env is None:
                os.environ.pop("HAKONIWA_PDU_ENDPOINT_PYTHON_PATH", None)
            else:
                os.environ["HAKONIWA_PDU_ENDPOINT_PYTHON_PATH"] = original_env
            sys.modules.pop("hakoniwa_pdu_endpoint", None)
            if original_module is not None:
                sys.modules["hakoniwa_pdu_endpoint"] = original_module


if __name__ == "__main__":
    unittest.main()
