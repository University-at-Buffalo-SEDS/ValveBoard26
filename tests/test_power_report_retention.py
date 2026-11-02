"""Exercise accepted power-report retention with the actual embedded SDK and schema."""
from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest
from native_sdk import copy_inputs, dependency_inputs, source_files
ROOT = Path(__file__).resolve().parents[1]

class PowerReportRetentionTests(unittest.TestCase):
    def test_accepted_reports_reach_the_wire_under_shared_queue_pressure(self):
        source = os.environ.get("SEDSNET_TEST_SOURCE")
        if not source:
            cache = ROOT / "build/Release/CMakeCache.txt"
            if cache.is_file():
                source = next((s.split("=", 1)[1] for s in cache.read_text().splitlines()
                               if s.startswith("SEDSNET_DIR:PATH=")), None)
        if not source:
            self.skipTest("configure Release or set SEDSNET_TEST_SOURCE to the SDK used by firmware")
        source = Path(source)
        self.assertTrue((source / "src/router.rs").is_file())
        with tempfile.TemporaryDirectory(prefix="valve-power-retention-") as directory:
            work = Path(directory)
            # The normal host prerequisite prepares an owned SDK snapshot with
            # relocated local CRC dependencies. Keep those real inputs when
            # creating this independent embedded test build too.
            lib, _ = copy_inputs(source, source_files(source), dependency_inputs(source), work / "inputs")
            manifest = lib / "Cargo.toml"
            manifest.write_text(re.sub(r"crate-type\s*=\s*\[[^\]]*\]", 'crate-type = ["rlib"]', manifest.read_text()))
            shutil.copy2(ROOT / "config/sedsnet.json", lib / "telemetry_config.json")
            shutil.copy2(source / "Cargo.lock", work / "Cargo.lock")
            (work / "src").mkdir()
            shutil.copy2(ROOT / "tests/native/power_report_retention.rs", work / "src/main.rs")
            (work / "Cargo.toml").write_text(
                '[package]\nname="valve-power-retention"\nversion="0.1.0"\nedition="2024"\n'
                '[dependencies]\nsedsnet={package="SEDSnet",path="inputs/source",default-features=false,'
                'features=["discovery","timesync","serde","serde_json","cryptography","compact-packet-store"]}\n'
                'serde_json={version="1",default-features=false,features=["alloc"]}\n')
            subprocess.run(["cargo", "run", "--offline", "--quiet"], cwd=work, check=True, timeout=180)
