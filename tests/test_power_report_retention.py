"""Exercise accepted power-report retention with the actual embedded SDK and schema."""
from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest
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
            lib = work / "sdk"
            lib.mkdir()
            for name in ("src", "sedsnet_macros", "Cargo.toml", "Cargo.lock", "build.rs"):
                path = source / name
                if path.is_dir():
                    shutil.copytree(path, lib / name, ignore=shutil.ignore_patterns("target", ".git"))
                else:
                    shutil.copy2(path, lib / name)
            manifest = lib / "Cargo.toml"
            manifest.write_text(re.sub(r"crate-type\s*=\s*\[[^\]]*\]", 'crate-type = ["rlib"]', manifest.read_text()))
            shutil.copy2(ROOT / "config/sedsnet.json", lib / "telemetry_config.json")
            shutil.copy2(source / "Cargo.lock", work / "Cargo.lock")
            (work / "src").mkdir()
            shutil.copy2(ROOT / "tests/native/power_report_retention.rs", work / "src/main.rs")
            (work / "Cargo.toml").write_text(
                '[package]\nname="valve-power-retention"\nversion="0.1.0"\nedition="2024"\n'
                '[dependencies]\nsedsnet={package="SEDSnet",path="sdk",default-features=false,'
                'features=["discovery","timesync","serde","serde_json","cryptography","compact-packet-store"]}\n'
                'serde_json={version="1",default-features=false,features=["alloc"]}\n')
            subprocess.run(["cargo", "run", "--offline", "--quiet"], cwd=work, check=True, timeout=180)
