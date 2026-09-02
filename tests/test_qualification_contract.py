import json
import unittest
from pathlib import Path

import build


class QualificationContractTests(unittest.TestCase):
    def test_full_runner_profiles_memory_and_linked_network(self):
        root = Path(build.__file__).resolve().parent
        runner = (root / "sim" / "run_full.py").read_text(encoding="utf-8")
        script = (root / "build.py").read_text(encoding="utf-8")

        self.assertIn('"profile"', runner)
        self.assertIn('"--sample-count", "20"', runner)
        self.assertIn('"--traffic-iterations", "1000000"', runner)
        self.assertIn('"bay"', runner)
        self.assertIn('"tx_probe": "fdcan_tx_ok"', runner)
        self.assertIn('"rx_probe": "fdcan_rx"', runner)
        self.assertIn('"host_nodes"', runner)
        self.assertIn('"groundstation"', runner)
        self.assertIn('"rocket_radio"', runner)
        self.assertIn('"fill_pico"', runner)
        self.assertIn('"GS_SIM_VALIDATE_VALVE_ROUNDTRIP": "1"', runner)
        self.assertIn('"probe": "valve_commands_received", "minimum": 1', runner)
        self.assertIn('"probe": "pilot_valve_state", "minimum": 1', runner)
        self.assertIn("forwarded status ACK to GroundStation", runner)
        self.assertIn('simulation_env["SEDS_FIRMWARE_SIM_TEST"] = "1"', runner)
        self.assertIn('run_live(command, "firmware simulation")', runner)
        self.assertIn('running ({int(now - started)}s elapsed)', runner)
        self.assertIn("Long-duration memory profile", script)
        self.assertIn("Network discovery and time sync", script)

    def test_layout_exposes_network_convergence(self):
        root = Path(build.__file__).resolve().parent
        layout = json.loads((root / "sim" / "board.json").read_text(encoding="utf-8"))
        self.assertLess(layout["execution"].get("memory_probe_warmup_samples", 0), layout["execution"]["sample_count"])
        probes = {
            probe["name"]: probe["symbol"]
            for probe in layout["execution"]["memory_probes"]
        }
        self.assertEqual(probes["network_ready"], "g_telemetry_network_ready")
        self.assertEqual(probes["discovery_seen"], "g_telemetry_discovery_seen")
        self.assertEqual(probes["timesync_valid"], "g_telemetry_timesync_valid")

        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        for symbol in (
            "g_telemetry_network_ready",
            "g_telemetry_discovery_seen",
            "g_telemetry_timesync_valid",
        ):
            self.assertIn(symbol, telemetry)

    def test_shared_can_avoids_hop_retry_storms(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        cmake = (root / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn('seds_router_add_side_packed(r, "can", 3U, tx_send, NULL, false)', telemetry)
        self.assertIn('SEDSNET_MAX_QUEUE_BUDGET "8192"', cmake)

    def test_measured_ram_budget_keeps_allocator_and_stack_guards(self):
        root = Path(build.__file__).resolve().parent
        app_config = (root / "AZURE_RTOS/App/app_azure_rtos_config.h").read_text()
        telemetry_thread = (root / "Core/Src/telemetry_thread.c").read_text()
        tx_config = (root / "Core/Inc/tx_user.h").read_text()
        layout = json.loads((root / "sim/board.json").read_text())
        probes = {
            probe["name"]: probe
            for probe in layout["execution"]["memory_probes"]
        }

        self.assertIn("TX_APP_MEM_POOL_SIZE                     46336", app_config)
        self.assertIn("TELEMETRY_THREAD_STACK_SIZE (10U * 1024U)", telemetry_thread)
        self.assertIn(
            "MAIN_THREAD_STACK_SIZE (11U * 1024U)",
            (root / "Core/Src/main_thread.c").read_text(),
        )
        self.assertIn(
            "SAFETY_THREAD_STACK_SIZE (3U * 1024U)",
            (root / "Core/Src/safety_thread.c").read_text(),
        )
        self.assertIn("TX_ENABLE_STACK_CHECKING", tx_config)
        self.assertEqual(probes["pool_low_water"]["minimum"], 1024)
        self.assertEqual(probes["allocation_failures"]["maximum"], 0)
        self.assertEqual(probes["telemetry_stack_remaining"]["minimum"], 1024)

    def test_asserted_valve_ack_has_a_bounded_retry(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core/Src/telemetry.c").read_text()
        status = telemetry.split("SedsResult telemetry_publish_umbilical_status", 1)[1]
        status = status.split("SedsResult telemetry_send_actuator_command", 1)[0]
        self.assertIn("attempt < 3U", status)
        self.assertIn("tx_thread_sleep(1U)", status)
        self.assertIn("seds_router_log_typed", status)


    def test_periodic_health_check_does_not_serialize_topology(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertNotIn("seds_router_export_topology_len", telemetry)
        self.assertIn("g_telemetry_discovery_seen = 1U", telemetry)

if __name__ == "__main__":
    unittest.main()
