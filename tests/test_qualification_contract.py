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
        self.assertEqual(runner.count('str(max(1000, layout["execution"]["virtual_time_ms"]))'), 2)
        self.assertIn('"--traffic-iterations", "1000000"', runner)
        self.assertIn('"bay"', runner)
        self.assertIn('"tx_probe": "fdcan_tx_ok"', runner)
        self.assertIn('"rx_probe": "fdcan_rx"', runner)
        self.assertIn('"host_nodes"', runner)
        self.assertIn('"groundstation"', runner)
        self.assertIn('"rocket_radio"', runner)
        self.assertIn('"fill_pico"', runner)
        self.assertIn('"GS_SIM_VALIDATE_VALVE_ROUNDTRIP": "1"', runner)
        self.assertIn('"GS_SIM_VALIDATE_SOAK_COMMANDS": "1" if ultra_soak else "0"', runner)
        self.assertIn("Valve command path remained alive during soak interval", runner)
        self.assertIn("Every ten-minute soak command returned an acknowledgement", runner)
        self.assertIn('"probe": "valve_commands_received", "minimum": 1', runner)
        self.assertIn('"probe": "pilot_valve_state", "minimum": 1', runner)
        self.assertIn("routed status ACK toward GroundStation", runner)
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
        self.assertIn("seds_router_add_side_packed_profile(", telemetry)
        self.assertIn("SEDS_SIDE_TRANSPORT_PROFILE_IPV6_LIKE", telemetry)
        can_bus = (root / "Core" / "Src" / "can_bus.c").read_text(encoding="utf-8")
        self.assertIn("can_bus_wait_for_tx_slot", can_bus)
        self.assertIn("CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 5U", can_bus)
        self.assertNotIn("< (uint32_t)frag_cnt", can_bus)
        self.assertIn("BOARD_CAN_MAX_FRAME_BYTES 128U", telemetry)
        self.assertIn('SEDSNET_MAX_QUEUE_BUDGET "8192"', cmake)
        self.assertIn('SEDSNET_MAX_RECENT_RX_IDS "16"', cmake)
        self.assertIn('SEDSNET_ENV_RELIABLE_MAX_PENDING "8"', cmake)
        self.assertIn('SEDSNET_ENV_RELIABLE_MAX_RETURN_ROUTES "8"', cmake)
        self.assertIn('SEDSNET_ENV_RELIABLE_MAX_END_TO_END_PENDING "8"', cmake)
        self.assertIn('SEDSNET_ENV_RELIABLE_MAX_END_TO_END_ACK_CACHE "8"', cmake)

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

        self.assertIn("TX_APP_MEM_POOL_SIZE                     40960", app_config)
        self.assertIn("TELEMETRY_THREAD_STACK_SIZE (13U * 1024U)", telemetry_thread)
        self.assertIn(
            "MAIN_THREAD_STACK_SIZE (12U * 1024U)",
            (root / "Core/Src/main_thread.c").read_text(),
        )
        self.assertIn(
            "DATA_ACQ_THREAD_STACK_SIZE ((4U * 1024U) + 512U)",
            (root / "Core/Src/data_acq_thread.c").read_text(),
        )
        self.assertIn(
            "SAFETY_THREAD_STACK_SIZE (4U * 1024U)",
            (root / "Core/Src/safety_thread.c").read_text(),
        )
        self.assertIn("TX_ENABLE_STACK_CHECKING", tx_config)
        self.assertIn(
            "FIRMWARE_USB_DEBUG_ENABLED=$<IF:$<CONFIG:Debug>,1,0>",
            (root / "CMakeLists.txt").read_text(),
        )
        self.assertEqual(probes["pool_low_water"]["minimum"], 1024)
        self.assertEqual(probes["allocation_failures"]["maximum"], 0)
        self.assertEqual(probes["telemetry_stack_remaining"]["minimum"], 1024)

    def test_both_valve_ack_states_have_a_bounded_immediate_retry(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core/Src/telemetry.c").read_text()
        status = telemetry.split("SedsResult telemetry_publish_umbilical_status", 1)[1]
        status = status.split("SedsResult telemetry_send_actuator_command", 1)[0]
        self.assertIn("attempt < 3U", status)
        self.assertIn("tx_thread_sleep(1U)", status)
        self.assertIn("seds_router_log_typed", status)
        self.assertNotIn("log_telemetry_asynchronous", status)
        self.assertNotIn("if (on != 0U)", status)

    def test_heartbeat_fail_safe_arms_only_after_a_link_is_established(self):
        root = Path(build.__file__).resolve().parent
        safety = (root / "Core/Src/safety_thread.c").read_text()
        no_link = safety.split("if (last_heartbeat_ms == 0ULL)", 1)[1]
        no_link = no_link.split("if ((now_ms - last_heartbeat_ms)", 1)[0]
        self.assertIn("return;", no_link)
        self.assertNotIn("safety_request_abort", no_link)


    def test_periodic_health_check_does_not_serialize_topology(self):
        root = Path(build.__file__).resolve().parent
        telemetry = (root / "Core" / "Src" / "telemetry.c").read_text(encoding="utf-8")
        self.assertNotIn("seds_router_export_topology_len", telemetry)
        self.assertIn("g_telemetry_discovery_seen = 1U", telemetry)

if __name__ == "__main__":
    unittest.main()
