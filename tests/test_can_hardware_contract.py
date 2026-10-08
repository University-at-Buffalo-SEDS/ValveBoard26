import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class CanHardwareContract(unittest.TestCase):
    def test_enqueue_timeout_is_counted_as_tx_failure(self):
        source = (ROOT / "Core/Src/can_bus.c").read_text()
        path = source.split("if (slot_status != HAL_OK)", 1)[1].split("return slot_status;", 1)[0]
        self.assertIn("g_fdcan_tx_fail_count++;", path)

    def test_preserves_bus_rate_with_later_sample_point(self):
        source = (ROOT / "Core/Src/main.c").read_text()
        ioc = (ROOT / "Valve_Board26.ioc").read_text()
        self.assertIn("hfdcan2.Init.AutoRetransmission = ENABLE", source)
        self.assertIn("hfdcan2.Init.NominalPrescaler = 2", source)
        self.assertIn("hfdcan2.Init.NominalTimeSeg1 = 19", source)
        self.assertIn("hfdcan2.Init.NominalTimeSeg2 = 4", source)
        self.assertIn("hfdcan2.Init.NominalSyncJumpWidth = 4", source)
        self.assertEqual(2 * (1 + 19 + 4), 16 * (1 + 1 + 1))
        self.assertAlmostEqual((1 + 19) / (1 + 19 + 4), 5 / 6)
        self.assertIn("FDCAN2.NominalPrescaler=2", ioc)
        self.assertIn("FDCAN2.NominalTimeSeg1=19", ioc)
        self.assertIn("FDCAN2.NominalTimeSeg2=4", ioc)
        self.assertIn("FDCAN2.NominalSyncJumpWidth=4", ioc)
        self.assertIn("FDCAN2.CalculateBaudRateNominal=3541666", ioc)
        self.assertIn("FDCAN2.AutoRetransmission=ENABLE", ioc)
        can = (ROOT / "Core/Src/can_bus.c").read_text()
        self.assertIn("can_bus_wait_for_tx_slot", can)
        self.assertIn("CAN_BUS_TX_ENQUEUE_TIMEOUT_MS 5U", can)
        self.assertIn("can_bus_recover_if_bus_off", can)
        self.assertNotIn("< (uint32_t)frag_cnt", can)
