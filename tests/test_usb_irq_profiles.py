"""Exercise the complete production IRQ source under both existing USB profiles."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests/native/usb_irq_profiles"


class UsbIrqProfiles(unittest.TestCase):
    def check_profile(self, enabled):
        device_header = ROOT / "Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g491xx.h"
        actual_irq = int(re.search(r"USB_LP_IRQn\s*=\s*(\d+)", device_header.read_text()).group(1))
        fixture_irq = int(re.search(r"USB_LP_IRQn\s*=\s*(\d+)", (FIXTURE / "main.h").read_text()).group(1))
        self.assertEqual(actual_irq, fixture_irq)
        with tempfile.TemporaryDirectory(prefix="vb-usb-irq-") as directory:
            executable = Path(directory) / "irq-control"
            subprocess.run([
                "cc", "-std=c11", "-O1", "-g", "-Wall", "-Wextra", "-Werror",
                "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
                f"-DFIRMWARE_USB_DEBUG_ENABLED={enabled}", "-I" + str(FIXTURE),
                str(ROOT / "Core/Src/stm32g4xx_it.c"), str(FIXTURE / "control.c"),
                "-o", str(executable),
            ], check=True, timeout=30)
            result = subprocess.run([str(executable)], check=True, text=True,
                                    capture_output=True, timeout=10)
            self.assertIn(f"USB_PROFILE_PASS enabled={enabled}", result.stdout)
            self.assertEqual(result.stderr, "")

    def test_disabled_usb_masks_spurious_irq_without_entering_uninitialized_hal(self):
        self.check_profile(0)

    def test_enabled_usb_services_pcd_and_other_interrupts_without_masking(self):
        self.check_profile(1)


if __name__ == "__main__":
    unittest.main()
