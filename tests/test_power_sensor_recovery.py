import re
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

class PowerSensorRecovery(unittest.TestCase):
    def test_failed_startup_recovers_without_reinitializing_healthy_sensor(self):
        source = (ROOT / "Core/Src/data_acq_thread.c").read_text()
        functions = "\n".join(re.search(
            r"static void " + name + r"\([^)]*\)\s*\{.*?\n\}", source, re.S).group()
            for name in ("data_acq_ltc2990_init", "data_acq_retry_power_init"))
        code = r"""
#include <assert.h>
#include <stdint.h>
typedef uint32_t ULONG;
#define DATA_ACQ_POWER_RETRY_TICKS 5000U
#define LTC2990_I2C_ADDRESS_VOLTAGE 0x4d
#define LTC2990_I2C_ADDRESS_CURRENT 0x4c
#define LTC2990_ROLE_VOLTAGE 0
#define LTC2990_ROLE_CURRENT 1
static int hi2c2,ltc2990_voltage_handle,ltc2990_current_handle;
static uint8_t ltc2990_voltage_ready,ltc2990_current_ready;
static uint32_t data_acq_voltage_init_fail_count,data_acq_current_init_fail_count;
static ULONG now,data_acq_last_power_retry_ticks;
static unsigned voltage_calls,current_calls;
static int voltage_failed=1,current_failed,bus_stuck;
#define HAL_OK 0
static unsigned recovery_calls;
static int power_i2c_needs_recovery(int *h){assert(h==&hi2c2);return bus_stuck;}
static int power_i2c_recover(int *h){assert(h==&hi2c2);recovery_calls++;bus_stuck=0;return HAL_OK;}
static ULONG tx_time_get(void){return now;}
static int LTC2990_Init(int *h,int *i,unsigned a,int role){
 (void)h;assert(i==&hi2c2);
 if(role==LTC2990_ROLE_VOLTAGE){assert(a==0x4d);voltage_calls++;return voltage_failed;}
 assert(a==0x4c);current_calls++;return current_failed;
}
""" + functions + r"""
int main(void){
 now=1000;data_acq_ltc2990_init();
 assert(!ltc2990_voltage_ready&&ltc2990_current_ready);
 for(now=1001;now<6000;now++)data_acq_retry_power_init();
 assert(voltage_calls==1&&current_calls==1);
 data_acq_retry_power_init();assert(voltage_calls==2&&current_calls==1);
 assert(data_acq_voltage_init_fail_count==2);
 voltage_failed=0;now=11000;data_acq_retry_power_init();
 assert(ltc2990_voltage_ready&&ltc2990_current_ready);
 assert(voltage_calls==3&&current_calls==1);
 now=30000;data_acq_retry_power_init();assert(voltage_calls==3&&current_calls==1);
 // A stuck bus invalidates both handles, even when both were healthy.
 bus_stuck=1;now=35000;data_acq_retry_power_init();
 assert(recovery_calls==1&&voltage_calls==4&&current_calls==2);
 // Unsigned deadline arithmetic remains correct across tick wrap.
 ltc2990_current_ready=0;current_failed=1;now=UINT32_MAX-1000;
 data_acq_ltc2990_init();unsigned before=current_calls;
 now=3998;data_acq_retry_power_init();assert(current_calls==before);
 now=3999;current_failed=0;data_acq_retry_power_init();
 assert(current_calls==before+1&&ltc2990_current_ready&&voltage_calls==4);
}
"""
        with tempfile.TemporaryDirectory() as tmp:
            exe = str(Path(tmp) / "sensor-recovery")
            subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                            "-fsanitize=address,undefined", "-x", "c", "-", "-o", exe],
                           input=code, text=True, check=True)
            subprocess.run([exe], check=True)
