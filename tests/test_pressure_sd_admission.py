"""Actual archive policy reserves configured reliable credit for live status."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
HEADER = '''#pragma once
#include <stddef.h>
#include <stdint.h>
typedef int SedsResult;
typedef struct { unsigned unused; } SedsRouter;
typedef struct {size_t reliable_max_end_to_end_pending;} SedsRuntimeTuningConfig;
#define SEDS_OK 0
#define SEDS_IO (-14)
#define SEDS_ERR (-1)
SedsResult seds_get_runtime_tuning_config(SedsRuntimeTuningConfig *out);
int32_t seds_router_copy_queue_counters(SedsRouter *,uint32_t *,size_t);
'''


class ArchiveAdmissionTests(unittest.TestCase):
    def compile(self, body, defines=(), valid=True):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            (path / 'sedsnet.h').write_text(HEADER)
            command = ['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                       '-fsanitize=address,undefined', '-I', str(path), '-I', str(ROOT / 'Core/Inc')]
            command += ['-D' + define for define in defines]
            result = subprocess.run(command + ['-x', 'c', '-', '-o', str(path / 'test')],
                                    input=body, text=True, capture_output=True)
            if valid:
                self.assertEqual(result.returncode, 0, result.stderr)
                subprocess.run([str(path / 'test')], check=True)
            else:
                self.assertNotEqual(result.returncode, 0)

    def test_configured_limits_and_stalled_queued_work_cannot_consume_status_reserve(self):
        self.compile(r'''
#include "pressure_sd_admission.h"
#include <assert.h>
#include <string.h>
static size_t limit=32;
static uint32_t pending,queued;
static int config_result,copy_result=16;
SedsResult seds_get_runtime_tuning_config(SedsRuntimeTuningConfig *out){out->reliable_max_end_to_end_pending=limit;return config_result;}
int32_t seds_router_copy_queue_counters(SedsRouter *r,uint32_t*out,size_t n){assert(r&&n==16);memset(out,0,n*4);out[2]=queued;out[11]=pending;return copy_result;}
int main(void){
 SedsRouter router={0};assert(pressure_sd_archive_admission(NULL)==SEDS_IO);
 assert(pressure_sd_archive_admission(&router)==SEDS_OK);
 pending=23;assert(pressure_sd_archive_admission(&router)==SEDS_OK);
 pending=24;assert(pressure_sd_archive_admission(&router)==SEDS_IO);
 pending=0;queued=24;assert(pressure_sd_archive_admission(&router)==SEDS_IO);
 queued=12;pending=12;assert(pressure_sd_archive_admission(&router)==SEDS_IO);
 queued=11;assert(pressure_sd_archive_admission(&router)==SEDS_OK);
 limit=8;pending=3;queued=0;assert(pressure_sd_archive_admission(&router)==SEDS_OK);
 pending=4;assert(pressure_sd_archive_admission(&router)==SEDS_IO);
 limit=1;pending=queued=0;assert(pressure_sd_archive_admission(&router)==SEDS_IO);
 limit=0;assert(pressure_sd_archive_admission(&router)==SEDS_IO);
 limit=SIZE_MAX;pending=queued=UINT32_MAX;assert(pressure_sd_archive_admission(&router)==SEDS_OK);
 config_result=-3;assert(pressure_sd_archive_admission(&router)==-3);
 config_result=0;copy_result=-14;assert(pressure_sd_archive_admission(&router)==SEDS_ERR);
 return 0;
}''')

    def test_invalid_reservation_configuration_is_rejected(self):
        for percent in [0, 100]:
            self.compile('#include "pressure_sd_admission.h"\nint main(void){return 0;}\n',
                         [f'PRESSURE_SD_CONTROL_RESERVE_PERCENT={percent}'], valid=False)
