"""Execute the real TLSF hook branch with controlled allocator outcomes."""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest
ROOT = Path(__file__).resolve().parents[1]

class TlsfHookDiagnosticsTests(unittest.TestCase):
    def test_request_size_and_failure_details_survive_tlsf_selection(self):
        source=(ROOT/'Core/Src/telemetry_hooks.c').read_text()
        start=source.index('void *telemetryMalloc(size_t xSize)')
        opening=source.index('{',start)
        depth=1;end=opening+1
        while depth:
            depth += (source[end]=='{')-(source[end]=='}')
            end+=1
        function=source[start:end]
        branch=function.split('#ifdef TELEMETRY_USE_TLSF',1)[1].split('#else',1)[0]
        variables=sorted(set(re.findall(r'\bg_telemetry_[a-z_]+\b',branch)))
        for variable in variables:
            self.assertIn(variable, source[:start], 'Hook uses an undeclared diagnostic: '+variable)
        definitions='\n'.join('volatile uint32_t '+v+';' for v in variables)
        actor=ROOT.name=='ActuatorBoard26'
        request='g_telemetry_failed_alloc_request' if actor else 'g_telemetry_alloc_failure_request'
        available='g_telemetry_failed_alloc_available' if actor else 'g_telemetry_alloc_failure_available'
        last='g_telemetry_last_alloc_request' if actor else 'g_telemetry_max_alloc_request'
        assertions='' if actor else 'assert(g_telemetry_alloc_failure_fragments==3); assert(g_telemetry_alloc_failure_status==TX_NO_MEMORY);'
        harness='''#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#define TELEMETRY_USE_TLSF 1
#define TX_NO_MEMORY 1
typedef unsigned long ULONG;
'''+definitions+'''
static int refuse;
static size_t requested;
static unsigned char storage[64];
static void *telemetry_tlsf_malloc(size_t n){requested=n;return refuse?NULL:storage;}
static void telemetry_memory_profile_sample(void){}
'''+function+f'''
int main(void){{
 assert(telemetryMalloc(0)==storage); assert(requested==1); assert({last}==1);
 assert(telemetryMalloc(32)==storage); assert({last}==32); assert(g_telemetry_alloc_count==2);
 refuse=1;g_telemetry_tlsf_free_bytes=256;
'''+('' if actor else 'g_telemetry_tlsf_free_blocks=3;')+f'''
 assert(telemetryMalloc(64)==NULL); assert({request}==64); assert({available}==256);
 assert(g_telemetry_alloc_fail==1); {assertions}
 refuse=0;assert(telemetryMalloc(16)==storage);
 assert({request}==64); assert({available}==256); assert(g_telemetry_alloc_count==3);
 return 0;
}}
'''
        compiler=shutil.which('cc')
        self.assertIsNotNone(compiler)
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);c=directory/'hook.c';exe=directory/'hook-test'
            c.write_text(harness)
            subprocess.run([compiler,'-std=c11','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True)

if __name__=='__main__':unittest.main()
