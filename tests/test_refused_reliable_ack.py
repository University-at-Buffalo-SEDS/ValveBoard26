"""A refused reliable packet is acknowledged, counted and never applied.

The SDK withholds its end-to-end receipt (and forgets the packet's dedupe
identity) whenever an endpoint handler or managed-variable callback fails, so a
deterministic refusal returned as an error makes the station retransmit the
same packet forever. The actual VB handlers run in a real host SDK router; a
receipt is an "E2EACK:" packet on the ingress side. Owners are inert fixtures.
"""
from pathlib import Path
import subprocess
import sys
import tempfile
import re
import unittest
from native_sdk import prepare_native_sdk

ROOT = Path(__file__).resolve().parents[1]


def function(text, name):
    """Return the definition of name, or '' when this source has none."""
    for match in re.finditer(r'^[A-Za-z_][^;{}()\n]*\b' + re.escape(name) + r'\(', text, re.M):
        j = text.index('{', match.end()) if '{' in text[match.end():] else -1
        semi = text.find(';', match.end())
        if j < 0 or (0 <= semi < j):
            continue
        depth, k = 1, j + 1
        while depth:
            depth += (text[k] == '{') - (text[k] == '}')
            k += 1
        return text[match.start():k]
    return ''


class RefusedReliableAckTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdk = prepare_native_sdk(ROOT)
        cls.tmp = tempfile.TemporaryDirectory(prefix='refused-reliable-ack-')
        path = Path(cls.tmp.name)
        code = (ROOT / 'tests/native/refused_reliable_ack.in.c').read_text()
        sources = {name: (ROOT / 'Core/Src' / name).read_text()
                   for name in ('telemetry.c', 'flight_state_cache.c', 'board_recovery_transport.c')}
        owners = {'authorized_primary': 'board_recovery_transport.c',
                  'board_recovery_refused': 'board_recovery_transport.c',
                  'send_status': 'board_recovery_transport.c',
                  'bootstrap_now': 'board_recovery_transport.c',
                  'bootstrap_reply': 'board_recovery_transport.c',
                  'handle_serialized': 'board_recovery_transport.c',
                  'clear_managed_cache': 'flight_state_cache.c',
                  'flight_state_cache_from_wire': 'flight_state_cache.c',
                  'persist_pending': 'flight_state_cache.c',
                  'persist_update': 'flight_state_cache.c',
                  'flight_state_admit': 'telemetry.c',
                  'Flight_State_handler': 'telemetry.c',
                  'flight_state_cache_apply_network_update': 'telemetry.c',
                  'network_admission_result': 'telemetry.c',
                  'Valve_Command_handler': 'telemetry.c',
                  'Abort_handler': 'telemetry.c',
                  'control_packet_handler': 'telemetry.c'}
        for name, owner in owners.items():
            marker = '/* SOURCE_FUNCTION:' + name + ' */'
            assert code.count(marker) == 1, marker
            code = code.replace(marker, function(sources[owner], name))
        source = path / 'refused.c'
        source.write_text(code)
        cls.exe = path / 'refused'
        command = ['cc', '-std=c11', '-DTELEMETRY_ENABLED=1', '-Wall', '-Wextra', '-Werror', '-g',
                   '-Wno-unused-function', '-fsanitize=address,undefined',
                   '-I' + str(ROOT / 'Core/Inc'), '-I' + str(Path(cls.sdk['header']).parent),
                   str(source), cls.sdk['archive'], '-lpthread', '-lm']
        if sys.platform.startswith('linux'):
            command.append('-ldl')
        command += ['-o', str(cls.exe)]
        subprocess.run(command, check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_case(self, mode):
        result = subprocess.run([str(self.exe), str(ROOT / 'config/sedsnet.json'), mode],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result.stdout

    def test_refused_flight_state_is_acknowledged_not_applied(self):
        self.assertIn('PASS refused FLIGHT_STATE', self.run_case('flight_state'))

    def test_in_flight_state_is_applied_and_out_of_set_refused(self):
        self.assertIn('PASS InFlight FLIGHT_STATE', self.run_case('in_flight'))

    def test_retired_flight_state_is_mapped_and_logged_once(self):
        self.assertIn('PASS retired FLIGHT_STATE', self.run_case('retired'))

    def test_refused_valve_command_is_acknowledged_and_counted(self):
        self.assertIn('PASS refused VALVE_COMMAND', self.run_case('valve_command'))

    def test_unauthorized_recovery_request_is_acknowledged(self):
        self.assertIn('PASS unauthorized BOARD_RECOVERY_REQUEST', self.run_case('recovery'))

    def test_station_abort_fanout_is_acknowledged(self):
        self.assertIn('PASS station ABORT', self.run_case('abort'))


if __name__ == '__main__':
    unittest.main()
