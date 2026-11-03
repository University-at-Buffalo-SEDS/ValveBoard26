from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]


def function(path, signature):
    source = path.read_text()
    start = source.index(signature)
    opening = source.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


class AutomaticAbortOriginTests(unittest.TestCase):
    def test_actual_cause_clock_reporter_and_peer_admission(self):
        source = (ROOT / 'tests/native/automatic_abort_origin.in.c').read_text()
        header = (ROOT / 'Core/Inc/thread_comm.h').read_text()
        start = header.index('typedef enum {')
        end = header.index(';', header.index('}', start)) + 1
        source = source.replace('@@ENUM@@', header[start:end])
        source = source.replace('@@DIAGNOSTIC_HEADER@@', (ROOT / 'Core/Inc/safety_diagnostics.h').read_text())
        clock = (ROOT / 'Core/Src/telemetry_event_clock.c').read_text()
        clock = '\n'.join(line for line in clock.splitlines() if not line.startswith('#include'))
        source = source.replace('@@EVENT_CLOCK@@', clock)
        functions = [('Core/Src/thread_comm.c', 'uint8_t thread_comm_request_automatic_abort(void)'), ('Core/Src/board_recovery_transport.c', 'uint8_t board_recovery_command_is_stale('), ('Core/Src/board_recovery_transport.c', 'uint8_t board_recovery_automatic_is_stale('), ('Core/Src/telemetry.c', 'SedsResult Abort_handler('), ('Core/Src/telemetry.c', 'static SedsResult log_telemetry_string_asynchronous_locked('), ('Core/Src/telemetry.c', 'static SedsResult telemetry_log_string_original('), ('Core/Src/telemetry.c', 'SedsResult log_telemetry_string_asynchronous_at('), ('Core/Src/telemetry_thread.c', 'static void telemetry_report_safety_abort(void)')]
        for index, (path, signature) in enumerate(functions):
            source = source.replace(f'@@FUNC{index}@@', function(ROOT / path, signature))
        with tempfile.TemporaryDirectory() as directory:
            directory = Path(directory)
            (directory / 'test.c').write_text(source)
            subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=address,undefined', str(directory / 'test.c'),
                            '-o', str(directory / 'test')], check=True)
            for case in ['clock', 'retry', 'delay', 'fresh', 'equal', 'unknown', 'expired', 'rollback', 'launch']:
                with self.subTest(case=case):
                    subprocess.run([str(directory / 'test'), case], check=True)


if __name__ == '__main__':
    unittest.main()
