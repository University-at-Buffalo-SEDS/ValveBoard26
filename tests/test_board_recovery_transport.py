"""Actual receiver, strict journal and staged owner contract; no ACK-as-apply."""
import unittest
import test_recovery_durable_barrier as durable
class RecoveryTransportTests(unittest.TestCase):
 def test_actual_transport_target_time_epoch_and_response(self):
  cls=durable.DurableRecoveryTests
  cls.setUpClass()
  try:cls().case('transport')
  finally:cls.tearDownClass()
 def test_resync_carries_station_phase_without_moving_outputs(self):
  cls=durable.DurableRecoveryTests
  cls.setUpClass()
  try:
   for name in ('resync','resync_unsafe','resync_launch'):
    with self.subTest(name=name):cls().case(name)
  finally:cls.tearDownClass()
