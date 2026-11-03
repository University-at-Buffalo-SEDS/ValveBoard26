"""Actual receiver, strict journal and staged owner contract; no ACK-as-apply."""
import unittest
import test_recovery_durable_barrier as durable
class RecoveryTransportTests(unittest.TestCase):
 def test_actual_transport_target_time_epoch_and_response(self):
  cls=durable.DurableRecoveryTests
  cls.setUpClass()
  try:cls().case('transport')
  finally:cls.tearDownClass()
