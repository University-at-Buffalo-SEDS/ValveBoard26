
 if(!strcmp(scenario,"seed_metadata")){assert(launch_request()==TX_SUCCESS);assert(thread_comm_set_flight_state(7)==TX_SUCCESS);save(argv[2]);return 0;}
 if(!strcmp(scenario,"cold_metadata")||!strcmp(scenario,"torn_metadata")){
  assert(!board_launch_latch_was_verified());drop_launch=1;
  assert(thread_comm_apply_managed_flight_state(7)!=TX_SUCCESS&&thread_comm_get_abort());
  assert(launch_request()!=TX_SUCCESS);puts("PASS no completed-inhibition authority after backup-domain loss/torn marker; initial write failure remains safe-off");return 0;
 }
 if(!strcmp(scenario,"warm_metadata")){
  assert(board_launch_latch_was_verified());unsigned before=dsb_writes;drop_launch=1;
  test_tamp.BKP28R=test_tamp.BKP29R=0;
  for(uint8_t state=7;state<=14;state++)assert(thread_comm_apply_network_flight_state(state,10001)==TX_SUCCESS&&!thread_comm_get_abort());
  assert(dsb_writes==before);assert(thread_comm_apply_managed_flight_state(15)!=TX_SUCCESS&&!thread_comm_get_abort());
  puts("PASS full valid retained warm marker established monotonic metadata inhibition; later register loss does not create Abort/refire");return 0;
 }
 if(!strcmp(scenario,"accepted_metadata")){
  assert(launch_request()==TX_SUCCESS);assert(thread_comm_set_flight_state(7)==TX_SUCCESS);

#ifdef VB
  uint64_t original_command;assert(thread_comm_take_launch_sequence_request(&original_command));
#else
  thread_comm_msg_t original_command;assert(thread_comm_receive(&original_command,TX_NO_WAIT)==TX_SUCCESS&&original_command.cmd==14);
#endif
  assert(thread_comm_publish_command_cutoff(10000)==TX_SUCCESS);
  unsigned before=dsb_writes;drop_launch=1;test_tamp.BKP28R=test_tamp.BKP29R=0;
  assert(thread_comm_apply_network_flight_state(7,9999)!=TX_SUCCESS&&!thread_comm_get_abort());
  for(uint8_t state=7;state<=14;state++){
   UINT admitted=thread_comm_apply_managed_flight_state(state);if(admitted!=TX_SUCCESS)fprintf(stderr,"managed state%u admitted=%u Abort=%u DSB_delta=%u\n",state,admitted,thread_comm_get_abort(),dsb_writes-before);assert(admitted==TX_SUCCESS);
   assert(thread_comm_apply_network_flight_state(state,10001)==TX_SUCCESS);
   assert(!thread_comm_get_abort());
  }
  assert(dsb_writes==before);
#ifdef VB
  assert(!peer_sends&&!status_sends&&!g_launch_sequence_active);assert(!thread_comm_take_launch_sequence_request(&original_command));
#else
  assert(thread_comm_receive(&original_command,TX_NO_WAIT)!=TX_SUCCESS);
#endif
  assert(thread_comm_apply_managed_flight_state(15)!=TX_SUCCESS&&!thread_comm_get_abort());
  assert(thread_comm_apply_managed_flight_state(6)!=TX_SUCCESS&&!thread_comm_get_abort());
  /* This is the SAME already accepted original, now at the published cutoff.
   * It must not reacquire physical ownership or reverify SECOND backup loss
   * into a new automatic Abort. Fresh INITIAL admission negatives above remain. */
  assert(launch_request()!=TX_SUCCESS&&!thread_comm_get_abort()&&dsb_writes==before);
#ifdef VB
  assert(!peer_sends&&!status_sends&&!g_launch_sequence_active&&!thread_comm_take_launch_sequence_request(&original_command));
#else
  assert(thread_comm_receive(&original_command,TX_NO_WAIT)!=TX_SUCCESS);
#endif
  puts("PASS already accepted metadata7..14 and retired original: no writes/Abort/regression/refire");return 0;
 }
