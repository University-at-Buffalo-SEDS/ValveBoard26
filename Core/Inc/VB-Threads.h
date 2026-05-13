#pragma once
#include "telemetry.h"
#include "tx_api.h"

/* ------ Telemetry Thread ------ */
extern TX_THREAD telemetry_thread;

void telemetry_thread_entry(ULONG initial_input);
UINT create_telemetry_thread(TX_BYTE_POOL *byte_pool);
/* ------ Telemetry Thread ------ */

void main_thread_entry(ULONG initial_input);
UINT create_main_thread(TX_BYTE_POOL *byte_pool);

extern TX_THREAD data_acq_thread;
void data_acq_thread_entry(ULONG initial_input);
UINT create_data_acq_thread(TX_BYTE_POOL *byte_pool);
void data_acq_get_latest_voltages(float voltages[4]);

void safety_thread_entry(ULONG initial_input);
UINT create_safety_thread(TX_BYTE_POOL *byte_pool);

extern TX_EVENT_FLAGS_GROUP event_flags;
extern TX_QUEUE tx_queue;
