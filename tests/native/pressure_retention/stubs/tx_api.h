#pragma once
#include <stdint.h>
#include <stddef.h>
typedef uint32_t ULONG;typedef unsigned UINT;
typedef struct{unsigned tx_queue_enqueued,capacity,head,tail;void*buffer;}TX_QUEUE;
typedef struct{void*tx_thread_stack_start,*tx_thread_stack_end,*tx_thread_stack_highest_ptr;}TX_THREAD;
#define TX_SUCCESS 0
#define TX_NO_WAIT 0
#define TX_WAIT_FOREVER 1
#define TX_TIMER_TICKS_PER_SECOND 1000
#define TX_NO_TIME_SLICE 0
#define TX_AUTO_START 1
#define TX_PTR_ERROR 2
#define TX_QUEUE_ERROR 3
static UINT tx_queue_create(TX_QUEUE*,const char*,unsigned,void*,size_t);
static UINT tx_queue_send(TX_QUEUE*,void*,unsigned);
static UINT tx_queue_receive(TX_QUEUE*,void*,unsigned);
static UINT tx_queue_delete(TX_QUEUE*);
static UINT tx_thread_create(TX_THREAD*,const char*,void(*)(ULONG),ULONG,void*,size_t,unsigned,unsigned,unsigned,unsigned);
static void tx_thread_sleep(unsigned);
