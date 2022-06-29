#include "judgement_task.h"
#include "FREERTOS.h"
#include "cmsis_os.h"
#include "judgement_info.h"

UBaseType_t judge_comm_surplus;

void judgement_unpack(void)
{
	osEvent event;
	
	for(;;)
	{
		event = osSignalWait(JUDGE_RX_SIGNAL, osWaitForever);
		
		if (event.status == osEventSignal)
		{
			if (event.value.signals & JUDGE_RX_SIGNAL)
			{
				unpack_fifo_handle();
			}
		}
		
		judge_comm_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}







