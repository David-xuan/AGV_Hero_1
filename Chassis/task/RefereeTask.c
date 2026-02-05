#include "dev_referee.h"
#include "RefereeTask.h"
#include "bsp_log.h"
#include "cmsis_os.h"


RefereeInitConfig_s referee_config = {
    .topic_name = "referee",
    .uart_handle = &huart6,
    .mode = UART_IT_MODE,
};
RefereeInstance_s* ref_instance;


void RefereeTask(void const * argument){
	ref_instance = Referee_Register(&referee_config);
     if (ref_instance == NULL)
     {
         Log_Error("Referee Register Failed!");
     }
    while (1)
    {
        osDelay(1);
    }
}