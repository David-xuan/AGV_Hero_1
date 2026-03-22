#include "dev_referee.h"
#include "RefereeTask.h"
#include "bsp_log.h"
#include "cmsis_os.h"
#include "dev_referee_ui.h"

RefereeInitConfig_s referee_config = {
    .topic_name = "referee",
    .uart_handle = &huart6,
    .mode = UART_IDLE_MODE,
};
RefereeInstance_s* ref_instance;


void RefereeTask(void const * argument){
	ref_instance = Referee_Register(&referee_config);
    if (ref_instance == NULL)
    {
		Log_Error("Referee Register Failed!");
    }
	 
	 uint16_t UI_PushUp_Counter = 261;
	vTaskDelay(300);
	 
    while (1)
    {
		vTaskDelay(8);
		UI_PushUp_Counter += 1;
		if(UI_PushUp_Counter > 300 && UI_PushUp_Counter % 80 == 0)
		{
			Referee_UI_Draw_Circle(ref_instance ,"CI" ,UI_Graph_Add ,1 ,UI_Color_Pink ,1 ,1000 ,500 ,10);
			Referee_UI_PushUp_Graphs(ref_instance);
		}
//		if (UI_PushUp_Counter % 500==0)
//					{
//						Referee_UI_Draw_Line(ref_instance, "CR", UI_Graph_Add, 1, UI_Color_Cyan, 1, 1366, 46, 1255,276);
//						Referee_UI_PushUp_Graphs(ref_instance);
//				}
    }
}