#include "rodos.h"

#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dcmi.h"

#include "../Camera.h"

int cnt = 0;
int start = 1;
int count = 0;

extern Camera camera;

extern "C" void DMA2_Stream1_IRQHandler(void) {
	static int K;
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) == SET) {
		DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);
	}
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TEIF1) == SET) {
		PRINTF("DMA Error\n");
		DMA_ClearFlag(DMA2_Stream1, DMA_IT_TEIF1);
	}

}

extern "C" void DCMI_IRQHandler(void) {
	if(DCMI_GetITStatus(DCMI_IT_VSYNC) == SET)
	{
		if(start == 0)
		{
			start = 1;
		}
		else
		{
			start = 0;
		}
		DCMI_ClearFlag(DCMI_IT_VSYNC);
	}
	else if(DCMI_GetITStatus(DCMI_IT_LINE) == SET)
	{
		if(start == 1)
		{
			count++;
		}
		else
		{
			if(count != 0)
			{
			}
			count = 0;
		}
		DCMI_ClearFlag(DCMI_IT_LINE);
	}
	else if(DCMI_GetITStatus(DCMI_IT_FRAME) == SET){
		PRINTF("&FRAME START");
		camera.sendPicture(true);

		DCMI_ClearFlag(DCMI_IT_FRAME);
	}
	else if(DCMI_GetITStatus(DCMI_IT_ERR)== SET){
		PRINTF("DCMI FLAG ERROR\n");
		DCMI_ClearFlag(DCMI_IT_ERR);
	}
	else if(DCMI_GetITStatus(DCMI_IT_OVF) == SET){
		PRINTF("OVERFLOW\n");
		DCMI_ClearFlag(DCMI_IT_OVF);
	}
}
