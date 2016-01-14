/*
 * irqhandler.cpp
 *
 *  Created on: 12.01.2015
 *      Author: Andreas Schartel
 */

#include "rodos.h"

#include "stm32f4xx.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_dcmi.h"

#include "../Camera.h"

int cnt = 0;
int start = 1;
int count = 0;

extern Camera camera;

//extern "C" void DMA1_Stream6_IRQHandler(void){
//	xprintf("Picture IT!\n");
//	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) == SET){
//		//xprintf("Frame send! -> Continue with Payload\n");
//		DMA_ClearFlag(DMA1_Stream6, DMA_IT_TCIF6);
//	}
//}

extern "C" void DMA2_Stream1_IRQHandler(void) {
	static int K;
	//Test on DMA2 Channel1 Transfer Complete interrupt
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1) == SET) {
		xprintf("Frame Complete, detecting Target...\n");
		//camera.ProcessData();
		DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);
	}
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TEIF1) == SET) {
		xprintf("DMA Error\n");
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
		//xprintf("VSYNC");
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
				//xprintf("count: %d \n\n", count); //just for counting the number of line
			}
			count = 0;
		}
		DCMI_ClearFlag(DCMI_IT_LINE);
	}
	else if(DCMI_GetITStatus(DCMI_IT_FRAME) == SET){
		xprintf("FRAME\n");
		camera.sendPicture(true);

		DCMI_ClearFlag(DCMI_IT_FRAME);
	}
	else if(DCMI_GetITStatus(DCMI_IT_ERR)== SET){
		xprintf("DCMI FLAG ERROR\n");
		DCMI_ClearFlag(DCMI_IT_ERR);
	}
	else if(DCMI_GetITStatus(DCMI_IT_OVF) == SET){
		xprintf("OVERFLOW\n");
		DCMI_ClearFlag(DCMI_IT_OVF);
	}
}
