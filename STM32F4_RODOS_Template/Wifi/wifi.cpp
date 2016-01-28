/*
 * wifi.cpp
 *
 *  Created on: 19.08.2015
 *      Author: tmikschl
 */
#include <rodos.h>
#include "wifi/wf121.h"
#include "wifi/linkinterfacewf121.h"
#include "../basics.h"


HAL_UART uart3(UART_IDX3); // USB-UART
WF121 wf121(&uart3);
LinkinterfaceWF121 linkwf121(&wf121);
Gateway gateway1(&linkwf121, true);
Topic<uint32_t> topicCounter1 (5001, "Counter");
int send_telemetry;

using namespace RODOS;

class WifiGateway : public Thread {
public:
  WifiGateway() : Thread("WifiGateway") {

  }

  void init() {
	  gateway1.addTopicsToForward(&topicCounter1, &tm_topic_imu, &tm_topic_electrical);
	  send_telemetry = 1;
  }

  void run() {
	  uint32_t cnt=0;

	  tmStructIMU imu;
	  tmStructElectrical electrical;

	  wf121.init("YETENet","yeteyete");
	  wf121.enableUDPConnection(0xFF01A8C0,37647);
	  while (1) {
		  if (send_telemetry) {
			  imuBuffer.get(imu);
			  electricalBuffer.get(electrical);
			  cnt++;

//			  topicCounter1.publish(cnt);
			  tm_topic_imu.publish(imu);
			  tm_topic_electrical.publish(electrical);

		  }
		  suspendCallerUntil(NOW()+250*MILLISECONDS);
		  //PRINTF("%d\n",cnt);
	  }
  }

} wifitest;

uint32_t oldval;

void printMsg(uint32_t& val) {
	PRINTF("%ld\n", val);
	oldval = val;
}
SubscriberReceiver<uint32_t> naneNotImportant02(topicCounter1, printMsg, "readerfunc");

void setFlag(int *flag) {
	send_telemetry = *flag;
}




