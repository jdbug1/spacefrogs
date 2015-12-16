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

using namespace RODOS;

class WifiGateway : public Thread {
public:
  WifiGateway() : Thread("WifiGateway") {

  }

  void init() {
	  gateway1.addTopicsToForward(&topicCounter1, &imu_topic, &ahrs_topic, &light_topic);
  }

  void run() {
	  uint32_t cnt=0;

	  wf121.init("YETENet","yeteyete");
	  wf121.enableUDPConnection(0xFF01A8C0,37647);
	  while (1) {
		  cnt++;
		  suspendCallerUntil(NOW()+1*SECONDS);
		  //topicCounter1.publish(cnt);
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






