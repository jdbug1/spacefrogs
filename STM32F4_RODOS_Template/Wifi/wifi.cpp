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
	  tmStructIMU tm_imu;
	  tmStructElectrical tm_electrical;

	  imuData imu;
	  imuPublish ahrs;
	  electricalStruct electrical;

	  wf121.init("YETENet","yeteyete");
	  wf121.enableUDPConnection(0xFF01A8C0,37647);
	  while (1) {
		  if (send_telemetry) {
			  imuBuffer.get(imu);
			  ahrsBuffer.get(ahrs);
			  electricalBuffer.get(electrical);

			  cnt++;
			  tm_imu.ax = imu.ax;
			  tm_imu.ay = imu.ay;
			  tm_imu.az = imu.az;
			  tm_imu.mx = imu.mx;
			  tm_imu.my = imu.my;
			  tm_imu.mz = imu.mz;
			  tm_imu.wx = imu.wx;
			  tm_imu.wy = imu.wy;
			  tm_imu.wz = imu.wz;

			  tm_imu.roll = ahrs.roll;
			  tm_imu.pitch = ahrs.pitch;
			  tm_imu.heading = ahrs.heading;

			  tm_electrical.knife = electrical.knife_status;
			  tm_electrical.em = electrical.em_status;
			  tm_electrical.light = electrical.light_status;
			  tm_electrical.light_value = electrical.light_raw;

			  topicCounter1.publish(cnt);
			  tm_topic_imu.publish(tm_imu);
			  tm_topic_electrical.publish(tm_electrical);

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




