
#include "ccs32_globals.h"
#include "main.h"

/* How to use CAN bus on the STM32?
Explanation e.g. here: https://controllerstech.com/can-protocol-in-stm32/
1. In the CubeIDE, open the .ioc file and configure the CAN.
2. In the Communication/CAN chapter, configure the prescaler and time quanta according to the
intended baud rate. Calculator: http://www.bittiming.can-wiki.info/
3. The configuration tool generates the main.c, which defines and calls MX_CAN_Init().
4. According to https://www.st.com/resource/en/user_manual/um1850-description-of-stm32f1-hal-and-lowlayer-drivers-stmicroelectronics.pdf
   we need to configure the filters: HAL_CAN_ConfigFilter()
   And finally Start the CAN module using HAL_CAN_Start() function. At this level the node is active on the bus: it receive
messages, and can send messages.
4. Hardware connection:
PA12 = CAN_TX = MCP2551.1
PA11 = CAN_RX = MCP2551.4
5. Transmit a frame using HAL_CAN_AddTxMessage()

*/

CAN_TxHeaderTypeDef   TxHeader;
uint8_t               TxData[8];
uint32_t              TxMailbox;
uint8_t canbus_divider100ms_to_1s;
uint8_t canbus_divider10ms_to_1s;
int16_t canDebugValue1, canDebugValue2, canDebugValue3, canDebugValue4;

void canbus_demoTransmit(void) {
  long int rc;
  uint32_t uptime_s;
  uptime_s = HAL_GetTick() / 1000; /* the uptime in seconds */
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x567;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;

  TxData[0] = (uint8_t)(uptime_s>>16);
  TxData[1] = (uint8_t)(uptime_s>>8);
  TxData[2] = (uint8_t)(uptime_s);
  TxData[3] = (uint8_t)(checkpointNumber>>8);
  TxData[4] = (uint8_t)(checkpointNumber);
  TxData[5] = cpDuty_Percent;
  TxData[6] = 0xFF;
  TxData[7] = 0xFF;
  
  rc = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
  if (rc != HAL_OK)
  {
   // Transmit did not work -> Error_Handler();
   sprintf(strTmp, "HAL_CAN_AddTxMessage failed %ld", rc);
   addToTrace(strTmp);
  } else {
   //sprintf(strTmp, "HAL_CAN_AddTxMessage ok for mailbox %ld", TxMailbox);
   //addToTrace(strTmp);
  }

}

void canbus_transmitFoccciFast01(void) {
  long int rc;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x568;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;

  TxData[0] = (uint8_t)(EVSEPresentVoltage>>8);
  TxData[1] = (uint8_t)(EVSEPresentVoltage);
  TxData[2] = (uint8_t)(uCcsInlet_V>>8);
  TxData[3] = (uint8_t)(uCcsInlet_V);
  TxData[4] = 0;
  TxData[5] = 0;
  TxData[6] = 0;
  TxData[7] = 0;

  rc = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
  if (rc != HAL_OK)
  {
   // Transmit did not work -> Error_Handler();
  }
}


void canbus_demoTransmitFoccciTemperatures01_569(void) {
  /* Temperature signals */
  long int rc;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x569;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;

  TxData[0] = temperatureCpu_M40;
  TxData[1] = temperatureChannel_1_M40;
  TxData[2] = temperatureChannel_2_M40;
  TxData[3] = temperatureChannel_3_M40;
  TxData[4] = 0;
  TxData[5] = 0;
  TxData[6] = 0;
  TxData[7] = 0;

  rc = HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
  if (rc != HAL_OK)
  {
   // Transmit did not work -> Error_Handler();
   //sprintf(strTmp, "HAL_CAN_AddTxMessage failed %ld", rc);
   //addToTrace(strTmp);
  } else {
   //sprintf(strTmp, "HAL_CAN_AddTxMessage ok for mailbox %ld", TxMailbox);
   //addToTrace(strTmp);
  }

}

void canbus_demoTransmit56A(void) {
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x56A;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxData[0] = (uint8_t)(canDebugValue1>>8);
  TxData[1] = (uint8_t)(canDebugValue1);
  TxData[2] = (uint8_t)(canDebugValue2>>8);
  TxData[3] = (uint8_t)(canDebugValue2);
  TxData[4] = (uint8_t)(canDebugValue3>>8);
  TxData[5] = (uint8_t)(canDebugValue3);
  TxData[6] = (uint8_t)(canDebugValue4>>8);
  TxData[7] = (uint8_t)(canDebugValue4);
  (void)HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}



void canbus_Init(void) {
	HAL_CAN_Start(&hcan);
}

void canbus_Mainfunction10ms(void) {
    /* The canbus_divider10ms_to_1s runs from 0 to 99. */

    if ((canbus_divider10ms_to_1s % 10)==0) { canbus_transmitFoccciFast01(); }

    if ((canbus_divider10ms_to_1s % 50)==1) { canbus_demoTransmit(); }
    if ((canbus_divider10ms_to_1s % 50)==3) { canbus_demoTransmit56A(); }
    if (canbus_divider10ms_to_1s ==4) canbus_demoTransmitFoccciTemperatures01_569(); /* send temperatures in the slow 1s interval */

    canbus_divider10ms_to_1s++;
    if (canbus_divider10ms_to_1s >= 100) {
    	canbus_divider10ms_to_1s=0;
    }
}

void canbus_Mainfunction100ms(void) {
}
