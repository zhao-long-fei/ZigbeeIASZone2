/***************************************************************************//**
 * @file
 * @brief Callback implementation for ZigbeeMinimal sample application.
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

// This callback file is created for your convenience. You may add application
// code to this file. If you regenerate this file over a previous version, the
// previous version will be overwritten and any code you have added will be
// lost.

#include "app/framework/include/af.h"
#include "gpiointerrupt.h"
#include "em_iadc.h"


EmberEventControl IADCCollectEventControl;

// Set CLK_ADC to 10kHz (this corresponds to a sample rate of 1ksps)
#define CLK_SRC_IADC_FREQ        1000000 // CLK_SRC_IADC
#define CLK_ADC_FREQ            10000   // CLK_ADC
// When changing GPIO port/pins above, make sure to change xBUSALLOC macro's
// accordingly.
#define IADC_INPUT_BUS          CDBUSALLOC
#define IADC_INPUT_BUSALLOC     GPIO_CDBUSALLOC_CDEVEN0_ADC0

// Stores latest ADC sample and converts to volts
static volatile IADC_Result_t sample;
static volatile double singleResult;



void halInitGpioInterrupt(void);
void IadcInitialize(void);
/** @brief Stack Status
 *
 * This function is called by the application framework from the stack status
 * handler.  This callbacks provides applications an opportunity to be notified
 * of changes to the stack status and take appropriate action.  The return code
 * from this callback is ignored by the framework.  The framework will always
 * process the stack status after the callback returns.
 *
 * @param status   Ver.: always
 */
bool emberAfStackStatusCallback(EmberStatus status)
{
  // This value is ignored by the framework.
  return false;
}

/** @brief Complete
 *
 * This callback is fired when the Network Steering plugin is complete.
 *
 * @param status On success this will be set to EMBER_SUCCESS to indicate a
 * network was joined successfully. On failure this will be the status code of
 * the last join or scan attempt. Ver.: always
 * @param totalBeacons The total number of 802.15.4 beacons that were heard,
 * including beacons from different devices with the same PAN ID. Ver.: always
 * @param joinAttempts The number of join attempts that were made to get onto
 * an open Zigbee network. Ver.: always
 * @param finalState The finishing state of the network steering process. From
 * this, one is able to tell on which channel mask and with which key the
 * process was complete. Ver.: always
 */
void emberAfPluginNetworkSteeringCompleteCallback(EmberStatus status,
                                                  uint8_t totalBeacons,
                                                  uint8_t joinAttempts,
                                                  uint8_t finalState)
{
  emberAfCorePrintln("%p network %p: 0x%X", "Join", "complete", status);
}



/** @brief Ok To Sleep
 *
 * This function is called by the Idle/Sleep plugin before sleeping. It is
 * called with interrupts disabled. The application should return true if the
 * device may sleep or false otherwise.
 *
 * @param durationMs The maximum duration in milliseconds that the device will
 * sleep. Ver.: always
 */
bool emberAfPluginIdleSleepOkToSleepCallback(uint32_t durationMs)
{
  halInternalDisableWatchDog(MICRO_DISABLE_WATCH_DOG_KEY);
  return true;
}


/** @brief Wake Up
 *
 * This function is called by the Idle/Sleep plugin after sleeping.
 *
 * @param durationMs The duration in milliseconds that the device slept.
 * Ver.: always
 */
void emberAfPluginIdleSleepWakeUpCallback(uint32_t durationMs)
{
  halInternalEnableWatchDog();

}

void emberAfMainInitCallback(void)
{
  halInitGpioInterrupt();
  IadcInitialize();
  emberEventControlSetDelayMS(IADCCollectEventControl,5000);
}

void gpioInterruptcallback(uint8_t intNo)
{
  emberAfCorePrintln("gpioInterruptcallback %d.",intNo);
  uint8_t state = 0;
  uint8_t data[2] = {0,0};
  uint8_t ieeeAddress[] = { 0, 0, 0, 0, 0, 0, 0, 0 };
  //IADC_command(IADC0, iadcCmdStartSingle);
  emberAfReadServerAttribute(1,
                             ZCL_IAS_ZONE_CLUSTER_ID,
                             ZCL_IAS_CIE_ADDRESS_ATTRIBUTE_ID,
                             (uint8_t*)ieeeAddress,
                             8);
  emberAfReadServerAttribute(1,
                             ZCL_IAS_ZONE_CLUSTER_ID,
                             ZCL_ZONE_STATE_ATTRIBUTE_ID,
                             (uint8_t*)&state,
                             8);
  for (int i = 0; i < 8; i++) {
    if (ieeeAddress[i] != 0) {
        emberAfCorePrintln("addr%d=%4x",i,ieeeAddress[i]);
    }
  }
  emberAfCorePrintln("state=%x ",state);
  emberAfReadServerAttribute(1,
                             ZCL_IAS_ZONE_CLUSTER_ID,
                             ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                             (uint8_t*)data,
                             8);

  uint16_t data16_t;
  data[0] = (data[0]==0x01 ? 0x00 :0x01 );
  data16_t = ((data[0]) & 0x00ff) +(data[1]<<8 & 0xff00);
  emberAfCorePrintln("data[0]=%x data[1]=%x data16_t=%x ",data[0],data[1],data16_t);
  emberAfWriteServerAttribute(1,
                              ZCL_IAS_ZONE_CLUSTER_ID,
                              ZCL_ZONE_STATUS_ATTRIBUTE_ID,
                              (uint8_t*)data,
                              ZCL_BITMAP16_ATTRIBUTE_TYPE);
  //sendZoneUpdate(0x0001, 0, 1);
  emberAfFillCommandIasZoneClusterZoneStatusChangeNotification(
      data16_t,
    0, // extended status, must be zero per spec
    emberAfPluginIasZoneServerGetZoneId(1),
    0); // called "delay" in the spec
  emberAfSetCommandEndpoints(1, 1);
  emberAfSendCommandUnicastToBindings();



}

void halInitGpioInterrupt(void)
{
  GPIOINT_IrqCallbackPtr_t isr ;
  isr = &gpioInterruptcallback;
  GPIOINT_Init();
  CMU_ClockEnable(cmuClock_GPIO, true);

  GPIO_PinModeSet(gpioPortB, 1,gpioModeInputPull,0);

  GPIOINT_CallbackRegister(0,isr);      //中断号，回调函数指针
  GPIO_ExtIntConfig(gpioPortB,          //GPIO_Port_TypeDef port
                         1,             //unsigned int pin
                         0,             //unsigned int intNo
                         false,         //bool risingEdge
                         true,          //bool fallingEdge
                         true);         //bool enable


}

void IadcInitialize(void)
{
  // Declare init structs
  IADC_Init_t init = IADC_INIT_DEFAULT;
  IADC_AllConfigs_t initAllConfigs = IADC_ALLCONFIGS_DEFAULT;
  IADC_InitSingle_t initSingle = IADC_INITSINGLE_DEFAULT;
  IADC_SingleInput_t initSingleInput = IADC_SINGLEINPUT_DEFAULT;


  // Reset IADC to reset configuration in case it has been modified
  IADC_reset(IADC0);
  // Configure IADC clock source for use while in EM2
  // Note that HFRCOEM23 is the lowest frequency source available for the IADC
  CMU_ClockSelectSet(cmuClock_IADCCLK, cmuSelect_HFRCOEM23);

  // Modify init structs and initialize
  init.warmup = iadcWarmupKeepWarm;
  // Set the HFSCLK prescale value here
  init.srcClkPrescale = IADC_calcSrcClkPrescale(IADC0,CLK_SRC_IADC_FREQ, 0);

  // Configuration 0 is used by both scan and single conversions by default
  // Use unbuffered AVDD as reference
  initAllConfigs.configs[0].reference = iadcCfgReferenceVddx;
  initAllConfigs.configs[0].adcClkPrescale = IADC_calcAdcClkPrescale(IADC0,
                                                                     CLK_ADC_FREQ,
                                                                     0,
                                                                     iadcCfgModeNormal,
                                                                     init.srcClkPrescale);
  // Single initialization
  initSingle.dataValidLevel = _IADC_SCANFIFOCFG_DVL_VALID1;
  // Set conversions to run one,continuously
  initSingle.triggerAction = iadcTriggerActionOnce;
  // Configure Input sources for single ended conversion
  initSingleInput.posInput = iadcPosInputPortCPin4;
  initSingleInput.negInput = iadcNegInputGnd;

  // Initialize the ADC peripheral
  IADC_init(IADC0, &init, &initAllConfigs);
  // Initialize Scan
  IADC_initSingle(IADC0, &initSingle,&initSingleInput);

  // Allocate the analog bus for ADC0 inputs
  GPIO->IADC_INPUT_BUS |= IADC_INPUT_BUSALLOC;

  // Enable interrupts on data valid level
  IADC_enableInt(IADC0, IADC_IF_SINGLEFIFODVL);

  // Enable ADC interrupts
  NVIC_ClearPendingIRQ(IADC_IRQn);
  NVIC_EnableIRQ(IADC_IRQn);
}

/**************************************************************************//**
 * @brief  ADC Handler
 *****************************************************************************/
void IADC_IRQHandler(void)
{
  uint8_t voltage;

  // Read data from the FIFO
  sample = IADC_pullSingleFifoResult(IADC0);
  emberAfCorePrintln("sample.data=%d ",sample.data);
  // For single-ended the result range is 0 to +Vref, i.e., 12 bits for the
  // conversion value.
  singleResult = sample.data * 3.3 / 0xFFF;
  emberAfCorePrintln("singleResult=%d ",(int)singleResult*10);
  //emberAfCorePrintln("singleResult1=%e ",singleResult);
  IADC_clearInt(IADC0, IADC_IF_SINGLEFIFODVL);
  voltage = sample.data*200/0xFFF;
  emberAfWriteServerAttribute(1,
                             ZCL_POWER_CONFIG_CLUSTER_ID,
                             ZCL_BATTERY_PERCENTAGE_REMAINING_ATTRIBUTE_ID,
                             (uint8_t*)&voltage,
                             ZCL_DATA8_ATTRIBUTE_TYPE);
}

void IADCCollectEventHandler(void)
{
  emberEventControlSetInactive(IADCCollectEventControl);
  IADC_command(IADC0, iadcCmdStartSingle);
  emberEventControlSetDelayMS(IADCCollectEventControl,5000);
}

