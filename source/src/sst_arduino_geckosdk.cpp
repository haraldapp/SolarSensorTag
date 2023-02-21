//- -----------------------------------------------------------------------------------------------------------------------
// SST Solar Sensor Tag
// 2022-09-01 haraldapp Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// file: sst_arduino_geckosdk.cpp
// API with ARDUINO like functions used by ASKSINPP based EFM32 GeckoSDK functions

#include <stdarg.h>
#include "sst_arduino_geckosdk.h"

#include <sst_em_device.h> // em_device like includes for SST supported mcu chips
// Gecko SDK platform/emlib/inc
#include "em_chip.h"
#include "em_core.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_system.h"
#include "em_gpio.h"
#include "em_usart.h"
#include "em_timer.h"
#include "em_rtc.h"
#include "em_adc.h"
#include "em_i2c.h"
#include "em_rmu.h"
#include "em_wdog.h"
#include "em_dbg.h"
// #include "em_int.h"
// Gecko SDK platform/emdrv
//    common/inc
//    spidrv/inc
//    dmadrv/inc
#include "spidrv.h"
// Gecko SDK hardware/kit/common/drivers
#include "i2cspm.h" // i2c simply polled master
#include "udelay.h"

volatile uint8_t _sst_savepower_mode=0; // current em sleep mode
volatile uint16_t _sst_savepower_wakeupevent=0;

// GPIO
void pinMode(uint8_t apin, uint8_t amode)
{
	// SSTPRINTS( "pinMode(" ); SSTPRINTPIN(apin); SSTPRINTS( "," ); SSTPRINTI(amode); SSTPRINTS( ")\n" );
	CMU_ClockEnable(cmuClock_GPIO,true);
	if ( amode==INPUT )
	   GPIO_PinModeSet( PORT_APIN(apin), PIN_APIN(apin), gpioModeInput, 0 );
	else if ( amode==INPUT_PULLUP )
       GPIO_PinModeSet( PORT_APIN(apin), PIN_APIN(apin), gpioModeInputPull, 1 );
	else if ( amode==INPUT_PULLDOWN )
       GPIO_PinModeSet( PORT_APIN(apin), PIN_APIN(apin), gpioModeInputPull, 0 );
	else if ( amode==OUTPUT && GPIO_PinModeGet(PORT_APIN(apin), PIN_APIN(apin))!=gpioModePushPull )
       GPIO_PinModeSet( PORT_APIN(apin), PIN_APIN(apin), gpioModePushPull, 0 );
}

int digitalRead(uint8_t apin)
{
    return GPIO_PinInGet( PORT_APIN(apin), PIN_APIN(apin) );
}

void digitalWrite(uint8_t apin, uint8_t val)
{
  if ( apin!=SPI_SS_APIN )  {
  // SSTPRINTS( "digitalWrite(" ); SSTPRINTPIN(apin); SSTPRINTS( "," ); SSTPRINTU(val); SSTPRINTS( ")\n" );
  }
  if ( apin==SPI_PWR_APIN )  { val=(!val); }
  if (val)   GPIO_PinOutSet( PORT_APIN(apin), PIN_APIN(apin) );
  else       GPIO_PinOutClear( PORT_APIN(apin), PIN_APIN(apin) );
}

static uint8_t _sst_adc_reference=0;

void analogReference(uint8_t mode)
{
  _sst_adc_reference=(uint8_t)mode;
}

int analogGetReference(void)
{
  return _sst_adc_reference;
}

int analogRead(uint8_t apin)
{
	uint8_t adcinput=SST_ADC_APinToInput( apin );
	if ( adcinput==SST_ADC_INPUTSEL_NONE )  return 0;
	uint8_t adcres=SST_ADC_RES_OVS;
	uint16_t val=0; SST_ADC_Measure( val, adcinput, _sst_adc_reference, adcres );
	return val;
}

void analogWrite(uint8_t apin, int val)
{
	// TODO implement by PWM or IDAC
}

void SST_GPIO_PinModeSet( uint8_t port, uint8_t pin, uint8_t mode, uint8_t out )
{
  // SSTPRINTS( "GPIO_PinModeSet(" ); SSTPRINTPIN(MAKE_APIN(port,pin)); SSTPRINTS( "," ); SSTPRINTI(mode); SSTPRINTS( "," ); SSTPRINTI(out); SSTPRINTS( ")\n" );
  GPIO_PinModeSet( (GPIO_Port_TypeDef)port, pin, (GPIO_Mode_TypeDef)mode, out );
}


// Interrupts
//   at the moment only supporting intterrupts on INT0 = PA0 and INT1 = PA1
typedef struct { uint8_t apin; uint32_t mask; voidFuncPtr cb; } SST_INT_STATUS;
SST_INT_STATUS _sst_int_status[2]={{0},{0}};
enum { SST_INT_EVEN=0, SST_INT_ODD=1 };
inline SST_INT_STATUS& sst_int_status_from_apin( uint8_t apin )  { return ((apin&1) ? _sst_int_status[SST_INT_ODD] : _sst_int_status[SST_INT_EVEN]); }
inline uint8_t sst_int_status_index_from_apin( uint8_t apin )  { return (apin&1) ? SST_INT_ODD : SST_INT_EVEN; }

void sst_gpio_irqhandler( uint8_t int_type ) {
  SST_INT_STATUS& intstatus=_sst_int_status[int_type];
  SSTPRINTF( _F(" Int GPIO %a\n"), intstatus.apin );
  if ( _sst_savepower_mode )   _sst_savepower_wakeupevent|=(int_type==SST_INT_EVEN?SST_SAVEPOWER_WAKEUP_GPIO_EVEN:SST_SAVEPOWER_WAKEUP_GPIO_ODD);
  uint32_t flags = GPIO_IntGet();
  if ( (intstatus.mask&flags) && (intstatus.cb) )  (*(intstatus.cb))();
  GPIO_IntClear( flags );
}

extern "C" void GPIO_EVEN_IRQHandler( void )  { sst_gpio_irqhandler(SST_INT_EVEN); }
extern "C" void GPIO_ODD_IRQHandler( void )    { sst_gpio_irqhandler(SST_INT_ODD); }

bool SST_GPIO_WakeUpFromPin( uint8_t apin )
{
  if ( (apin&1) && (_sst_savepower_wakeupevent&SST_SAVEPOWER_WAKEUP_GPIO_ODD) && _sst_int_status[SST_INT_ODD].apin==apin )
    return true;
  if ( !(apin&1) && (_sst_savepower_wakeupevent&SST_SAVEPOWER_WAKEUP_GPIO_EVEN) && _sst_int_status[SST_INT_EVEN].apin==apin )
    return true;
  return false;
}

void enableInterrupt( uint8_t apin, voidFuncPtr callback, PinStatus mode )
{
	SSTPRINTF( "enableInterrupt(%a,%p,%u)\n", apin, callback, mode );
	NVIC_EnableIRQ(GPIO_ODD_IRQn); NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	SST_INT_STATUS& int_status=sst_int_status_from_apin( apin );
	if ( int_status.mask )   GPIO_ExtIntConfig( PORT_APIN(int_status.apin), PIN_APIN(int_status.apin), PIN_APIN(int_status.apin), false, false, false );
	int_status.apin=apin; int_status.mask=(1 << PIN_APIN(apin)); int_status.cb=callback;
	GPIO_ExtIntConfig( PORT_APIN(apin), PIN_APIN(apin), PIN_APIN(apin), (mode==CHANGE || mode==RISING), (mode==CHANGE || mode==FALLING), true );
}

void disableInterrupt( uint8_t apin )
{
	SSTPRINTF( "disableInterrupt(%a)\n", apin );
	SST_INT_STATUS& int_status=sst_int_status_from_apin( apin );
	if ( int_status.mask )   GPIO_ExtIntConfig( PORT_APIN(int_status.apin), PIN_APIN(int_status.apin), PIN_APIN(int_status.apin), false, false, false );
	int_status.apin=0; int_status.mask=0; int_status.cb=(voidFuncPtr)0;
}

void attachInterrupt(pin_size_t interruptNumber, voidFuncPtr callback, PinStatus mode)
{
	SSTPRINTF( "attachInterrupt(%u,%x,%u) - not supported\n", interruptNumber, (uint32_t)callback, mode );
}

void detachInterrupt(pin_size_t interruptNumber)
{
	SSTPRINTF( "disableInterrupt(%u) - not supported\n", interruptNumber );
}

int8_t digitalPinToInterrupt(uint8_t apin)
{
  return NOT_AN_INTERRUPT; // -> use enableInterrupt
}

// Delay, ticks in milliseconds (trough SysTick_Handler)
volatile uint32_t g_msTicks=0;
#ifndef NDEBUG
volatile uint32_t g_msTicksNDebug=0;
volatile uint8_t g_msTicksDebugStop=0;
#else
#define g_msTicksNDebug g_msTicks
#endif
#ifndef SST_SYSTICK_CB_CNT
#define SST_SYSTICK_CB_CNT 3 // default used by asksin sysclock, SSTRadioFreqAdjust, BatteryOverchargeProtect
#endif
typedef struct { void *callback; uint32_t tickcb; uint32_t tickcur; void *param; } SSTSYSTICKCALLBACKENTRY;
volatile SSTSYSTICKCALLBACKENTRY g_msSysTickCallbacks[SST_SYSTICK_CB_CNT]={0};

static volatile uint32_t AddTickCount( uint32_t add )
{
  volatile uint32_t ret;
  CORE_ATOMIC_SECTION( g_msTicks+=add; ret=g_msTicks; )
  return ret;
}

extern "C" void SysTick_Handler(void) // overwrites empty weak geckosdk function
{
  if ( _sst_savepower_mode )   _sst_savepower_wakeupevent|=SST_SAVEPOWER_WAKEUP_SYSTICK;
  AddTickCount(1); // increment 1ms tick counter
  #ifndef NDEBUG
  if ( !g_msTicksDebugStop )   g_msTicksNDebug++; // tick counter w/o  debug output
  #endif
  for ( uint8_t u=0; u<SST_SYSTICK_CB_CNT; u++ )
  {
    volatile SSTSYSTICKCALLBACKENTRY& cb=g_msSysTickCallbacks[u]; void *run_callback=0; void *run_param=0;
    CORE_ATOMIC_SECTION(
      if ( cb.callback ) {
        cb.tickcur++;
        if ( cb.tickcur>=cb.tickcb )  { cb.tickcur=0; run_callback=cb.callback; run_param=cb.param; }
      }
    )
    if ( !run_callback )   continue;
    if ( run_param )   (*(voidFuncPtrParam)run_callback)( run_param );
    else               (*(voidFuncPtr)run_callback)();
  }
}

uint32_t GetTickCount(void)
{
  return AddTickCount(0);
}

void Delay(uint32_t dlyTicks)
{
  uint32_t curTicks=GetTickCount();
  while ((GetTickCount()-curTicks) < dlyTicks) ;
}


// Delay, ticks in mircoseconds (trough instruction loop, see gecko sdk udelay)
void DelayMicroseconds(uint32_t usec)
{
  UDELAY_Delay(usec);
}

bool SST_SYSTICK_AddCallback( voidFuncPtr callback, uint32_t ticks_ms )
{
   return SST_SYSTICK_AddCallbackParam( (voidFuncPtrParam)callback, ticks_ms, 0 );
}

bool SST_SYSTICK_AddCallbackParam( voidFuncPtrParam callback, uint32_t ticks_ms, void *param )
{
  bool ret=false;
  CORE_ATOMIC_SECTION(
  for ( uint8_t u=0; u<SST_SYSTICK_CB_CNT && !ret; u++ )
  {
    volatile SSTSYSTICKCALLBACKENTRY& cb=g_msSysTickCallbacks[u];
    if ( !(cb.callback==0 || cb.callback==(void*)callback) )  continue;
    cb.tickcur=0; cb.tickcb=ticks_ms; cb.callback=(void*)callback; cb.param=param;
    ret=true;
  }
  )
  return ret;
}

bool SST_SYSTICK_RemoveCallback( void *callback )
{
  bool ret=false;
  CORE_ATOMIC_SECTION(
  for ( uint8_t u=0; u<SST_SYSTICK_CB_CNT && !ret; u++ )
  {
    volatile SSTSYSTICKCALLBACKENTRY& cb=g_msSysTickCallbacks[u];
	if ( cb.callback!=(void*)callback )   continue;
    cb.tickcur=0; cb.tickcb=0; cb.callback=0; cb.param=0;
    ret=true;
  }
  )
  return ret;
}

// SST SavePower
uint8_t _sst_sleep_stat=0;
uint32_t _sst_run_em0_start=0;
uint32_t _sst_run_em0_ms=0;
uint32_t _sst_sleep_em2_s=0;
void sst_get_sleep_statics( uint32_t& run_em0_ms, uint32_t& sleep_em2_s )
{
  run_em0_ms=_sst_run_em0_ms; sleep_em2_s=_sst_sleep_em2_s;
}

uint16_t SST_SavePowerRTC( uint32_t wakeup_ticks, uint32_t ticks_per_second /*=100*/, uint32_t *pruntime_ms /*=0*/, uint32_t *psleeped_ticks /*=0*/  )
{
  SST_RTC_Start( wakeup_ticks, ticks_per_second );
  if ( !_sst_sleep_stat )  { _sst_sleep_stat=1; _sst_run_em0_start=0; _sst_run_em0_ms=0; _sst_sleep_em2_s=0; } // reset on first sleep period
  uint32_t runtime_ms=0; if ( _sst_run_em0_start )   runtime_ms=g_msTicksNDebug-_sst_run_em0_start;
  _sst_run_em0_ms+=runtime_ms;
  uint16_t wakeup_event=SST_SavePower();
  uint32_t ticks_sleeped=SST_RTC_GetCurrentTicks( wakeup_ticks, ticks_per_second );
  SST_RTC_Stop();
  _sst_sleep_em2_s+=ticks_sleeped/ticks_per_second;
  _sst_run_em0_start=g_msTicksNDebug;
  if ( pruntime_ms )  *pruntime_ms=runtime_ms;
  if ( psleeped_ticks )  *psleeped_ticks=ticks_sleeped;
  return wakeup_event;
}

uint16_t SST_SavePower( void )
{
  #if !(defined(NDEBUG) || defined(NDEBUG_SWO))
  SSTDisableSWODebug();
  #endif
  // disable systicks
  uint32_t sysTickCtrl=SysTick->CTRL; SysTick->CTRL&=~0x03; // clear TICKINT & ENABLE
  NVIC_ClearPendingIRQ(SysTick_IRQn);
  NVIC_DisableIRQ(SysTick_IRQn);
  // sleep
  _sst_savepower_mode=2; _sst_savepower_wakeupevent=0;
  EMU_EnterEM2(true);
  _sst_savepower_mode=0; uint16_t ret=_sst_savepower_wakeupevent;
  // enable systicks
  SysTick->CTRL=sysTickCtrl;
  #if !(defined(NDEBUG) || defined(NDEBUG_SWO))
  SSTEnableSWODebug();
  #endif
  return ret;
}

void SST_SleepForever()
{
  // sleep until INT0 / button pressed (does chip reset)
  #if !(defined(NDEBUG) || defined(NDEBUG_SWO))
  SSTDisableSWODebug();
  #endif
  GPIO_PinModeSet( INT0_PORT, INT0_PIN, gpioModeInputPullFilter, 1 );
  GPIO_EM4EnablePinWakeup( GPIO_EM4WUEN_EM4WUEN_A0, 0 );
  SST_Watchdog_Enable( false );
  EMU_EnterEM4();
}

// Reset
static uint32_t _sst_resetcause_mask=0;

void SST_SystemReset()
{
  NVIC_SystemReset();
}

uint8_t SST_GetSytemResetCause()
{
  if ( _sst_resetcause_mask&(RMU_RSTCAUSE_PORST) )   return SST_RESETCAUSE_POWERON;
  if ( _sst_resetcause_mask&(RMU_RSTCAUSE_EXTRST) )   return SST_RESETCAUSE_PIN;
  if ( _sst_resetcause_mask&(RMU_RSTCAUSE_BODUNREGRST|RMU_RSTCAUSE_BODREGRST|RMU_RSTCAUSE_BODAVDD0|RMU_RSTCAUSE_BODAVDD1) )   return SST_RESETCAUSE_BROWNOUT;
  if ( _sst_resetcause_mask&(RMU_RSTCAUSE_WDOGRST) )   return SST_RESETCAUSE_WATCHDOG;
  if ( _sst_resetcause_mask&(RMU_RSTCAUSE_LOCKUPRST) )   return SST_RESETCAUSE_LOCKUP;
  if ( _sst_resetcause_mask&(RMU_RSTCAUSE_SYSREQRST) )   return SST_RESETCAUSE_SOFT;
  if ( _sst_resetcause_mask&(RMU_RSTCAUSE_EM4RST) )   return SST_RESETCAUSE_EM4;
  return SST_RESETCAUSE_NA;
}

// Watchdog
WDOG_TypeDef _sst_watchdog_instance={0};

void SST_Watchdog_Init( uint16_t sec )
{
  // TODO: check
  // watchdog timeout spec (? has factor 8 + div): (2 ^ ( 3 + perSel ) + 1 ) / freq
  // testing around some nights... factor 2 gives reasonable time above desired seconds
  WDOG_Init_TypeDef init=WDOG_INIT_DEFAULT; // enable and start watchdog
  init.clkSel=wdogClkSelULFRCO;
  init.perSel=wdogPeriod_129;
  uint32_t freq=1000; uint32_t div=128/2; // CMU_ClockFreqGet(cmuClock_ULFRCO);
  while ( init.perSel<wdogPeriod_256k && ((div+1)/freq)<sec )  { div=(div<<1); init.perSel=(WDOG_PeriodSel_TypeDef)(((uint16_t)init.perSel)+1); }
  SSTPRINTF( "Watchdog init %u sec: freq %u per %u sec %u\n", sec, freq, (uint32_t)init.perSel, ((div+1)/freq) );
  WDOGn_Init( &_sst_watchdog_instance, &init );
}

uint8_t SST_Watchdog_Done( void )
{
  if ( !_sst_watchdog_instance.CTRL )   return 0;
  WDOGn_Enable( &_sst_watchdog_instance, false );
  CMU_ClockEnable( cmuClock_LFB, false );
  _sst_watchdog_instance.CTRL=0;
  return 1;
}

void SST_Watchdog_Enable( bool enable )
{
  if ( _sst_watchdog_instance.CTRL )  WDOGn_Enable( &_sst_watchdog_instance, enable );
}

void SST_Watchdog_Feed()
{
  if ( _sst_watchdog_instance.CTRL )  WDOGn_Feed( &_sst_watchdog_instance );
}

/*
// see emodes.c sample
static void disableHFClocks(void)
{
  // Disable High Frequency Peripheral Clocks
  CMU_ClockEnable(cmuClock_HFPER, false);
  CMU_ClockEnable(cmuClock_USART0, false);
  CMU_ClockEnable(cmuClock_USART1, false);
  CMU_ClockEnable(cmuClock_TIMER0, false);
  CMU_ClockEnable(cmuClock_TIMER1, false);
  // CMU_ClockEnable(cmuClock_CRYOTIMER, false);
  CMU_ClockEnable(cmuClock_ACMP0, false);
  // CMU_ClockEnable(cmuClock_ACMP1, false);
  CMU_ClockEnable(cmuClock_IDAC0, false);
  CMU_ClockEnable(cmuClock_ADC0, false);
  CMU_ClockEnable(cmuClock_I2C0, false);

  // Disable High Frequency Bus Clocks
  // CMU_ClockEnable(cmuClock_CRYPTO0, false);
  // CMU_ClockEnable(cmuClock_LDMA, false);
  // CMU_ClockEnable(cmuClock_GPCRC, false);
 CMU_ClockEnable(cmuClock_GPIO, false);
  CMU_ClockEnable(cmuClock_HFLE, false);
  CMU_ClockEnable(cmuClock_PRS, false);
}

static void disableLFClocks(void)
{
  // Enable LFXO for Low Frequency Clock Disables
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
  // Disable Low Frequency A Peripheral Clocks
  // Note: LFA clock must be sourced before modifying peripheral clock enables
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  // CMU_ClockEnable(cmuClock_LETIMER0, false);
  CMU_ClockEnable(cmuClock_PCNT0, false);
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_Disabled);
  // Disable Low Frequency B Peripheral Clocks
  // Note: LFB clock must be sourced before modifying peripheral clock enables
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
  CMU_ClockEnable(cmuClock_LEUART0, false);
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_Disabled);
  // Disable Low Frequency E Peripheral Clocks
  // Note: LFE clock must be sourced before modifying peripheral clock enables
  // CMU_ClockSelectSet(cmuClock_LFE, cmuSelect_LFXO);
  // CMU_ClockEnable(cmuClock_RTCC, false);
  // CMU_ClockSelectSet(cmuClock_LFE, cmuSelect_Disabled);
  // Disable Low Frequency Oscillator
  CMU_OscillatorEnable(cmuOsc_LFXO, false, true);
}
*/


// LED
void SST_LED_Init()
{
  SST_GPIO_PinModeSet( LED_R_PORT, LED_R_PIN, gpioModePushPull, 0 );
  SST_GPIO_PinModeSet( LED_G_PORT, LED_G_PIN, gpioModePushPull, 0 );
}

void SST_LED_Done()
{
  SST_GPIO_PinModeSet( LED_R_PORT, LED_R_PIN, gpioModeDisabled, 0 );
  SST_GPIO_PinModeSet( LED_G_PORT, LED_G_PIN, gpioModeDisabled, 0 );
}


// SST internal VCC
bool SST_ADC_Init( void )
{
  CMU_ClockEnable(cmuClock_ADC0, true);
  return true;
}

bool SST_ADC_Done( void )
{
  CMU_ClockEnable(cmuClock_ADC0, false);
  return true;
}

uint8_t SST_ADC_APinToInput( uint8_t apin )
{
	if ( apin==MAKE_APIN(gpioPortD,4) )   return SST_ADC_INPUTSEL_CH4;
	if ( apin==MAKE_APIN(gpioPortD,5) )   return SST_ADC_INPUTSEL_CH5;
	if ( apin==MAKE_APIN(gpioPortE,12) )   return SST_ADC_INPUTSEL_CH0;
	if ( apin==MAKE_APIN(gpioPortE,13) )   return SST_ADC_INPUTSEL_CH1;
	return SST_ADC_INPUTSEL_NONE;
}

bool SST_ADC_Measure( uint16_t& val, uint8_t adcinput, uint8_t adcref /*=SST_ADC_REF_1V25*/, uint8_t adcres /*=SST_ADC_RES_OVS*/ )
{ // see gecko sdk samples
  if ( adcinput==SST_ADC_INPUTSEL_NONE )  { val=0; return false; }
  ADC_Init_TypeDef init=ADC_INIT_DEFAULT;
  init.timebase=ADC_TimebaseCalc(0);
  init.prescale=ADC_PrescaleCalc(400000, 0);
  init.ovsRateSel=adcOvsRateSel256; // set oversampling
  ADC_Init(ADC0, &init);
  ADC_InitSingle_TypeDef singleInit=ADC_INITSINGLE_DEFAULT;
  singleInit.input=(ADC_SingleInput_TypeDef)adcinput;
  singleInit.reference=(ADC_Ref_TypeDef)adcref;
  singleInit.resolution=(ADC_Res_TypeDef)adcres;
  singleInit.acqTime=adcAcqTime32; // 32 cycles should be safe for all ADC clock frequencies
  ADC_InitSingle(ADC0, &singleInit);
  ADC_Start(ADC0, adcStartSingle);
  // poll status until done
  while (ADC0->STATUS & ADC_STATUS_SINGLEACT) ;
  // get
  val=ADC_DataSingleGet(ADC0);
  ADC_Reset(ADC0);
  return true;
}

// adcSingleInpVDDDiv3 // init for single conversion, measure VDD/3 with 1.25V reference

bool SST_ADC_MeasureInternalVCC( uint16_t& vcc )
{
  // measure
  uint16_t val=0; SST_ADC_Measure( val, adcSingleInpVDDDiv3, adcRef1V25, adcResOVS );
  vcc=(val * 1250 * 3) / 65536;  // 3,3V -> vcc=3300
  return true;
  /*
	// see gecko sdk samples
    ADC_Init_TypeDef init=ADC_INIT_DEFAULT;
    ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
    init.timebase = ADC_TimebaseCalc(0);
    init.prescale = ADC_PrescaleCalc(400000, 0);
    // set oversampling
    init.ovsRateSel = adcOvsRateSel256;
    ADC_Init(ADC0, &init);
    // init for single conversion, measure VDD/3 with 1.25V reference
    singleInit.input = adcSingleInpVDDDiv3;
    // the datasheet specifies a minimum aquisition time when sampling VDD/3
    // 32 cycles should be safe for all ADC clock frequencies
    singleInit.acqTime = adcAcqTime32;
    // enable oversampling rate
    singleInit.resolution = adcResOVS;
    ADC_InitSingle(ADC0, &singleInit);
    ADC_Start(ADC0, adcStartSingle);
    // poll status until done
    while (ADC0->STATUS & ADC_STATUS_SINGLEACT)  ;
    // calculate voltage relative based on 1.25V reference
    volatile uint16_t sampleValue = ADC_DataSingleGet(ADC0);
    vcc = (sampleValue * 1250 * 3) / 65536;  // 3,3V -> vcc=3300
    ADC_Reset(ADC0);
    */
    return true;
}

bool SST_ADC_CheckInternalVCC( uint16_t critical, uint8_t signal_apin /*=NO_APIN*/ )
{
  SST_ADC_Init();
  uint16_t vcc=0; SST_ADC_MeasureInternalVCC( vcc );
  SST_ADC_Done();
  if ( (vcc+50)/100>=critical )   return true;
  if ( signal_apin==NO_APIN )   return false;
  CMU_ClockEnable( cmuClock_GPIO, true );
  GPIO_PinModeSet( PORT_APIN(signal_apin), PIN_APIN(signal_apin), gpioModePushPull, 1 );
  for ( volatile uint32_t u=0; u<199999; u++ ) ;
  GPIO_PinModeSet( PORT_APIN(signal_apin), PIN_APIN(signal_apin), gpioModePushPull, 0 );
  GPIO_PinModeSet( PORT_APIN(signal_apin), PIN_APIN(signal_apin), gpioModeDisabled, 0 );
  return false;
}

bool SST_ADC_MeasureInternalTemp( int32_t& temp )
{
  // measure
  uint16_t val=0; SST_ADC_Measure( val, adcSingleInputTemp, adcRef1V25, adcResOVS );
  // factory calibration temperature from device information page
  int32_t cal_temp_0=((DEVINFO->CAL & _DEVINFO_CAL_TEMP_MASK) >> _DEVINFO_CAL_TEMP_SHIFT);
  int32_t cal_value_0=((DEVINFO->ADC0CAL2 & _DEVINFO_ADC0CAL2_TEMP1V25_MASK) >> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
  // calculate with temperature gradient -627/100
  temp=((cal_temp_0*10) - (((cal_value_0-val)*100*10)/-627));
  return true;
}

// RTC
void SST_RTC_Init()
{
  CMU_ClockEnable(cmuClock_HFLE, true);
  // enable LFACLK in CMU (will also enable oscillator if not enabled)
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
  // set clock divider (RTC running at 32768Hz / div)
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_8);
  // enable RTC module clock
  CMU_ClockEnable(cmuClock_RTC, true);
  // init RTC
  static const RTC_Init_TypeDef initRTC={
    false, // start counting after initialization
    false, // disable updating RTC during debug stop
    false  // count to maximum and wrap around
  };
  RTC_Init(&initRTC);
  // setup interrupts
  RTC_IntDisable(RTC_IFC_COMP0);
  SSTPRINTF( "SST_RTC_Init() freq %u\n", CMU_ClockFreqGet(cmuClock_RTC) );
}


void SST_RTC_Start( uint32_t wakeup_ticks, uint32_t tick_cnt_sec /*=100*/ )
{
  uint32_t freq=CMU_ClockFreqGet(cmuClock_LFA);
  uint32_t divider=(1000U * CMU_ClockDivGet(cmuClock_RTC));
  uint32_t cnt=(((((uint64_t)wakeup_ticks)*(1000/tick_cnt_sec) * freq) + (divider/2) ) / divider);
  if ( cnt>_RTC_COMP0_MASK )  cnt=_RTC_COMP0_MASK;
  if ( cnt==0 )  cnt=1;
  SSTPRINTF( "SST_RTC_Start freq %u ticks %u cnt %u\n", freq, wakeup_ticks, cnt );
  RTC_Enable(false);
  RTC_CompareSet(0, cnt);
  RTC_IntClear(RTC_IFC_COMP0);
  NVIC_ClearPendingIRQ(RTC_IRQn);
  RTC_IntEnable(RTC_IF_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);
  RTC_Enable(true); // will reset the counter too
}

void SST_RTC_Stop()
{
  RTC_Enable(false);
  RTC_IntClear(_RTC_IFC_MASK);
  __DSB(); // flush instructions
  RTC_IntDisable(RTC_IF_COMP0);
  NVIC_DisableIRQ(RTC_IRQn);
}

uint32_t SST_RTC_GetCurrentTicks( uint32_t wakeup_ticks, uint32_t tick_cnt_sec /*=100*/ )
{
  uint32_t cnt=RTC_CounterGet();
  if ( cnt==0 )   return wakeup_ticks;
  uint32_t freq=CMU_ClockFreqGet(cmuClock_LFA);
  uint32_t divider=CMU_ClockDivGet(cmuClock_RTC);
  // SSTPRINTF( _F(" RTC counter %u freq %u div %u\n"), cnt, freq, divider);
  uint32_t ticks=((((uint64_t)(cnt) * divider * tick_cnt_sec) + (freq/2)) / freq );
  return ticks;
}

extern "C" void RTC_IRQHandler( void )
{
  SSTPRINTS( _F(" Int RTC\n") );
  if ( _sst_savepower_mode )   _sst_savepower_wakeupevent|=SST_SAVEPOWER_WAKEUP_RTC;
  // clear interrupt
  RTC_IntClear(_RTC_IFC_MASK);
  __DSB(); // flush instructions
}


// I2C
static uint8_t _sst_i2c_optionpoweron=0xFF;
static uint8_t _sst_i2c_powerstate=0xFF;

void SST_I2C_PowerUp( uint8_t sst_opt_poweron_i2c )
{
  if ( _sst_i2c_powerstate==1 )   return; // already on
  if ( _sst_i2c_optionpoweron==0 )   { _sst_i2c_powerstate=1; return; } // power is always on
  if ( sst_opt_poweron_i2c==0 ) { // power is always on
    _sst_i2c_optionpoweron=0; _sst_i2c_powerstate=1; return;
  }
  if ( sst_opt_poweron_i2c==1 ) {
    _sst_i2c_optionpoweron=1; _sst_i2c_powerstate=0;
  }
  if ( sst_opt_poweron_i2c==2 ) {
    // try auto detect
    GPIO_Mode_TypeDef sdaPinMode=GPIO_PinModeGet( I2C_SDA_PORT, I2C_SDA_PIN );
    if ( sdaPinMode==gpioModeDisabled || sdaPinMode==gpioModeInput )  {
      SST_GPIO_PinModeSet( I2C_SDA_PORT, I2C_SDA_PIN, gpioModeInput, 0 );
      unsigned int uSDAState=GPIO_PinInGet( I2C_SDA_PORT, I2C_SDA_PIN );
      _sst_i2c_optionpoweron=(uSDAState==0?1:0); _sst_i2c_powerstate=(uSDAState==0?0:1);
    }
    else { // assume sst i2c_optionpoweron is installed
      _sst_i2c_optionpoweron=1; _sst_i2c_powerstate=0;
    }
  }
  if ( _sst_i2c_powerstate!=1 ) {
     SST_GPIO_PinModeSet( I2C_PWR_PORT, I2C_PWR_PIN, gpioModePushPull, 1 );
     _sst_i2c_powerstate=1;
     Delay(10);
  }
}

uint8_t SST_I2C_PowerDown( void )
{
  if ( _sst_i2c_optionpoweron==0 )   return 0;
  SST_GPIO_PinModeSet( I2C_PWR_PORT, I2C_PWR_PIN, gpioModePushPull, 0 );
  DelayMicroseconds(30);
  SST_GPIO_PinModeSet( I2C_PWR_PORT, I2C_PWR_PIN, gpioModeDisabled, 0 );
  _sst_i2c_powerstate=0; return _sst_i2c_optionpoweron;
}

void SST_I2C_Init( void )
{
  I2CSPM_Init_TypeDef init=
    { SST_I2C_INSTANCE,         // I2C instance
      I2C_SCL_PORT,             // SCL port
      I2C_SCL_PIN,              // SCL pin
	  I2C_SDA_PORT,             // SDA port
	  I2C_SDA_PIN,              // SDA pin
	  SST_I2C_LOCATION,         // Location
      0,                        // Use currently configured reference clock
      I2C_FREQ_STANDARD_MAX,    // Set to standard rate
      i2cClockHLRStandard,      // Set to use 4:4 low/high duty cycle
  };
  I2CSPM_Init( &init ); // setup location and pins
}

void SST_I2C_Done( void )
{
  GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeWiredAndPullUp, 1);
  GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModeWiredAndPullUp, 1);
  DelayMicroseconds(30);
  GPIO_PinModeSet(I2C_SCL_PORT, I2C_SCL_PIN, gpioModeDisabled, 0);
  GPIO_PinModeSet(I2C_SDA_PORT, I2C_SDA_PIN, gpioModeDisabled, 0);
}


bool SST_I2C_Present( uint8_t i2c_id )
{
   I2C_TransferSeq_TypeDef seq; uint8_t offsetData[2]={0,0};
   // send just dummy offset and check for ack
   seq.addr=i2c_id; seq.flags=I2C_FLAG_WRITE;
   seq.buf[0].data=offsetData; seq.buf[0].len=2;
   seq.buf[1].data=(uint8_t*)0; seq.buf[1].len=0;
   I2C_TransferReturn_TypeDef ret=I2CSPM_Transfer( SST_I2C_INSTANCE, &seq );
   return (ret==i2cTransferDone);
}

uint8_t SST_I2C_Transfer( uint8_t i2c_id, uint8_t i2c_transfer_type, const uint8_t *cmd, uint8_t cmdlen, uint8_t *data, uint16_t datalen )
{
	I2C_TransferSeq_TypeDef seq;
    seq.addr=i2c_id; seq.flags=i2c_transfer_type;
    seq.buf[0].data=(uint8_t *)cmd; seq.buf[0].len=cmdlen;
    seq.buf[1].data=data; seq.buf[1].len=datalen;
    I2C_TransferReturn_TypeDef ret=I2CSPM_Transfer( SST_I2C_INSTANCE, &seq );
    return (uint8_t)ret;
}

uint8_t SST_I2C_WaitBusy( uint8_t i2c_id, uint16_t max_time_ms, uint16_t poll_time_ms )
{
   I2C_TransferReturn_TypeDef ret; I2C_TransferSeq_TypeDef seq; uint16_t waittime=0;
   seq.addr=i2c_id; seq.flags=I2C_FLAG_WRITE;
   seq.buf[0].data=(uint8_t*)0; seq.buf[0].len=0; seq.buf[1].data=(uint8_t*)0; seq.buf[1].len=0;
   while ( true ) {
     ret=I2CSPM_Transfer( SST_I2C_INSTANCE, &seq );
     if ( ret==i2cTransferDone || ret!=i2cTransferNack )   break;
     if ( waittime>=max_time_ms )   return SST_ERROR_CODE_COMM_TIMEOUT;
     Delay( poll_time_ms ); waittime+=poll_time_ms;
   }
   return (uint8_t)ret;
}

bool SST_I2C_RetIsNack( uint8_t ret )
{
	return ret==i2cTransferNack;
}


// SPI

SPIDRV_HandleData_t _sst_spi_spidrvhandleData;

uint8_t SST_SPI_Init( uint32_t bitrate /*=2000000*/, uint8_t bitorder /*=SST_SPI_BITORDER_MSBFIRST*/ )
{
	// SST SPI uses location 3: PD6 RX, PD7 RX, PC15 CLK, PC14 /CS
	SPIDRV_Init_t init;
	init.port=USART0;                              // USART_TypeDef: USART used for SPI
    #if (SST_SPI_UART==1)
	init.port=USART1;
    #endif
	init.portLocation=SST_SPI_LOCATION;            // uint8_t: location number for SPI pins _USART_ROUTE_LOCATION_LOC3
    init.bitRate=bitrate;                          // uint32_t: SPI bitrate
    init.frameLength=8;                            // uint32_t: SPI framelength (4..16)
    init.dummyTxValue=0;                           // uint32_t: value to transmit when using SPI receive API functions
    init.type=spidrvMaster;                        // SPIDRV_Type_t: master or slave.
    init.bitOrder=(SPIDRV_BitOrder_t)bitorder;     // SPIDRV_BitOrder_t: bit order on the SPI bus (1=MSB first, 0=LSB first)
    init.clockMode=spidrvClockMode0;               // SPIDRV_ClockMode_t: SPI mode, CLKPOL/CLKPHASE setting.
    init.csControl=spidrvCsControlApplication;     // SPIDRV_CsControl_t: /< A select master mode chip select (CS) control scheme
    init.slaveStartMode=spidrvSlaveStartImmediate; // SPIDRV_SlaveStart_t: slave mode transfer start scheme
    Ecode_t ret=SPIDRV_Init( &_sst_spi_spidrvhandleData, &init );
    delayMicroseconds(50);
    return (uint8_t)ret;
}

void SST_SPI_Done()
{
	SPIDRV_DeInit( &_sst_spi_spidrvhandleData );
}

uint8_t SST_SPI_TransferSingleByte( uint8_t send, uint8_t *recv )
{
	return (uint8_t)SPIDRV_MTransferSingleItemB( &_sst_spi_spidrvhandleData, send, recv );
}

uint8_t SST_SPI_TransferMultiByte( const uint8_t *send, uint8_t *recv, int16_t count )
{
   return (uint8_t)SPIDRV_MTransferB( &_sst_spi_spidrvhandleData, send, recv, count );
}

#ifdef SST_SPI_MULTIBYTE_TRANSFER
uint8_t SST_SPI_ReceiveMultiByte( uint8_t *recv, int16_t count )
{
  return (uint8_t)SPIDRV_MReceiveB( &_sst_spi_spidrvhandleData, recv, count );
}

uint8_t SST_SPI_TransmitMultiByte( const uint8_t *bufTx, int16_t count )
{
  return (uint8_t)SPIDRV_MTransmitB( &_sst_spi_spidrvhandleData, bufTx, count);
}
#endif

SPIClass SPI;

// Unique device ID and serial number
void SST_GetDeviceIDFromChipID( uint8_t (&DeviceID)[3], char (&Serial)[11] )
{
  uint64_t chipID=SYSTEM_GetUnique(); uint8_t *pID=(uint8_t*)&chipID;
  // hash with crc32
  uint32_t crc=0xFFFFFFFF;
  for ( uint8_t i=0; i<8; i++ ) {
    uint8_t byte=pID[i];
    for (uint8_t j=0; j<8; j++) {
      uint32_t b=(byte^crc)&1;
      crc=crc>>1;
      if (b)  crc=crc^0xEDB88320;
      crc=crc>>1;
    }
  }
  crc=(~crc)&0x007FFFFFU; // max. 7 digits decimal number
  DeviceID[0]=(uint8_t)(crc&0xFFU); DeviceID[1]=(uint8_t)((crc>>8)&0xFF); DeviceID[2]=(uint8_t)((crc>>16)&0xFF);
  Serial[0]='S'; Serial[1]='S'; Serial[2]='T'; Serial[10]=0;
  for ( uint8_t i=9; i>2; i--) {
	Serial[i]=(crc%10)+'0'; crc/=10;
  }
}

// SST run mode
static uint8_t _sst_runmode=0;
uint8_t SSTGetRunMode() {
  return _sst_runmode;
}

uint8_t SSTSetRunMode( uint8_t runmode ) {
  uint8_t ret=_sst_runmode; _sst_runmode=runmode; return ret;
}


#if !(defined(NDEBUG) || defined(NDEBUG_SWO)) // all debug output not for release
// SWO debug
//   ARM cortex-m0 devices do not have program access to the ITM registers
//   so ITM/TPUI/CoreDebug cannot be used to output debug messages to the SWO pin.
//   We need a poor man workaround to see anything in the DebugViewer or ProgramOutputConsole.
//
//   The SWO stream transmits 8-bit data with a start sequence 10 and no stop sequence (other than uarts):
//      <1><0><b0><b1><b2><b3><b4><b5><b6><b7>
//   The data is packed in SWIT packets with a one byte header followed by up to four byte data:
//      Lx: data count 1=one byte, 2=two bytes, 3=four bytes
//      Ax: stimulus port/address (port 0 is used)
//      <L0><L1><0><A0><A1><A2><A3><A4> header byte
//   Testing around with some debug probes showed that some older only accept packets with one byte data.
//
//   In the poor man implementation is no calibrated timer used, so may have to adjust the bitrate in
//   SSTInitSWODebug() to get good results:
//     - Setup the debugger tracing SWO clock (eg. 30 kHz)
//     - Run SSTTestSWODebugBitrate(30000), it will write debug strings with different bitrates around 30000
//     - Check the debug output and replace the bitrate in SSTInitSWODebug() or asksin DINIT with the best result
//     - eg. DINIT(29400)
int8_t _swoMode=SST_SWODEBUG_MODE_DISABLED;
uint32_t _swoBitrate=0;
int32_t _swoBitLenUSec=0;
volatile uint32_t _swoDataOut=0; // low16: bits, high16: current count
#define SST_SWO_LINESTAT_NEWLINE   0x80
#define SST_SWO_LINESTAT_NEWLEVEL  0x40
#define SST_SWO_LINESTAT_LEVELMASK 0x0F
uint8_t _swoDebugLineStat=0;


static void sst_init_swo_timer( uint32_t bitrate )
{
  // use timer1 to shift out bits
  uint32_t refFreq=CMU_ClockFreqGet(cmuClock_HF);
  uint32_t clkdiv=CMU_ClockDivGet(cmuClock_HFPER); // eg. cmuClkDiv_1
  uint32_t timertop=(refFreq-1)/(1 * bitrate);
  // enable clock for GPIO module and TIMER1 module
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);
  // enable overflow interrupt
  TIMER_IntEnable(TIMER1, TIMER_IF_OF);
  NVIC_EnableIRQ(TIMER1_IRQn);
  NVIC_SetPriority(TIMER1_IRQn, 0); // highest
  // set TIMER top value = divisor and init timer
  TIMER_TopSet(TIMER1, timertop);
  TIMER_Init_TypeDef timerInit=TIMER_INIT_DEFAULT;
  TIMER_Init(TIMER1, &timerInit);
}

static void sst_done_swo_timer()
{
  TIMER_IntDisable(TIMER1, TIMER_IF_OF);
  NVIC_DisableIRQ(TIMER1_IRQn);
  CMU_ClockEnable(cmuClock_TIMER1, false);
}


static void sst_init_swo_delay( uint32_t bitrate )
{
  // calculate bit length in us for delay
	int32_t loopCodeExecTimeUSec=5; // assumed at default core HFRCO speed
	_swoBitLenUSec=(1000000-1)/bitrate;
	if ( _swoBitLenUSec>loopCodeExecTimeUSec )  _swoBitLenUSec-=loopCodeExecTimeUSec;
}

static void sst_shift_bit_out_d()
{
  uint32_t d=_swoDataOut; uint8_t l=(d>>16); uint16_t b=(d&0xFFFF);
  if ( l==0 )   return;
  if ( b&1 )   GPIO_PinOutSet( SWO_PORT, SWO_PIN );
  else         GPIO_PinOutClear( SWO_PORT, SWO_PIN );
  l--; if ( l==0 )  b=0;  else  b=(b>>1);
  _swoDataOut=(l<<16)|b;
}

static void sst_shift_bits_out_d( uint16_t b, uint8_t l )
{
  if ( _swoMode<=0 )   return; // debug disabled
  bool bIrqDisabled=CORE_IrqIsDisabled(); bool bIrqContext=CORE_InIrqContext();
  _swoDataOut=(l<<16)|b; // set data to send
  if ( _swoMode==SST_SWODEBUG_MODE_TIMER && !bIrqDisabled && !bIrqContext ) {
    // use timer interrupt to to shift out bits
	while ( _swoDataOut ) ; // wait done, sst_shift_bit_out_d() is call by timer IRQ
  } else {
    // use us delay to shift out bits
    while ( _swoDataOut )  { sst_shift_bit_out_d(); delayMicroseconds(_swoBitLenUSec); }
  }
  GPIO_PinOutClear( SWO_PORT, SWO_PIN );
}

extern "C" void TIMER1_IRQHandler(void)
{
  if ( _sst_savepower_mode )   _sst_savepower_wakeupevent|=SST_SAVEPOWER_WAKEUP_TIMER1;
  // clear flag for TIMER1 overflow interrupt
  TIMER_IntClear(TIMER1, TIMER_IF_OF);
  // process
  sst_shift_bit_out_d();
}

static uint8_t sst_byte_write_d( uint8_t header, const char *data, uint8_t datalen )
{
  uint8_t ret=0;
  // use delay to shift out bits
  for ( uint8_t bytenr=0; bytenr<=datalen; bytenr++ ) {
    uint16_t byte;
    if ( bytenr==0 )  { byte=header; }
    else              { byte=(*data); data++; ret++; }
    // bit sequence: 1,0,data,<0>, bit count=10
    sst_shift_bits_out_d( (byte<<2)|0x01, 10 );
  }
  return ret;
}

static uint8_t sst_handle_line_stat_d( char cout, char *bufextra, uint8_t maxlen )
{
  uint8_t is_newline=(_swoDebugLineStat&SST_SWO_LINESTAT_NEWLINE);
  uint8_t is_newlevel=(_swoDebugLineStat&SST_SWO_LINESTAT_NEWLEVEL);
  uint8_t level=(_swoDebugLineStat&SST_SWO_LINESTAT_LEVELMASK);
  uint8_t extracnt=0;
  if ( is_newlevel && !is_newline ) {
    bufextra[extracnt++]='\n'; is_newline=1;
  }
  if ( is_newline ) {
	  uint32_t u=GetTickCount(); uint32_t pwr=1000000000U;
      while ( pwr>0 ) {
        uint8_t v=(uint8_t)(u/pwr); u-=pwr*v; pwr/=10;
        bufextra[extracnt++]=v+'0';
      }
      bufextra[extracnt++]=' ';
  }
  if ( is_newline && level>0 ) {
	for ( uint8_t u=0; u<10 && u<level; u++ )
      bufextra[extracnt++]=' ';
  }
  _swoDebugLineStat&=~(SST_SWO_LINESTAT_NEWLEVEL|SST_SWO_LINESTAT_NEWLINE);
  if ( cout=='\n' )   _swoDebugLineStat|=SST_SWO_LINESTAT_NEWLINE;
  return extracnt;
}

void SSTInitSWODebug( uint32_t bitrate, int8_t mode /*=SST_SWODEBUG_MODE_TIMER*/ )
{
  _swoBitrate=bitrate; _swoMode=-mode;
  sst_init_swo_delay(bitrate);
  SSTEnableSWODebug();
}

void SSTEnableSWODebug()
{
  if ( _swoBitrate==0 || _swoMode>=0 )   return;
  GPIO_PinModeSet( SWO_PORT, SWO_PIN, gpioModePushPull, 0 );
  _swoMode=-_swoMode;
  if ( _swoMode==SST_SWODEBUG_MODE_TIMER )   sst_init_swo_timer(_swoBitrate);
  Delay( 1 );
  SSTWriteSWODebugChar( '\n' );
}

void SSTDisableSWODebug()
{
  if ( _swoMode<=0 )   return;
  if ( _swoMode==SST_SWODEBUG_MODE_TIMER )   sst_done_swo_timer();
  GPIO_PinModeSet( SWO_PORT, SWO_PIN, gpioModeInputPull, 0 ); // pull down to keep low until enable
  _swoMode=-_swoMode;
}

void SSTDoneSWODebug()
{
  SSTDisableSWODebug();
  GPIO_PinModeSet( SWO_PORT, SWO_PIN, gpioModeDisabled, 0 );
  _swoBitrate=0; _swoMode=SST_SWODEBUG_MODE_DISABLED;
}

void SSTWriteSWODebugLineLevel( uint8_t bIncrement )
{
  uint8_t level=(_swoDebugLineStat&SST_SWO_LINESTAT_LEVELMASK);
  if ( bIncrement && level<SST_SWO_LINESTAT_LEVELMASK ) {
    level++; _swoDebugLineStat|=SST_SWO_LINESTAT_NEWLEVEL;
  }
  if ( !bIncrement && level>0 ) {
	level--; _swoDebugLineStat&=~SST_SWO_LINESTAT_NEWLEVEL;
  }
  _swoDebugLineStat&=~SST_SWO_LINESTAT_LEVELMASK; _swoDebugLineStat|=level;
}

short SSTWriteSWODebugChar( char c )
{
  return SSTWriteSWODebugString( &c, 1 );
}

short SSTWriteSWODebugString( const char* buf, short len /*=-1*/ )
{
  short ret=0; if ( _swoMode==SST_SWODEBUG_MODE_DISABLED )   return ret;
  #ifndef NDEBUG
  g_msTicksDebugStop++; // do not count systicks for debug output
  #endif
  GPIO_PinOutClear( SWO_PORT, SWO_PIN ); // if left undefined
  // string zero terminated
  if ( len==-1 )   { len=0; while ( buf[len] )   len++; }
  // poor man SWIT packet emulation
  while ( len>0 ) {
    // send SWIT packet with header and packet data length 1
    //  handle debug lines and levels (insert some formatting for nested class/function calls)
    char bufextra[26]; short lenextra=sst_handle_line_stat_d( *buf, bufextra, sizeof(bufextra) );
    for (uint8_t u=0; u<lenextra; u++ )
      sst_byte_write_d( 0x01, bufextra+u, 1 );
    //  send char
    sst_byte_write_d( 0x01, buf, 1 );
    buf++; len--;
  }
  #ifndef NDEBUG
  g_msTicksDebugStop--;
  #endif
  return ret;
}

void SSTWriteSWODebugStringF( const char* fmt, ... )
{
  __gnuc_va_list ap; __builtin_va_start(ap,fmt); if ( fmt && *fmt )  SSTWriteSWODebugStringVA( fmt, ap );
}

void SSTWriteSWODebugStringVA( const char* fmt, __gnuc_va_list args )
{
	// %s: char * (0 terminated string)
	// %c: char
	// %u: unsigned integer
	// %h: unsigned integer hex
	// %x: unsigned integer "0x" hex
	// %i: integer
	// %p: pointer unsigned integer "&0x" hex
	// %%: "%"
	int iVal;
	while ( *fmt )
	{
		const char *str=fmt; uint16_t strlen=0; while ( (*fmt) && (*fmt!='%') )  { strlen++; fmt++; }
		if ( strlen>0 )   SSTWriteSWODebugString( str, strlen );
		if ( *(fmt++)!='%' )   break;
		if ( fmt[0]=='%' )       { fmt++; SSTWriteSWODebugChar('%'); }
		else if ( fmt[0]=='s' )  { fmt++; iVal=__builtin_va_arg(args,int); SSTWriteSWODebugString((const char *)iVal); }
		else if ( fmt[0]=='c' )  { fmt++; iVal=__builtin_va_arg(args,int); SSTWriteSWODebugChar((char)iVal); }
		else if ( fmt[0]=='u' )  { fmt++; iVal=__builtin_va_arg(args,int); SSTWriteSWODebugUInt((uint32_t)iVal); }
		else if ( fmt[0]=='h' )  { fmt++; iVal=__builtin_va_arg(args,int); SSTWriteSWODebugUInt((uint32_t)iVal,16); }
		else if ( fmt[0]=='x' )  { fmt++; iVal=__builtin_va_arg(args,int); SSTWriteSWODebugString(_F("0x")); SSTWriteSWODebugUInt((uint32_t)iVal,16); }
		else if ( fmt[0]=='i' )  { fmt++; iVal=__builtin_va_arg(args,int); SSTWriteSWODebugInt((int32_t)iVal); }
		else if ( fmt[0]=='p' )  { fmt++; iVal=__builtin_va_arg(args,int); SSTWriteSWODebugString(_F("&0x")); SSTWriteSWODebugUInt((uint32_t)iVal,16); }
		else if ( fmt[0]=='a' )  { fmt++; iVal=__builtin_va_arg(args,int); SSTWriteSWODebugAPin((uint8_t)iVal); }
		else                     { SSTWriteSWODebugChar('%'); }
	}
}


short SSTWriteSWODebugInt( int32_t i, uint8_t base /*=10*/ )
{
	short ret=0;
	if ( i<0 )   { ret+=SSTWriteSWODebugChar('-'); i=-i; }
	ret+=SSTWriteSWODebugUInt( ((uint32_t)i), base );
	return ret;
}

short SSTWriteSWODebugUInt( uint32_t u, uint8_t base /*=10*/, uint8_t digits /*=0*/ )
{
	short ret=0; uint32_t pwr=1000000000; uint8_t digit=10;
	if ( base==10 )       { pwr=1000000000U; digit=10; }
	else if ( base==16 )  { pwr=268435456U; digit=8; }
	else                  return 0;
	while ( digits>digit )
	{
		digits--; ret+=SSTWriteSWODebugChar( '0' );
	}
	while ( pwr>0 )
	{
		uint8_t v=(uint8_t)(u/pwr); u-=pwr*v; pwr/=base;
		if ( v!=0 )   digits=127;
		if ( v!=0 || pwr==0 || digits>=digit )
    	{
			char c=(char)(v+0x30); if ( c>=0x3A )   c+=0x41-0x3A;
			ret+=SSTWriteSWODebugChar( c );
	    }
		digit--;
	}
	return ret;
}

void SSTWriteSWODebugAPin( uint8_t apin )
{
  static const struct { uint8_t apin; const char *n; } p2n[]={
    {LED_R_APIN,"LED_R"}, {LED_G_APIN,"LED_G"},
    {INT0_APIN,"INT0"}, {INT1_APIN,"INT1"}, {INT1_APIN,"INT1"},
	{I2C_PWR_APIN,"I2C_PWR"}, {I2C_SDA_APIN,"I2C_SDA"}, {I2C_SDA_APIN,"I2C_SCL"},
	{SPI_PWR_APIN,"SPI_PWR"}, {SPI_SS_APIN,"SPI_SS"}, {SPI_SCK_APIN,"SPI_SCK"}, {SPI_MISO_APIN,"SPI_MISO"}, {SPI_MOSI_APIN,"SPI_MOSI"},
	{VBAT_M_APIN,"VBAT_M"}, {VBAT_M_ON_APIN,"VBAT_M_ON"},
	{PB0_APIN,"PB0"}, {ADC0_APIN,"ADC0"}, {PWR_ON_APIN,"PWR_ON"}, {SWO_APIN,"SWO"} };
  SSTWriteSWODebugUInt(apin, 16);
  for ( uint8_t u=0; u<sizeof(p2n)/sizeof(p2n[0]); u++ )
    if ( p2n[u].apin==apin )  {
      SSTWriteSWODebugString(_F(" ")); SSTWriteSWODebugString(p2n[u].n); break;
    }
 }

void SSTWriteSWODebugHWInfo( void )
{
	SYSTEM_PartFamily_TypeDef fam=SYSTEM_GetFamily(); uint16_t pn=SYSTEM_GetPartNumber();
	uint16_t fs=SYSTEM_GetFlashSize(); uint16_t rs=SYSTEM_GetSRAMSize(); uint32_t ps=SYSTEM_GetFlashPageSize();
	char cFam[3]={'?','?',0};
	if ( fam==_DEVINFO_PART_DEVICE_FAMILY_EFM32G )        { cFam[0]='G'; cFam[2]=0; }
	else if ( fam==_DEVINFO_PART_DEVICE_FAMILY_EFM32HG )  { cFam[0]='H'; cFam[1]='G'; }
	else if ( fam==_DEVINFO_PART_DEVICE_FAMILY_EFM32TG )  { cFam[0]='T'; cFam[1]='G'; }
	else if ( fam==_DEVINFO_PART_DEVICE_FAMILY_EFM32ZG )  { cFam[0]='Z'; cFam[1]='G'; }
	SSTWriteSWODebugStringF( _F("MCU EFM32%s%uF%u (SRAM: %ukB, FLASH: %ukB, FLASHPage: %ubyte)\n"), cFam, pn, fs, rs, fs, ps );
}

void SSTTestSWODebugBitrate( uint32_t test_bitrate, int8_t mode )
{
   uint32_t range=test_bitrate/3;
   for ( volatile uint32_t bitrate=test_bitrate-range; bitrate<test_bitrate+range; bitrate+=range/50 )
   {
      SSTInitSWODebug( bitrate, mode );
      Delay(1);
	  SSTWriteSWODebugString( _F("abc ") );
	  SSTWriteSWODebugUInt( bitrate );
	  SSTWriteSWODebugString( _F(" bitrate\n") );
	  SSTDisableSWODebug();
   }
}

// debug unused interrupt handlers
#define SST_DEBUG_IRQHANDLER(t) extern "C" void t##_IRQHandler() { SSTPRINTS(_F("IRQ - ")); SSTPRINTS(SST_STRINGIZE(t)); SSTPRINTS(_F("\n")); }
// SST_DEBUG_IRQHANDLER(DMA)          // 0 - DMA em_dma.c
// SST_DEBUG_IRQHANDLER(GPIO_EVEN)    // 1 - GPIO_EVEN sst_arduino_gecko_sdk.cpp
SST_DEBUG_IRQHANDLER(TIMER0)          // 2 - TIMER0
SST_DEBUG_IRQHANDLER(ACMP0)           // 3 - ACMP0
SST_DEBUG_IRQHANDLER(ADC0)            // 4 - ADC0
SST_DEBUG_IRQHANDLER(I2C0)            // 5 - I2C0
// SST_DEBUG_IRQHANDLER(GPIO_ODD)     // 6 - GPIO_ODD sst_arduino_gecko_sdk.cpp
// SST_DEBUG_IRQHANDLER(TIMER1)       // 7 - TIMER1 sst_arduino_gecko_sdk.cpp
SST_DEBUG_IRQHANDLER(USART1_RX)       // 8 - USART1_RX
SST_DEBUG_IRQHANDLER(USART1_TX)       // 9 - USART1_TX
SST_DEBUG_IRQHANDLER(LEUART0)         // 10 - LEUART0
SST_DEBUG_IRQHANDLER(PCNT0)           // 11 - PCNT0
// SST_DEBUG_IRQHANDLER(RTC)          // 12 - RTC sst_arduino_gecko_sdk.cpp
SST_DEBUG_IRQHANDLER(CMU)             // 13 - CMU
SST_DEBUG_IRQHANDLER(VCMP)            // 14 - VCMP
SST_DEBUG_IRQHANDLER(MSC)             // 15 - MSC
SST_DEBUG_IRQHANDLER(AES)             // 16 - AES
SST_DEBUG_IRQHANDLER(USART0_RX)       // 17 - USART0_RX
SST_DEBUG_IRQHANDLER(USART0_TX)       // 18 - USART0_TX
SST_DEBUG_IRQHANDLER(USB)             // 19 - USB
SST_DEBUG_IRQHANDLER(TIMER2)          // 20 - TIMER2

// HardFault debugging
/*
extern "C" {
static void volatile SST_DEBUG_HARDFAULT(uint32_t *sp)
{
    uint32_t r0  = sp[0];
    uint32_t r1  = sp[1];
    uint32_t r2  = sp[2];
    uint32_t r3  = sp[3];
    uint32_t r12 = sp[4];
    uint32_t lr  = sp[5];
    volatile uint32_t pc  = sp[6];
    uint32_t psr = sp[7];

    SSTPRINTS("HardFault:\n");
    SSTPRINTF(" SP  %h\n", (uint32_t)sp);
    SSTPRINTF(" PC  %h\n", pc);
    SSTPRINTF(" PSR %h\n", psr);
    while(1);
}
}

__attribute__( (naked) ) void HardFault_Handler(void) {
 __asm__ volatile
    (
     //   "IMPORT SST_DEBUG_HARDFAULT \n"
        "MOVS   R0, #4  \n"
        "MOV    R1, LR  \n"
        "TST    R0, R1  \n"
        "BEQ    _MSP    \n"
        "MRS    R0, PSP \n"
       	"B SST_DEBUG_HARDFAULT\n"
        "_MSP:  \n"
        "MRS    R0, MSP \n"
    	"B SST_DEBUG_HARDFAULT\n"
    );
}
*/

#endif // #ifndef NDEBUG // all debug output not for realease
#ifdef DSERIAL
SST_SWODebugSerial _SST_SWODebugSerial;
#endif


// init this lib
void SSTInitArduinoGeckoSDK()
{
  // reset reason
  _sst_resetcause_mask=RMU_ResetCauseGet();
  RMU_ResetCauseClear();
  // TODO external crystals
	/*
    // enable the External HF Oscillator and wait till it stabilizes
    CMU_OscillatorEnable(cmuOsc_HFXO,true,true);
    // set the HFXO as the Clock Source
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
    // disable the HFRCO
     CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
	*/

   // needed to do any gpio
   CMU_ClockEnable(cmuClock_GPIO,true);
   // debug
   // if ( DBG_Connected() ) ;
   // setup SysTick Timer for 1 msec interrupts
   if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000)) { while (1) ; }
   // calibrate microsecond delay loop
   UDELAY_Calibrate();
}

// arduino like main function
int main( void )
{
  // init
  CHIP_Init();   // see em_chip.h
  SSTInitArduinoGeckoSDK();
  // init sketch
  setup();
  // loop sketch
  while ( 1 )  { loop(); }
}





