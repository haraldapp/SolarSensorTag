//- -----------------------------------------------------------------------------------------------------------------------
// SolarSensorTag
// 2022-09-01 haraldapp Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// file: main.cpp
// Implements a ASKSINPP based code for some sensors on the SolarSensorTag hardware
//
// REFERENCES
// [1] AskSinPP https://github.com/pa-pa/AskSinPP and https://asksinpp.de/
//- -----------------------------------------------------------------------------------------------------------------------

// map aduino
#include "sst_arduino_geckosdk.h"

// to compile an individual SST sensor:
//  create a new configuration file, eg. sst_config_HM-WDS10-TH-O.h
//  set/add a build configuration with global define, eg. SST_CONFIG_FILE="sst_config_HM-WDS10-TH-O.h"
#ifdef SST_CONFIG_FILE
#include SST_CONFIG_FILE
#else
#error "no global define SST_CONFIG_FILE"
#endif

// Version
#define SST_VER_MAJOR 1
#define SST_VER_MINOR 2

// TODO implement asksin option wake up on radio
// TODO implement asksin AES using EFM32 hardware
// TODO implement SST option battery monitor
// TODO implement periodic polling for configuration changes (otherwise we have to push the button somewhere mounted devices)
// TODO optimize power consumption
// TODO    eg. asksin Radio-CC1101.h init() sets always MaxPower
// TODO    eg. asksin Radio-CC1101.h reduce EM0 runtime, check delays, current consumption is 47ms 5mA + 24ms 32mA
// TODO    eg. check if scaling down clocks has some effect on power consumption
// TODO optimize some code size

// asksin radio pins (radio.h)
#define MISO SPI_MISO_APIN
#define MOSI SPI_MOSI_APIN
#define SCK  SPI_SCK_APIN

// asksin storage (eeprom)
#ifndef SST_OPT_EEPROM_I2CADDR
#define SST_OPT_EEPROM_I2CADDR 0xA0
#endif
#if SST_OPT_EEPROM==0 // internal flash
 #error "eeprom internal flash not implemented"
#elif SST_OPT_EEPROM==1 // M24M01
 #define STORAGEDRIVER SST_EEPROM_M24MXX<SST_OPT_EEPROM_I2CADDR,128,256>
#elif  SST_OPT_EEPROM==2 // M24256
 #define STORAGEDRIVER SST_EEPROM_M24MXX<SST_OPT_EEPROM_I2CADDR,32,64>
#elif  SST_OPT_EEPROM==9 // M24xx compatible
 #ifndef SST_OPT_EEPROM_SIZE
 #error "SST_OPT_EEPROM_SIZE not defined"
 #endif
 #ifndef SST_OPT_EEPROM_PAGE
 #error "SST_OPT_EEPROM_PAGE not defined"
 #endif
 #define STORAGEDRIVER SST_EEPROM_M24MXX<SST_OPT_EEPROM_I2CADDR,SST_OPT_EEPROM_SIZE,64>
#endif

// asksin library
#define SPI_MODE0     0   // radio.h use template class LibSPI
#define EnableInterrupt_h   // asksinpp.h we have interrupts, do not use pin polling
#include "asksinpp.h"
#include "Register.h"
#include <MultiChannelDevice.h>
#include <Weather.h>
using namespace as;

// device info
static struct DeviceInfo devinfo = {
    {0x4e,0x52,0x12},            // Device ID
    "SST1234567",                // Device Serial
    {0x00,DEVICE_MODEL},         // Device Model
    0x10,                        // Firmware Version
    as::DeviceType::DEVICE_TYPE, // Device Type
    {0x01,0x00}                  // Info Bytes
};
// device ID and Serial from EFM32 chip ID (like USE_HW_SERIAL)
// remark:
//  to avoid modifying asksin code (see define USE_HW_SERIAL/Device.h)
//  we use a memory structure and initialize ID and Serial
//  early within a class constructor
SST_InitDeviceInfoFromHWSerial initDevInfo(devinfo.DeviceID,devinfo.Serial);

// radio
#ifndef RADIO_SENDDELAY
 #define RADIO_SENDDELAY 100
#endif
// custom config: reduce radio power
#define CONFIG_RADIOPOWER 3 // custom config offsets see asksinpp.h: CONFIG_FREQ1=0, CONFIG_FREQ2=1, CONFIG_BOOTSTATE=2
static uint8_t _sst_custom_radio_freq1=0;
static uint8_t _sst_custom_radio_freq2=0;
enum { sst_radiopower_notset=0, sst_radiopower_low=1, sst_radiopower_normal=2, sst_radiopower_max=3 };
static uint8_t _sst_custom_radio_power=sst_radiopower_normal;
typedef LibSPI<SPI_SS_APIN> SPIType;
#if !defined(SST_OPT_POWERON_TRX) || (SST_OPT_POWERON_TRX==0)
 typedef CC1101Radio<SPIType,INT1_APIN,NO_APIN,RADIO_SENDDELAY> RadioBaseType;
#else
 typedef CC1101Radio<SPIType,INT1_APIN,SPI_PWR_APIN,RADIO_SENDDELAY> RadioBaseType;
#endif
class RadioType : public RadioBaseType {
public:
  bool init () {
    // CC1101Radio:init() initializes
    //   CC1101_PATABLE=PA_MaxPower
    //   CC1101_FREQ2=0x21, CC1101_FREQ1=0x65, CC1101_FREQ0=0x6A (868.3 MHz)
    bool ret=RadioBaseType::init();
    if ( ret )  { // set custom config values on every radio init
      if( _sst_custom_radio_freq1>=0x60 && _sst_custom_radio_freq1<=0x6A )
        this->tuneFreq(0x21, _sst_custom_radio_freq1, _sst_custom_radio_freq2);
      if ( _sst_custom_radio_power==sst_radiopower_normal ) {
    	SSTPRINTS( _F("Set radio power normal\n") );
        initReg(CC1101_PATABLE, PA_Normal);
      }
      else if ( _sst_custom_radio_power==sst_radiopower_low ) {
      	SSTPRINTS( _F("Set radio power low\n") );
        initReg(CC1101_PATABLE, PA_LowPower);
      }
    }
    return ret;
  }
};

// led
static const LedStates::BlinkPattern _sst_led_blinkpattern_dual1_ex[] PROGMEM = SST_INIT_LED_BLINKPATTERN_DUAL1_EX;
static const LedStates::BlinkPattern _sst_led_blinkpattern_dual2_ex[] PROGMEM = SST_INIT_LED_BLINKPATTERN_DUAL2_EX;
class LedType : public SSTLedType<Led<ArduinoPins>,LedStates,LedStates::Mode,LedStates::BlinkPattern> {
public:
  LedType() : SSTLedType(_sst_led_blinkpattern_dual1_ex,_sst_led_blinkpattern_dual2_ex) {};
};

// battery
#if SST_OPT_BATTERY>0
//  defaults for battery measue interval
#if (DEVICE_TYPE==THSensor) && !defined(SST_OPT_BATTERY_INTERVAL)
#define SST_OPT_BATTERY_INTERVAL 0 // battery measured on every sensor measure
#endif
//  defaults for VL2020 (Vanadium-Lithium)
#if (SST_OPT_BATTERY==1) && !defined(SST_OPT_BATTERY_LOW)
#define SST_OPT_BATTERY_LOW 27
#endif
#if (SST_OPT_BATTERY==1) && !defined(SST_OPT_BATTERY_CRITICAL)
#define SST_OPT_BATTERY_CRITICAL 26
#endif
#if (SST_OPT_BATTERY==1) && !defined(SST_OPT_BATTERY_OVERCHARGE)
#define SST_OPT_BATTERY_OVERCHARGE 33
#endif
//  check battery paramters
#if !defined(SST_OPT_BATTERY_LOW) || !defined(SST_OPT_BATTERY_CRITICAL) || !defined(SST_OPT_BATTERY_INTERVAL) || !defined(SST_OPT_BATTERY_OVERCHARGE)
#error "missing battery parameters SST_OPT_BATTERY_xxx"
#endif

class BatterySensorType : public Alarm {
  uint32_t  m_Period;
  uint8_t   m_Low, m_Critical, m_Overcharge;
  uint16_t  m_Vcc;
  uint8_t   m_OverchargeProtect;
  uint8_t   m_OverchargeMeasure;
 public:
  BatterySensorType () : Alarm(0), m_Period(0), m_Low(0), m_Critical(0), m_Overcharge(0), m_Vcc(0),
                         m_OverchargeProtect(0), m_OverchargeMeasure(0) {
  }
  virtual ~BatterySensorType() {
  }
  // SST software overcharge protection
  #if SST_OPT_BATTERY_PROTECTION>0
  bool overchargeProtect( AlarmClock& clock, LedType& led ) {
    if ( m_OverchargeMeasure>0 ) {
      m_OverchargeMeasure=0;
      SSTLedOverChargeMode ledorg=led.overcharge();
      led.overcharge(sst_ledovercharge_off); // check
      Delay(10); measure();
      led.overcharge(ledorg);
    }
    if ( overcharge() ) {
      led.overcharge(sst_ledovercharge_on); // (re)initialize protection
      if ( m_OverchargeProtect==0 ) {
        SSTPRINTS( _F("Start battery overcharge protection\n") );
    	SST_SYSTICK_AddCallbackParam( overchargeProtectTickSecCB, 1000, this );
      }
      m_OverchargeProtect=1;
    }
    else if ( m_OverchargeProtect>0 && overchargedone() ) {
      led.overcharge(sst_ledovercharge_none);
      if ( m_OverchargeProtect>4*60 ) { // check at least for 4 minutes
        SST_SYSTICK_RemoveCallback( (void*)overchargeProtectTickSecCB );
        m_OverchargeProtect=0; m_OverchargeMeasure=0;
        SSTPRINTS( _F("End battery overcharge protection\n") );
      }
    }
    return (m_OverchargeProtect!=0);
  }
  static void overchargeProtectTickSecCB( void *param )  {
	  // SSTPRINTS( "o" );
      SST_Watchdog_Feed();
	  BatterySensorType *pThis=((BatterySensorType*)param);
      uint8_t& OverchargeProtect=pThis->m_OverchargeProtect;
      if ( OverchargeProtect>0 && OverchargeProtect<255 )   OverchargeProtect++; // next protection step
      uint8_t& OverchargeMeasure=pThis->m_OverchargeMeasure;
      if ( OverchargeMeasure<255 )  OverchargeMeasure++;
  }
  #else
  bool overchargeProtect( LedType& led ) { return false; }
  #endif
  virtual void trigger( AlarmClock& clock ) {
    measure();
    if ( m_Period>0 ) { set(m_Period); clock.add(*this); }
  }
  // BattSensor basics
  uint8_t current() const { return (m_Vcc+50)/100; }
  bool critical() const { return current()<m_Critical; }
  void critical(uint8_t value) { m_Critical=value; }
  bool low() const { return current()<m_Low; }
  void low (uint8_t value) { m_Low=value; }
  bool overcharge() const { return (m_Vcc+50)/100>m_Overcharge; }
  bool overchargecritical() const { return (m_Vcc+50)/100>m_Overcharge+1; }
  bool overchargedone() const { return (m_Vcc/100)<=m_Overcharge; }
  void overcharge(uint8_t value) { m_Overcharge=value; }
  void resetCurrent() { m_Vcc=0; }
  void init(uint32_t period,AlarmClock& clock) {
    SST_ADC_Init();
    measure();
    m_Period=period; set(m_Period);
    if ( period>0 )  clock.add(*this);
  }
  void measure() {
    #if SST_OPT_BATTERYMONITOR==1
    // TODO implement SST option battery monitor
    #error "not implemented: SST_OPT_BATTERYMONITOR"
    #else
    SST_ADC_MeasureInternalVCC( m_Vcc );
    SSTPRINTF( _F("iVcc: %u\n"), m_Vcc );
    #endif
  }
  void setIdle () {}
  void unsetIdle () {}
  // for backward compatibility
  uint16_t voltageHighRes() { return m_Vcc; }
  uint8_t voltage() { return current(); }
  // asksin METER for compatibility
  BatterySensorType& meter() { return *this; }
  BatterySensorType& sensor() { return *this; }
  void start () {}
  uint16_t value () const { return m_Vcc; }
  uint16_t finish () { measure(); return m_Vcc; }
};
#endif // #if SST_OPT_BATTERY>0


// asksin hal extended
//  battery overcharge
//  customized radio frequency and radio power
typedef AskSin<LedType,BatterySensorType,RadioType> HALBASETYPE;
class Hal : public HALBASETYPE {
public:
  void initBattery(uint16_t interval,uint8_t low,uint8_t critical,uint8_t overcharge) {
    battery.init(seconds2ticks(interval),sysclock);
    battery.low(low);
    battery.critical(critical);
    battery.overcharge(overcharge);
  }
  void config(const StorageConfig& sc) { // called by device::init
    if ( sc.valid()==true )  { // read custom radio setting - set by radio.init
      _sst_custom_radio_freq1=sc.getByte(CONFIG_FREQ1);
      _sst_custom_radio_freq2=sc.getByte(CONFIG_FREQ2);
      _sst_custom_radio_power=sc.getByte(CONFIG_RADIOPOWER);
      if ( _sst_custom_radio_power==sst_radiopower_notset )   _sst_custom_radio_power=sst_radiopower_normal;
    }
  }
  void config_setradiofreq(StorageConfig sc, uint8_t f1, uint8_t f2 ) {
    _sst_custom_radio_freq1=f1; sc.setByte(CONFIG_FREQ1,f1);
    _sst_custom_radio_freq2=f2; sc.setByte(CONFIG_FREQ2,f2);
    sc.validate();
  }
  void config_setradiopower(StorageConfig sc, uint8_t p ) {
    _sst_custom_radio_power=p; sc.setByte(CONFIG_RADIOPOWER,p);
    sc.validate();
  }
  uint8_t config_getradiopower() {
    return _sst_custom_radio_power;
  }
} hal;

// sensors

// temperature+humidity sensor
#if DEVICE_TYPE == THSensor
class WeatherSensor : public Alarm {
  // sensor	SHT4X is assembled
  #if SST_OPT_BATTERY_DEBUGTEST==1
   class SSTInternalVCCSensor m_sensor;
  #elif SST_OPT_SENSOR_SHT4X>0
   #if (SST_OPT_SENSOR_SHT4X==1) && !defined(SST_OPT_SENSOR_SHT4X_ADDR)
   #define SST_OPT_SENSOR_SHT4X_ADDR 0x44
   #endif
   #if (SST_OPT_SENSOR_SHT4X==2) && !defined(SST_OPT_SENSOR_SHT4X_ADDR)
   #define SST_OPT_SENSOR_SHT4X_ADDR 0x45
   #endif
   #if !defined(SST_OPT_SENSOR_SHT4X_ADDR)
   #error "SST_OPT_SENSOR_SHT4X_ADDR not defined"
   #endif
   class SensorSHT4x<SST_OPT_SENSOR_SHT4X_ADDR> m_sensor;
  #else
   #error "no weather sensor defined"
  #endif
public:
  WeatherSensor () { }
  virtual ~WeatherSensor () { }
  void init () {
    m_sensor.init(); // initialize the sensor
  }
  uint16_t before () const {
    return 3000; // return how many milliseconds the measure should start in front of sending the message
  }
  virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
    DPRINT("Measure...  ");
    hal.battery.meter().resetCurrent();
    hal.battery.meter().measure();
    bool ok=m_sensor.measure();
    if ( ok )  { DPRINT("T: ");DDEC(m_sensor.temperature()/10);DPRINT(", H: ");DDECLN(m_sensor.humidity()); }
    else       { DPRINTLN(" error"); }
  }
  //  temperature+humidity (used by WeatherChannel)
  uint16_t temperature ()  {
    if ( SSTGetRunMode()==SST_RUNMODE_DEBUG_VBAT )  return hal.battery.meter().value()/10;
    return m_sensor.temperature();
  }
  uint8_t  humidity ()  {
    if ( SSTGetRunMode()==SST_RUNMODE_DEBUG_VBAT )  return 99;
    return m_sensor.humidity();
  }
};
// weather channel
// TODO implement periodic polling for configuration changes (otherwise we have to push the button somewhere mounted devices)
// asksin WeatherChannel measure rate is about 150 seconds +/- slot spread + EXTRAMILLIS, see AskSinBase::nextSendSlot
#ifndef PEERS_PER_CHANNEL
#define PEERS_PER_CHANNEL 6   // number of available peers per channel
#endif
#ifndef EXTRAMILLIS
#define EXTRAMILLIS       456 // millisecond extra time to better hit the slot
#endif
// WeatherSensor List0 registers
//  the one and only device parameter is "wake up on radio"
DEFREGISTER(WeatherRegsList0,MASTERID_REGS,DREG_BURSTRX)
typedef RegList0<WeatherRegsList0> WeatherList0;
typedef WeatherChannel<Hal,SysClock,WeatherSensor,PEERS_PER_CHANNEL,EXTRAMILLIS,WeatherList0> SSTWeatherChannelType;
class SSTWeatherChannel : public SSTWeatherChannelType {
public:
  SSTWeatherChannel() : SSTWeatherChannelType() {
  }
  virtual ~SSTWeatherChannel() {
  }
  RTCAlarm& alarm() {
    return *this; // RTCAlarm is a protected base class, make it public
  }
};
typedef MultiChannelDevice<Hal,SSTWeatherChannel,1,WeatherList0> SSTDeviceTypeBase;
#endif // #if DEVICE_TYPE == THSensor

// asksin device
#if SST_OPT_RADIO_FREQADJUST==1
typedef SSTRadioFreqAdjust<SSTDeviceTypeBase,DeviceInfo,Message,AlarmClock,Alarm> SSTDeviceType;
#else
typedef SSTDeviceTypeBase SSTDeviceType;
#endif

class SSTDevice : public SSTDeviceType {
public:
  SSTDevice(const DeviceInfo& i,uint16_t addr) : SSTDeviceType(i,addr) {
  }
  virtual ~SSTDevice() {
  }
  void reset () {
    SSTDeviceTypeBase::reset();
    Delay(200);
    SST_SystemReset();
    // CONFIG_FREQ1
  }
  void freqadjust( uint8_t set ) {
    #if SST_OPT_RADIO_FREQADJUST==1
    if ( set==0 )   stopRadioFreqAdjust();
    if ( set==1 )   startRadioFreqAdjust();
    if ( set==2 )   toogleRadioFreqAdjust();
    #endif
  }
  bool freqadjust() {
    #if SST_OPT_RADIO_FREQADJUST==1
    return radioFreqAdjust();
    #else
    return false;
    #endif
  }
};
SSTDevice sdev(devinfo,0x20);

// ConfigButton
#if SST_OPT_RADIO_FREQADJUST==1
const uint8_t cfgBtnMenu[]=SST_BUTTONMENU_THSENSOR_DEVELOP;
#else
const uint8_t cfgBtnMenu[]=SST_BUTTONMENU_THSENSOR;
#endif
#define CFGBTNMENUSZE (sizeof(cfgBtnMenu)/sizeof(uint8_t))
typedef SSTMenuButton<SSTDevice,StateButton<>,LedStates,SST_VER_MAJOR,SST_VER_MINOR> SSTButtonType;
class SSTButton : public SSTButtonType {
public:
  SSTButton() : SSTButtonType(sdev,cfgBtnMenu,CFGBTNMENUSZE,seconds2ticks(2)) {
  };
  virtual void measure() {
    #if DEVICE_TYPE == THSensor
    SSTWeatherChannel& channel=sdev.channel(1);
    WeatherSensor& sensor=channel.sensors();
    uint32_t delay=1000; uint32_t before=sensor.before();
    SSTPRINTS( "Menu measure\n" );
    sysclock.cancel(channel.alarm());
    sysclock.instance().cancel(sensor);
    sysclock.instance().add(channel.alarm(),delay+before);
    sysclock.instance().add(sensor,delay);
    #endif
  }
} cfgBtn;

void setup( void )
{
  // check battery voltage first to avoid getting in a brownout reset loop
  #if SST_OPT_BATTERY>0
  if ( SST_GetSytemResetCause()==SST_RESETCAUSE_BROWNOUT )
    SST_SleepForever();
  if ( !SST_ADC_CheckInternalVCC(SST_OPT_BATTERY_CRITICAL,LED_R_APIN) )
    SST_SleepForever();
  #endif
  // init debug
  #if (SST_OPT_SWO_DEBUG==1) && !defined(NDEBUG)
  // SSTTestSWODebugBitrate( SST_OPT_SWO_DEBUG_BITRATE );
  DINITSWO( SST_OPT_SWO_DEBUG_BITRATE, SST_SWODEBUG_MODE_TIMER ); // 30000
  DPRINTLN( _F("setup()") );
  #endif
  // init watchdog
  #if ( SST_OPT_DISABLE_WATCHDOG!=1 )
  SST_Watchdog_Init( 30 );
  #endif
  DHWINFO();
  DPRINTLN(SST_ARDUINO_GECKOSDK_IDENTIFIER);
  DPRINTLN(ASKSIN_PLUS_PLUS_IDENTIFIER);
  DDEVINFO(sdev);
  // init I2C
  SST_I2C_PowerUp( SST_OPT_POWERON_I2C );
  SST_I2C_Init(); // Delay(1);
  // init asksin device
  DPRINTLN( _F("sdev.init") );
  sdev.init(hal);
  DPRINTLN( _F("buttonISR") );
  buttonISR(cfgBtn,INT0_APIN);
  sdev.initDone();
  DPRINTLN( _F("sdev.initDone") );
  // init SysClock and RTC
  SST_SYSTICK_AddCallback( callback, 1000/TICKS_PER_SECOND ); // SysClock callback defined in AlarmClock
  SST_RTC_Init();
  // init battery
  DPRINTLN( _F(" hal.initBattery") );
  hal.initBattery(SST_OPT_BATTERY_INTERVAL,SST_OPT_BATTERY_LOW,SST_OPT_BATTERY_CRITICAL,SST_OPT_BATTERY_OVERCHARGE);
  hal.activity.stayAwake(seconds2ticks(5));
  DPRINTLN( _F("setup() Done") );
}

void loop( void )
{
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  bool freqadjust = sdev.freqadjust();
  bool btnmenu= cfgBtn.isActive();
  if ( worked==false && poll==false && freqadjust==false && btnmenu==false ) {
    if ( hal.battery.critical() || cfgBtn.switchOff() ) {
      // sleep in EM4 until button pressed
      SSTSleep<Hal,SysClock>::SleepForever(hal);
    }
    #if SST_OPT_BATTERY_PROTECTION==1
    else if ( hal.battery.overchargeProtect(sysclock,hal.led) ) {
      // keep on running consuming power with leds on
      // switch on radio to consume power, if overcharge gets critical
      bool critical=hal.battery.overchargecritical();
      bool stayawake=hal.activity.stayAwake();
      if ( critical && !stayawake )  { stayawake=true; hal.activity.stayAwake(seconds2ticks(10)); hal.wakeup(); }
      if ( !critical && !stayawake ) hal.setIdle();
    }
    #endif
    else  {
      // sleep in EM2 with low power RTC wakeup
      typedef SSTSleep<Hal,SysClock,TICKS_PER_SECOND> Sleep;
      hal.activity.savePower<Sleep,Hal>(hal); // note: asksin savePower: sleeps only when ready to sleep, otherwise returns to continue loop
    }
  }
}



