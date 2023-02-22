//- -----------------------------------------------------------------------------------------------------------------------
// SST Solar Sensor Tag
// 2022-09-01 haraldapp Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------
// file: sst_arduino_geckosdk.h
// API with ARDUINO like functions used by ASKSINPP based EFM32 GeckoSDK functions

#ifndef _SST_ARDUINO_GECKOSDK_H_
#define _SST_ARDUINO_GECKOSDK_H_

// #include <cstddef>
#include <stdarg.h>
#include <string.h> // memcpy() memset() memcmp()
#include <stdlib.h> // rand() asksin message.h

#ifndef ARDUINO
#define ARDUINO 1
#endif
// use for sst arduino sdk extentions
#define ARDUINO_SST 1

// Version
#define SST_ARDUINO_GECKOSDK_MAJOR 1
#define SST_ARDUINO_GECKOSDK_MINOR 2

#define F(x) ((const char*)(x))
#define _F(x) ((const char*)(x))
#define SST_STRINGIZE_2(s) #s
#define SST_STRINGIZE(s) SST_STRINGIZE_2(s)

#define SST_ARDUINO_GECKOSDK_VERSION_STR SST_STRINGIZE(SST_ARDUINO_GECKOSDK_MAJOR) "." SST_STRINGIZE(SST_ARDUINO_GECKOSDK_MINOR)
#define SST_ARDUINO_GECKOSDK_IDENTIFIER "SSTArduinoGeckoSDK v" SST_ARDUINO_GECKOSDK_VERSION_STR

// Preprocessor
#define SST_STRINGIZE_2(s) #s
#define SST_STRINGIZE(s) SST_STRINGIZE_2(s)

// Types
typedef unsigned char byte; // ASKSIN storage.h
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned long uint32_t;
typedef long int32_t;
typedef unsigned long long uint64_t;

// C min/max
#ifdef __cplusplus
  template<class T, class L> auto min(const T& a,const L& b) -> decltype((b < a) ? b : a)  { return (b < a) ? b : a; }
  template<class T, class L> auto max(const T& a, const L& b) -> decltype((b < a) ? b : a)  { return (a < b) ? b : a; }
#else
#ifndef min
#define min(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#endif
#ifndef max
#define max(a,b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#endif
#endif

// Mem/ProgMem
#define PROGMEM
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define memcpy_P(dest, src, num) memcpy((dest), (src), (num))

// Arduino bit operations
#define bitRead(val,bit) (((val)>>(bit))&1)
#define bitSet(val,bit) ((val)|=(1UL<<(bit)))
#define bitClear(val,bit) ((val)&=~(1UL<<(bit)))
#define bitWrite(val,bit,bitval) ((bitval)?bitSet(val,bit):bitClear(val,bit))

// FunctPtr
typedef void (*voidFuncPtr)(void);
typedef void (*voidFuncPtrParam)(void*);

// GPIO
//  For referencing a pin for the EFM32 we need a port number and a pin number -
//  the ARDUINO library functions do have only one uint8_t to reference the pin.
//  To get around a global pin mapping array (see implementation from huaweiwx at github),
//  we here simply stuff EFM32 port and pin into one uint8_t.
//   APIN == "ARDUINO pin"
typedef enum { LOW=0, HIGH=1, CHANGE=2, FALLING=3, RISING=4, } PinStatus;
typedef enum { INPUT=0, OUTPUT=1, INPUT_PULLUP=2, INPUT_PULLDOWN=3, } PinMode;
typedef PinMode WiringPinMode; // button.h
typedef enum { LSBFIRST=0, MSBFIRST=1, } BitOrder;
typedef uint8_t pin_size_t;

#define MAKE_APIN(gpioportnr,gpiopinnr) ((((uint8_t)(gpioportnr))<<4)|((gpiopinnr)&0xF))
#define PORT_APIN(apin) ((GPIO_Port_TypeDef)((apin)>>4))
#define PIN_APIN(apin) ((apin)&0xF)
#define NO_APIN 0xFF

#include <sst_pins.h>  // sst custom pin names

void pinMode(uint8_t apin, uint8_t amode);
int digitalRead(uint8_t apin);
void digitalWrite(uint8_t apin, uint8_t val);
int analogRead(uint8_t pin);
void analogReference(uint8_t mode);
int analogGetReference(void);
void analogWrite(uint8_t apin, int val);
void SST_GPIO_PinModeSet( uint8_t port, uint8_t pin, uint8_t mode, uint8_t out );

// Interrupts
bool SST_GPIO_WakeUpFromPin( uint8_t apin );
inline void noInterrupts(void) {}; // ASKSIN storage.h EE_Write, not needed
inline void interrupts(void) {}; // ASKSIN storage.h
void enableInterrupt(pin_size_t interruptNumber, voidFuncPtr callback, PinStatus mode);
void disableInterrupt(pin_size_t interruptNumber);
void attachInterrupt(pin_size_t interruptNumber, voidFuncPtr callback, PinStatus mode);
void detachInterrupt(pin_size_t interruptNumber);
#define NOT_AN_INTERRUPT -1
int8_t digitalPinToInterrupt(uint8_t uAPin);

// Delay, ticks in milliseconds (trough SysTick_Handler)
void Delay(uint32_t dlyTicksMS);
uint32_t GetTickCount(void);
bool SST_SYSTICK_AddCallback( voidFuncPtr callback, uint32_t ticks_ms );
bool SST_SYSTICK_AddCallbackParam( voidFuncPtrParam callback, uint32_t ticks_ms, void *param ); // rem.: param not NULL
bool SST_SYSTICK_RemoveCallback( void *callback );
inline void _delay_ms(uint32_t msec) { Delay(msec); } // ASKSIN storage.h
inline void delay(unsigned long msec) { Delay(msec); }
inline uint32_t millis(void) { return GetTickCount(); } // millis since start/reset
#define SST_TICKS_PER_SECOND 100 // standard asksin alarm clock ticks per second

// Delay, ticks in microseconds (trough instruction loop, see gecko sdk udelay)
// used in asksinpp Radio-CC1101 only?
#include "hardware/kit/common/drivers/udelay.h"
void DelayMicroseconds(uint32_t usec);
inline void delayMicroseconds(unsigned int usec) { DelayMicroseconds(usec); }
inline void _delay_us(uint32_t usec) { DelayMicroseconds(usec); }

// PowerSave
#define SST_SAVEPOWER_WAKEUP_RTC       0x0001
#define SST_SAVEPOWER_WAKEUP_TIMER0    0x0010
#define SST_SAVEPOWER_WAKEUP_TIMER1    0x0020
#define SST_SAVEPOWER_WAKEUP_GPIO_EVEN 0x0100
#define SST_SAVEPOWER_WAKEUP_GPIO_ODD  0x0200
#define SST_SAVEPOWER_WAKEUP_SYSTICK   0x1000
uint16_t SST_SavePowerRTC( uint32_t wakeup_ticks, uint32_t ticks_per_second=100, uint32_t *pruntime_ms=0, uint32_t *psleeped_ticks=0 );
uint16_t SST_SavePower( void );
void SST_SleepForever();
void sst_get_sleep_statics( uint32_t& run_em0_ms, uint32_t& sleep_em2_s );

// Reset
enum { SST_RESETCAUSE_NA=0,
	   SST_RESETCAUSE_POWERON=1, SST_RESETCAUSE_PIN=1, SST_RESETCAUSE_BROWNOUT=2, SST_RESETCAUSE_WATCHDOG=3,
       SST_RESETCAUSE_LOCKUP=4, SST_RESETCAUSE_SOFT=5, SST_RESETCAUSE_EM4=6 };
void SST_SystemReset();
uint8_t SST_GetSytemResetCause();

// Watchdog
void SST_Watchdog_Init( uint16_t sec );
uint8_t SST_Watchdog_Done( void );
void SST_Watchdog_Enable( bool enable );
void SST_Watchdog_Feed();

// LED
void SST_LED_Init();
void SST_LED_Done();

// ADC
#define SST_ADC_INPUTSEL_CH0       0U    // input adc channel 0
#define SST_ADC_INPUTSEL_CH1       1U    // input adc channel 0
#define SST_ADC_INPUTSEL_CH4       4U    // input adc channel 0
#define SST_ADC_INPUTSEL_CH5       5U    // input adc channel 0
#define SST_ADC_INPUTSEL_TEMP      8U    // input internal temperature
#define SST_ADC_INPUTSEL_VDDDIV3   9U    // input VDD/3
#define SST_ADC_INPUTSEL_VDD       10U   // input VDD
#define SST_ADC_INPUTSEL_VSS       11U   // input VSS
#define SST_ADC_INPUTSEL_VREFDIV2  12U   // input VRef/2
#define SST_ADC_INPUTSEL_NONE      0xFFU // none
#define SST_ADC_REF_1V25           0     // internal 1.25V reference
#define SST_ADC_REF_2V50           1     // internal 2.5V reference
#define SST_ADC_REF_VDD            2     // VDD as reference
#define SST_ADC_RES_12BIT          0     // 12 bit resolution
#define SST_ADC_RES_8BIT           1     // 8 bit resolution
#define SST_ADC_RES_6BIT           2     // 6 bit resolution
#define SST_ADC_RES_OVS            3     // oversampling
bool SST_ADC_Init( void );
bool SST_ADC_Done( void );
uint8_t SST_ADC_APinToInput( uint8_t apin );
bool SST_ADC_Measure( uint16_t& val, uint8_t adcinput, uint8_t adcref=SST_ADC_REF_1V25, uint8_t adcres=SST_ADC_RES_OVS );
bool SST_ADC_MeasureInternalVCC( uint16_t& vcc );
bool SST_ADC_CheckInternalVCC( uint16_t critical, uint8_t signal_apin=NO_APIN );
bool SST_ADC_MeasureInternalTemp( int32_t& temp );

// RTC
void SST_RTC_Init();
void SST_RTC_Start( uint32_t wakeup_ticks, uint32_t tick_cnt_sec=SST_TICKS_PER_SECOND );
void SST_RTC_Stop();
uint32_t SST_RTC_GetCurrentTicks( uint32_t wakeup_ticks, uint32_t tick_cnt_sec=SST_TICKS_PER_SECOND );


// EEPROM
// TODO fix required E2END in asksin code (here only workaround)
#define E2END 1023 // storage.h InternalEprom
inline void eeprom_read_block( void *buf, const void *addr, uint16_t len ) {};
inline void eeprom_write_block( void *buf, void *addr, uint16_t len ) {};

// I2C
//   SST I2C uses location 4: pins PC0 SDA, PC1 SCL
//   with option PowerOn at pin PA2
//   (wrapper does use I2CSPM_Transfer() for communication)
#define SST_I2C_INSTANCE I2C0
#define SST_I2C_LOCATION 4
void SST_I2C_PowerUp( uint8_t sst_opt_poweron_i2c );
uint8_t SST_I2C_PowerDown( void );
void SST_I2C_Init( void );
void SST_I2C_Done( void );
#define SST_I2C_TRANSFER_WRITE       0x01 // plain write sequence: S+ADDR(W)+CMD+P.
#define SST_I2C_TRANSFER_READ        0x02 // plain read sequence: S+ADDR(R)+CMD+P.
#define SST_I2C_TRANSFER_WRITE_READ  0x04 // combined write/read sequence: S+ADDR(W)+CMD+Sr+ADDR(R)+DATA+P
#define SST_I2C_TRANSFER_WRITE_WRITE 0x08 // combined write/write sequence: S+ADDR(W)+CMD+DATA+P.
uint8_t SST_I2C_Transfer( uint8_t i2c_id, uint8_t i2c_transfer_type, const uint8_t *cmd, uint8_t cmdlen, uint8_t *data, uint16_t datalen );
uint8_t SST_I2C_WaitBusy( uint8_t i2c_id, uint16_t max_time_ms, uint16_t poll_time_ms );
bool SST_I2C_RetIsNack( uint8_t ret );

// SPI
// SST SPI uses location 3: pins PD6 RX, PD7 RX, PC15 CLK, PC14 /CS
#define SST_SPI_UART              1
#define SST_SPI_LOCATION          3
#define SST_SPI_BITORDER_LSBFIRST 0
#define SST_SPI_BITORDER_MSBFIRST 1
// #define SST_SST_SPI_MULTIBYTE_TRANSFER
uint8_t SST_SPI_Init( uint32_t bitrate, uint8_t bitorder );
void SST_SPI_Done();
uint8_t SST_SPI_TransferSingleByte( uint8_t send, uint8_t *recv );
uint8_t SST_SPI_TransferMultiByte( const uint8_t *send, uint8_t *recv, int16_t count );
#ifdef SST_SPI_MULTIBYTE_TRANSFER
uint8_t SST_SPI_ReceiveMultiByte( uint8_t *recv, int16_t count );
uint8_t SST_SPI_TransmitMultiByte( const uint8_t *bufTx, int16_t count );
#endif

// SST run mode
// used for release build sensors to observe/debug
// - battery voltage
// - sleep/awake ratio (power consumption)
// can be set by SST button menu
#define SST_RUNMODE_NORMAL     0
#define SST_RUNMODE_DEBUG_VBAT 1 // for THSensors: transmit Battery Voltage instead of temperature
uint8_t SSTGetRunMode();
uint8_t SSTSetRunMode( uint8_t runmode );

// SWO debug
//   to use the SWO debug output - it's may needed to open bridged R3 on the SST V1.0 pcb
enum { SST_SWODEBUG_MODE_DISABLED=0, SST_SWODEBUG_MODE_DELAY=1, SST_SWODEBUG_MODE_TIMER=2 };
#if !(defined(NDEBUG) || defined(NDEBUG_SWO))
void SSTInitSWODebug( uint32_t bitrate, int8_t mode=SST_SWODEBUG_MODE_TIMER );
void SSTEnableSWODebug();
void SSTDisableSWODebug();
void SSTDoneSWODebug();
void SSTWriteSWODebugLineLevel( uint8_t bIncrement );
short SSTWriteSWODebugChar( char c );
short SSTWriteSWODebugString( const char* buf, short len=-1 );
void SSTWriteSWODebugStringF( const char* fmt, ... );
void SSTWriteSWODebugStringVA( const char* fmt, __gnuc_va_list args );
short SSTWriteSWODebugInt( int32_t i, uint8_t base=10 );
short SSTWriteSWODebugUInt( uint32_t u, uint8_t base=10, uint8_t digits=0 );
void SSTWriteSWODebugAPin( uint8_t apin );
void SSTWriteSWODebugHWInfo( void );
void SSTTestSWODebugBitrate( uint32_t test_bitrate, int8_t mode=SST_SWODEBUG_MODE_TIMER );
#endif
// SST internal debug defs
#if defined(NDEBUG) || defined(NDEBUG_SWO)
#define SSTPRINTC(c)
#define SSTPRINTS(s)
#define SSTPRINTF(...)
#define SSTPRINTU(u)
#define SSTPRINTI(i)
#define SSTPRINTH(i)
#define SSTPRINTPIN(apin)
#define SSTPRINTM(...)
#define SSTDEBUG(...)
#else
#define SSTPRINTC(c) SSTWriteSWODebugChar(c)
#define SSTPRINTS(s) SSTWriteSWODebugString(s)
#define SSTPRINTF(f,...) SSTWriteSWODebugStringF(f,__VA_ARGS__)
#define SSTPRINTU(u) SSTWriteSWODebugUInt(u)
#define SSTPRINTI(i) SSTWriteSWODebugInt(i)
#define SSTPRINTH(u) SSTWriteSWODebugUInt(u,16)
#define SSTPRINTPIN(apin) SSTWriteSWODebugAPin(apin)
#define SSTPRINTM(classname,methodname,retp,fmt,...) CSSTPRINTM DBG(classname,methodname,retp,fmt,__VA_ARGS__)
#define SSTDEBUG(...) (__VA_ARGS__)
class CSSTPRINTM { public: // class method tracing helper
  uint8_t *m_pRet;
  CSSTPRINTM( const char* c, const char* m, uint8_t *pRet, const char*pfmt, ... ) {
    m_pRet=pRet;
    SSTWriteSWODebugString(c);
    if ( m && *m ) { SSTWriteSWODebugString(_F("::")); SSTWriteSWODebugString(m); }
    SSTWriteSWODebugString(_F("("));
    __gnuc_va_list ap; __builtin_va_start(ap,pfmt); if ( pfmt && *pfmt )  SSTWriteSWODebugStringVA( pfmt, ap );
    SSTWriteSWODebugString(_F(")")); SSTWriteSWODebugLineLevel(1);
  }
  ~CSSTPRINTM() {
    SSTWriteSWODebugLineLevel(0);
    SSTWriteSWODebugString(_F(" - "));
    if ( m_pRet && *m_pRet==0 )  SSTWriteSWODebugString(_F("00 ok"));
    else if ( m_pRet )           SSTWriteSWODebugUInt( *m_pRet );
    else                         SSTWriteSWODebugString( _F("done") );
    SSTWriteSWODebugString( _F("\n") );
  }
};
#endif

// arduino like main
void setup( void );
void loop( void );

// init (call from main or setup)
void InitSSTArduinoGeckoSDK();

// sst extended error handling/recording
//   record errors in one uint32_t with 4 bytes, mask:
//     0xFF000000: 0
//     0x00FF0000: error source
//     0x0000FF00: error (in) function  (specific to error source)
//     0x000000FF: error code           (specific to error source)
#define SST_ERROR_SOURCE_I2C           0x80
#define SST_ERROR_SOURCE_SPI           0x81
#define SST_ERROR_SOURCE_EEPROM_M24MXX 0x88
#define SST_ERROR_SOURCE_SENSOR_STH4X  0x90
#define SST_ERROR_FCT_UNSPECIFIED      0x00
#define SST_ERROR_CODE_OK              0x00
#define SST_ERROR_CODE_COMM_INVALIDARG 0x7F // may be used if not in range of divice specific error code
#define SST_ERROR_CODE_COMM_TIMEOUT    0x7E // may be used if not in range of divice specific error code
#define SST_ERROR_CODE_COMM_CRC        0x7D // may be used if not in range of divice specific error code
#define SST_MAKE_ERROR(code,fct,src) ( (((src)&0xFF)<<24) | (((fct)&0xFF)<<16) | ((code)&0xFF) )
#define SST_GET_ERROR_CODE(err) ((uint8_t)((err)&0xFF))
#define SST_GET_ERROR_FCT(err)  ((uint8_t)(((err)>>16)&0xFF))
#define SST_GET_ERROR_SRC(err)  ((uint8_t)(((err)>>24)&0xFF))

template <uint8_t SST_ERROR_SOURCE>
class SSTErrorSourceBase
{
protected:
  //
  uint8_t m_ErrorFct;
  uint8_t m_ErrorCode;
  inline bool setLastError( uint8_t ret, uint8_t retOK=SST_ERROR_CODE_OK, uint8_t fct=SST_ERROR_FCT_UNSPECIFIED ) {
    m_ErrorFct=fct; m_ErrorCode=ret; return (ret==retOK);
  }
  inline bool setLastErrorCode( uint8_t code,uint8_t fct=SST_ERROR_FCT_UNSPECIFIED ) {
    m_ErrorFct=fct; m_ErrorCode=code; return false;
  }
  inline bool clearLastError(uint8_t fct=SST_ERROR_FCT_UNSPECIFIED) {
    m_ErrorFct=fct; m_ErrorCode=0; return true;
  }
public:
  inline uint32_t getLastError() { return SST_MAKE_ERROR(m_ErrorCode,m_ErrorFct,SST_ERROR_SOURCE); }
  inline uint8_t getLastErrorCode() { return m_ErrorCode; }
  inline uint8_t getLastErrorFct() { return m_ErrorFct; }
  inline uint8_t getLastErrorSrc() { return SST_ERROR_SOURCE; }
};

// ADC
bool SST_ADC_Init( void );
bool SST_ADC_MeasureInternalVCC( uint16_t& vcc );

// askin METER template
class SSTInternalVCC {
public:
  typedef uint16_t ValueType; // for ASKSIN sensor
  static const int DefaultDelay = 0;
  void init () {
	  SST_ADC_Init();
  }
  void done () {
	  SST_ADC_Done();
  }
  void start () {}
  uint16_t finish () {
    uint16_t vcc=0; SST_ADC_MeasureInternalVCC( vcc );
    SSTPRINTS(_F("iVcc: ")); SSTPRINTU(vcc); SSTPRINTC('\n');
    return vcc;
  }
};

// askin vcc sensor
class SSTInternalVCCSensor {
protected:
	uint16_t m_vcc;
    uint8_t m_datavalid;
public:
  SSTInternalVCCSensor()  { init(); }
  void init()  { m_vcc=0; m_datavalid=0; SST_ADC_Init(); };
  void done () { SST_ADC_Done(); }
  inline int16_t vcc() { return m_vcc; }
  inline uint8_t datavalid() { return m_datavalid; }
  bool measure() { SST_ADC_MeasureInternalVCC(m_vcc); return true; }
  // for SST_OPT_BATTERY_DEBUGTEST only
  int16_t temperature() { return m_vcc/10; } // send voltage as temperature and run/sleep factor as humidity ( *10000 )
  uint8_t humidity() { uint32_t run_ms,sleep_s; sst_get_sleep_statics(run_ms,sleep_s); return (run_ms*10U)/(sleep_s+run_ms/1000U); }
};


// AES
// TODO implement EFM32 hardware AES for asksin


// arduino style classes
class SPISettings {
  public:
//#define SPI_MODE0
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode): clock(clock), bitOrder(bitOrder), dataMode(dataMode) {};
    SPISettings(): clock(0), bitOrder(0), dataMode(0) {};
    uint32_t clock;
    uint8_t bitOrder;
    uint8_t dataMode;
};

#ifdef SST_OPT_BITRATE_TRX
#define SST_SPI_BITRATE SST_OPT_BITRATE_TRX
#else
#define SST_SPI_BITRATE 1000000
#endif

class SPIClass : public SSTErrorSourceBase<SST_ERROR_SOURCE_SPI> {
private:
public:
  enum { errorFctBegin=1, errorFctEnd=2, errorFctTransferSingle=10, errorFctTransfer=11, errorFctGetData=20, errorFctClearData=30 };

  SPIClass()  {}
  virtual ~SPIClass () {}
  // Initialize the SPI library
  void  begin() {
    SSTPRINTM("SPIClass","begin",&m_ErrorCode,"",0);
    uint8_t ret=SST_SPI_Init( SST_SPI_BITRATE, SST_SPI_BITORDER_MSBFIRST );
	setLastError( ret, 0, errorFctBegin );
  }
  void end() {
    SSTPRINTM("SPIClass","end",&m_ErrorCode,"",0);
    disableInterrupt( INT1_APIN );
    SST_SPI_Done();
	setLastError( 0, 0, errorFctEnd );
  }
  inline void beginTransaction(__attribute__((unused))  SPISettings settings) {
  }
  inline void endTransaction(void) {
  }
  uint8_t transfer(uint8_t data) {
    uint8_t recv=0; uint8_t ret=SST_SPI_TransferSingleByte( data, &recv );
	setLastError( ret, 0, errorFctTransferSingle );
    return recv;
  }
  inline void transfer(const void *bufTx, void *bufRx, uint16_t count) {
	uint8_t ret=SST_SPI_TransferMultiByte( (const uint8_t *)bufTx, (uint8_t *)bufRx, count );
	setLastError( ret, 0, errorFctTransfer );

    SST_GPIO_PinModeSet( LED_G_APORT, LED_G_PIN, 4, 1 );
    DelayMicroseconds(10);
    SST_GPIO_PinModeSet( LED_G_APORT, LED_G_PIN, 4, 0 );
  }
  #ifdef SST_SPI_MULTIBYTE_TRANSFER
  // Read multiple bytes (blocking)
  inline void receive(void *bufRx, size_t count) {
    SST_SPI_ReceiveMultiByte( (uint8_t *)bufRx, count );
  }
  // Write multiple bytes (blocking)
  inline void transmit(const void *bufTx, size_t count) {
    SST_SPI_TransmitMultiByte( (const uint8_t *)bufTx, count );
  }
  #endif
  /* unused
  // inline static uint16_t transfer16(__attribute__((unused)) uint16_t data) { return 0;  }
  // inline static void transfer(__attribute__((unused)) void *buf, __attribute__((unused)) size_t count) { }
  inline static void setBitOrder(__attribute__((unused)) uint8_t bitOrder) {
  }
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure SPI settings.
  inline static void setDataMode(__attribute__((unused)) uint8_t dataMode) {
  }
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure SPI settings.
  inline static void setClockDivider(__attribute__((unused))  uint8_t clockDiv) {
  }
  // These undocumented functions should not be used.  SPI.transfer()
  // polls the hardware flag which is automatically cleared as the
  // AVR responds to SPI's interrupt
  inline static void attachInterrupt() {}
  inline static void detachInterrupt() {}
*/
};
extern SPIClass SPI; // used in radio.h


// EEPROM template class for asksin Storage
// (extended by: last error handling)
//  eg.:   #define STORAGEDRIVER SST_EEPROM_M24MXX<0xA0,1024,256> // M24M01
template <uint8_t I2C_ID,uint16_t EEPROM_SIZE_KBYTE,uint16_t EEPROM_PAGESIZE_BYTE=256>
class SST_EEPROM_M24MXX : public SSTErrorSourceBase<SST_ERROR_SOURCE_EEPROM_M24MXX> {
protected:
public:
  enum { errorFctStore=10, errorFctSetData=20, errorFctGetData=30, errorFctClearData=40 };
  SST_EEPROM_M24MXX() { }
  inline bool present()
  {
	  uint8_t offsetData[2]={0,0}; // just send offset data and check for ack
	  uint8_t ret=SST_I2C_Transfer( I2C_ID, SST_I2C_TRANSFER_WRITE, offsetData, 2, 0, 0 );
	  return ret==0;
  }
  inline uint16_t size() { return EEPROM_SIZE_KBYTE; }
  void store() {
     SSTPRINTM("SST_EEPROM_M24MXX","store",&m_ErrorCode,"",0);
     setLastError( 0, 0, errorFctStore );
  }
  inline uint8_t getBusyStatus(void) {
     return 0;
  }
  uint8_t getByte( uint32_t offset ) {
    uint8_t buf[1]={0}; bool ok=getData( offset, buf, 1 );
    return (ok?buf[0]:0);
  }
  bool setByte( uint16_t offset, uint8_t val ) {
    return setData( offset, &val, 1 );
  }
  bool setData( uint32_t offset, uint8_t* buf, uint16_t len ) {
    SSTPRINTM("SST_EEPROM_M24MXX","setData",&m_ErrorCode,"%u,%p,%u",offset,buf,len);
    uint8_t ret=writeData( offset, buf, len );
    return setLastError( ret, 0, errorFctSetData );
  }
  bool getData( uint32_t offset,uint8_t* buf,uint16_t len ) {
    // SSTPRINTM("SST_EEPROM_M24MXX","getData",&m_ErrorCode,"%u,%p,%u",offset,buf,len);
    if ( !offsetLenCheck(offset,len)  )   return setLastErrorCode(SST_ERROR_CODE_COMM_INVALIDARG,errorFctGetData);
    // highest offset bit to i2c addr
    uint16_t i2c_addr=I2C_ID; if ( !offsetToAddr(offset,i2c_addr) )   return setLastErrorCode(SST_ERROR_CODE_COMM_INVALIDARG,errorFctGetData);
    uint8_t ret=0;
    while ( len>0 ) {
      uint16_t readlen=len; alignLenToPage( offset, readlen ); // do not read across page boundary
      // send offset (to start reading from) and get data
      uint8_t offsetData[2]; offsetData[0]=(uint8_t)offset; offsetData[1]=(uint8_t)(offset>>8);
      ret=SST_I2C_Transfer( i2c_addr, SST_I2C_TRANSFER_WRITE_READ, offsetData, 2, buf, readlen );
      if ( ret!=0 )   break;
      // wait busy
      ret=SST_I2C_WaitBusy( I2C_ID, 10, 2 );
      if ( ret!=0 )   break;
      // next page
      offset+=readlen; buf+=readlen; len-=readlen;
    }
    return setLastError( ret, 0, errorFctGetData );
  }
  bool clearData( uint32_t offset, uint16_t len ) {
    // SSTPRINTM("SST_EEPROM_M24MXX","clearData",&m_ErrorCode,"%u,%u",offset,len);
    uint8_t buf[32]={0}; if ( !offsetLenCheck(offset,len)  )   return setLastError(SST_ERROR_CODE_COMM_INVALIDARG,errorFctClearData);
    while ( len>0 )
    {
      uint16_t lenTemp=(len>sizeof(buf)?sizeof(buf):len); alignLenToPage( offset, lenTemp );
      uint8_t ret=writeData( offset, buf, lenTemp );
      if ( ret )     return setLastError(ret,errorFctClearData);
      offset+=lenTemp; len-=lenTemp;
    }
    return setLastError(0,errorFctClearData);
  }
protected:
  inline bool offsetLenCheck( uint32_t& offset, uint16_t& len ) {
    if ( offset>=EEPROM_SIZE_KBYTE*1024 )   return false;
    if ( offset+len>EEPROM_SIZE_KBYTE*1024 )   len=(EEPROM_SIZE_KBYTE*1024)-offset;
    return true;
  }
  inline bool offsetToAddr( uint32_t& offset, uint16_t& i2c_addr ) {
    // M24MXX: move eeprom read/write offset bit(s) to i2c address
    i2c_addr&=0xFD; if ( offset>=0x00010000 )  { i2c_addr|=0x02; offset&=0xFFFEFFFF; }
    return ((offset&0xFFFF0000)==0);
  }
  inline uint8_t alignLenToPage( uint32_t offset, uint16_t& len ) {
    uint32_t pagenr=(offset/EEPROM_PAGESIZE_BYTE);
	uint32_t pageend=((pagenr+1)*EEPROM_PAGESIZE_BYTE);
	if ( offset+len>pageend )   { len=pageend-offset; return 1; }
	return 0;
  }
  uint8_t writeData( uint32_t offset, uint8_t* buf, uint16_t len ) {
    if ( !offsetLenCheck(offset,len)  )   return SST_ERROR_CODE_COMM_INVALIDARG;
    // highest offset bit to i2c addr
    uint16_t i2c_addr=I2C_ID; if ( !offsetToAddr(offset,i2c_addr) )   return SST_ERROR_CODE_COMM_INVALIDARG;
    // check addresses are within one eeprom page
    uint8_t ret=0;
    while ( len>0 ) {
      uint16_t writelen=len; alignLenToPage( offset, writelen ); // do not write across page boundary
      // send offset (to start writing to) and data
      uint8_t offsetData[2]; offsetData[0]=(uint8_t)offset; offsetData[1]=(uint8_t)(offset>>8);
      ret=SST_I2C_Transfer( i2c_addr, SST_I2C_TRANSFER_WRITE_WRITE, offsetData, 2, buf, writelen );
      if ( ret!=0 )   break;
      // wait busy
      ret=SST_I2C_WaitBusy( I2C_ID, 50, 2 );
      if ( ret!=0 )   break;
      // next page
      offset+=writelen; buf+=writelen; len-=writelen;
    }
    return ret;
  }

};

template<uint8_t I2C_ID=0x44>
class SensorSHT4x : public SSTErrorSourceBase<SST_ERROR_SOURCE_SENSOR_STH4X> {
protected:
	int16_t m_temperature;
    uint8_t m_humidity;
    uint8_t m_datavalid;
	uint8_t m_temperature_factor;
    uint8_t m_precition;
    enum { SHT_CMD_READ_HIGH_PRECITION=0xFD, SHT_CMD_READ_MEDIUM_PRECITION=0xF6, SHT_CMD_READ_LOW_PRECITION=0xE0,
           SHT_CMD_READ_SERIALNR=0x89,
		   SHT_CMD_SOFT_RESET=0x94,
		   SHT_CMD_ACTIVATE_HEATER_200MW_1S=0x39, SHT_CMD_ACTIVATE_HEATER_200MW_100MS=0x32,
		   SHT_CMD_ACTIVATE_HEATER_110MW_1S=0x2F, SHT_CMD_ACTIVATE_HEATER_110MW_100MS=0x24,
		   SHT_CMD_ACTIVATE_HEATER_20MW_1S=0x1E, SHT_CMD_ACTIVATE_HEATER_20MW_100MS=0x15,
    };
    enum { SHT_POWERUP_TIME_MS=1, SHT_SOFTRESET_TIME_MS=1,
    	   SHT_MEASUREMENT_HP_TIME_MS=9, SHT_MEASUREMENT_MP_TIME_MS=5, SHT_MEASUREMENT_LP_TIME_MS=2,
		   SHT_HEATERON_SHORT_TIME_MS=120, SHT_HEATERON_LONG_TIME_MS=1200,
    };
public:
  enum { SHT_PRECITION_DEFAULT=0, SHT_PRECITION_HIGH=0, SHT_PRECITION_MEDIUM=1, SHT_PRECITION_LOW=2 };
  enum { error_fct_measure=10, error_fct_measure_sendcmd=11, error_fct_measure_readdata=12 };
  SensorSHT4x()  { init(); }
  void init()  { m_temperature=0; m_humidity=0; m_datavalid=0; m_temperature_factor=10; m_precition=SHT_PRECITION_DEFAULT; };
  inline void precition( uint8_t p )  { m_precition=p; }
  inline uint8_t precition()  { return m_precition; }
  inline void temperature_factor( uint8_t f )  { m_temperature_factor=f; } // =1: 22,1°c read as 22, =10 22,1°c read as 221
  inline uint8_t temperature_factor()  { return m_temperature_factor; }
  inline int16_t temperature() { return m_temperature; }
  inline uint8_t humidity() { return m_humidity; }
  inline uint8_t datavalid() { return m_datavalid; }
  bool measure()  {
    uint8_t cmd[1]={SHT_CMD_READ_HIGH_PRECITION}; uint8_t data[6]={0}; uint16_t cmdtime=SHT_MEASUREMENT_HP_TIME_MS;
    if ( m_precition==SHT_PRECITION_MEDIUM )   { cmd[0]=SHT_CMD_READ_MEDIUM_PRECITION; cmdtime=SHT_MEASUREMENT_MP_TIME_MS; }
    if ( m_precition==SHT_PRECITION_LOW )   { cmd[0]=SHT_CMD_READ_LOW_PRECITION; cmdtime=SHT_MEASUREMENT_LP_TIME_MS; }
    uint8_t i2c_addr=(I2C_ID)<<1;
    uint8_t ret=SST_I2C_Transfer( i2c_addr, SST_I2C_TRANSFER_WRITE, cmd, 1, 0, 0 );
    if ( ret!=0 )   return setLastErrorCode( ret, error_fct_measure_sendcmd );
    Delay( cmdtime ); int8_t extratime=cmdtime;
    while ( extratime>=0 )  {
      ret=SST_I2C_Transfer( i2c_addr, SST_I2C_TRANSFER_READ, data, 6, data, 6 );
      if ( ret==0 )  {
        // calc result
        uint16_t t_ticks=(((uint16_t)data[0])<<8)|data[1];
        uint8_t t_checksum=data[2];
        uint16_t rh_ticks=(((uint16_t)data[3])<<8)|data[4];
        uint8_t rh_checksum=data[5];
        if ( !checkCRC(data,2,t_checksum) )  return setLastErrorCode( SST_ERROR_CODE_COMM_CRC, error_fct_measure_readdata );
        if ( !checkCRC(data+3,2,rh_checksum) )  return setLastErrorCode( SST_ERROR_CODE_COMM_CRC, error_fct_measure_readdata );
        m_temperature=( (-45 * m_temperature_factor) + (175 * t_ticks * m_temperature_factor)/65535 );
        uint32_t humidity=( -6 + (125 * rh_ticks)/65535 );
        if ( humidity>100 )     m_humidity=100;
        else if ( humidity<0 )  m_humidity=0;
        else                    m_humidity=humidity;
        return clearLastError(error_fct_measure_readdata);
      }
      else if ( SST_I2C_RetIsNack(ret) )  {
        // not ready now? put on the sunglasses and relax
    	Delay(2); extratime-=2;
      }
      else
        return setLastErrorCode( ret, error_fct_measure_readdata );
    }
    return setLastErrorCode( SST_ERROR_CODE_COMM_TIMEOUT, error_fct_measure_readdata );
  }
protected:
  bool checkCRC( uint8_t *data, uint8_t len, uint8_t crcCheck ) {
    uint8_t crc=0xff;
    for ( uint8_t i=0; i<len; i++ ) {
      crc^=data[i];
      for ( uint8_t j=0; j<8; j++ ) {
        if ( (crc&0x80)!=0 )   crc=(uint8_t)((crc<<1)^0x31); // default polynomial see datasheet
        else                   crc<<=1;
      }
    }
    return (crc==crcCheck);
  }
};

// AskSin Sleep
template <class Hal, class SysClockType, uint32_t const ticks_per_second=SST_TICKS_PER_SECOND>
class SSTSleep {
public:
  static uint16_t powerSave(Hal& hal) {
    uint16_t ret=0; SysClockType& sysclock=SysClockType::instance();
    if (sysclock.isready() == false) {
      uint32_t wakeup_ticks=sysclock.next();
      if ( wakeup_ticks<10 ) {
        Delay( (wakeup_ticks*1000)/SST_TICKS_PER_SECOND );
      }
      else {
      SSTPRINTS( "Sleep EM2 prepare\n");
      sysclock.disable();
      hal.setIdle();
      SST_I2C_Done();
      uint8_t i2coptpwr=SST_I2C_PowerDown();
      SST_ADC_Done();
      if ( hal.led.sleep() )   SST_LED_Done(); // save power on gpio pins
      bool ledsleep=hal.led.sleep();
      if ( wakeup_ticks/ticks_per_second>10 )   ledsleep=true; // assume no user action for long sleeps
      hal.led.sleep( true );
      uint8_t reinit_watchdog=SST_Watchdog_Done();
      // sleep EM2
      SSTPRINTF( "Sleep EM2 start: %u ticks\n", wakeup_ticks );
      uint32_t ticks_sleeped=0; uint32_t ms_runtime=0;
      ret=SST_SavePowerRTC( wakeup_ticks, ticks_per_second, &ms_runtime, &ticks_sleeped ); // sleep in EM2
      SSTPRINTF( "Sleep EM2 end: wakeup %h, runtime_ms %u, sleeptime_ms %u\n", ret, ms_runtime, (ticks_sleeped*1000)/ticks_per_second );
      // re init
      bool bButtonWakeUp=SST_GPIO_WakeUpFromPin(INT0_APIN);
      if ( reinit_watchdog )  SST_Watchdog_Init( bButtonWakeUp ? 30 : 4 );
      sysclock.enable();
      sysclock.correct(ticks_sleeped == 0 ? 0 : ticks_sleeped-1);
      SST_LED_Init(); // re init gpio
      SSTPRINTF( "Led sleep state: buttonwakeup %u ledsleep %u\n", bButtonWakeUp, ledsleep );
      hal.led.sleep( (!bButtonWakeUp) && ledsleep ); // button wake up, enable LEDs to show what's going on
      SST_ADC_Init();
      SST_I2C_PowerUp( i2coptpwr );
      SST_I2C_Init();
      }
    } else {
      hal.unsetIdle();
      sysclock.enable();
    }
    return ret;
  }
  static void SleepForever(Hal& hal)
  {
	// sleep in EM4 until button pressed (causes reset)
    SysClockType& sysclock=SysClockType::instance();
    sysclock.disable();
    hal.setIdle();
    SST_I2C_Done();
    SST_I2C_PowerDown();
    SST_ADC_Done();
    SST_LED_Done(); // save power on gpio pins
    SST_Watchdog_Enable( false );
    SSTPRINTS( "Sleep EM4 start\n" );
    SST_SleepForever();
  }
  static void waitSerial() {
  }
};

// Unique device ID
void SST_GetDeviceIDFromChipID( uint8_t (&DeviceID)[3], char (&Serial)[11] );

// AskSin device ID init from chip ID
class SST_InitDeviceInfoFromHWSerial {
public:
  SST_InitDeviceInfoFromHWSerial( uint8_t (&DeviceID)[3], char (&Serial)[11] ) {
	SST_GetDeviceIDFromChipID( DeviceID, Serial );
  }
};

// SST led ex
// TODO think about: asksin DualStatusLed has no members to customize led states or blink patterns
// TODO think about: asksin template class DualStatusLed: led1/2 defined private and class does not have functions to set led's individual
enum SSTLedModeEx {
  sst_ledmode_none=-1,
  sst_ledmode_off=0,
  sst_ledmode_signal_ok, sst_ledmode_signal_error, sst_ledmode_signal_debug, sst_ledmode_signal_running,
  sst_ledmode_exec,
  sst_ledmode_menu_startmenu,
  sst_ledmode_menu_startpairing, sst_ledmode_menu_setradiopower, sst_ledmode_menu_startfreqadjust,
  sst_ledmode_menu_resetdevice, sst_ledmode_menu_resetsoft, sst_ledmode_menu_switchoff,
  sst_ledmode_menu_modenormal, sst_ledmode_menu_modedebugvbat,
  sst_ledmode_menu_showversion,
  sst_ledmode_menu_radiopower1, sst_ledmode_menu_radiopower2, sst_ledmode_menu_radiopower3,
};
enum SSTLedOverChargeMode {
  sst_ledovercharge_none=0,
  sst_ledovercharge_on=1,
  sst_ledovercharge_off=2,
};

template <class TLED, class TLEDSTATES, typename TMODE, typename TBLINKPATTERN>  // TLEDSTATES LedState, TLED Led<ArduinoPins>
class SSTLedType : public TLEDSTATES {
protected:
  TLED m_led1;
  TLED m_led2;
  uint8_t m_enable;
  uint8_t m_overcharge;
  uint8_t m_sleep;
  uint8_t m_menu;
  const TBLINKPATTERN *m_blinkpattern_dual1_ex;
  const TBLINKPATTERN *m_blinkpattern_dual2_ex;
public:
  SSTLedType (const TBLINKPATTERN *blinkpattern_dual1_ex, const TBLINKPATTERN *blinkpattern_dual2_ex) {
    m_enable=true; m_overcharge=0; m_sleep=0; m_menu=0;
    m_blinkpattern_dual1_ex=blinkpattern_dual1_ex; m_blinkpattern_dual2_ex=blinkpattern_dual2_ex;
  }
  void init() {
    m_led1.init(LED_R_APIN); m_led2.init(LED_G_APIN);
  }
  bool active() const {
    return (m_led1.active() || m_led2.active());
  }
  void ledOn() {
    if ( stdmode() )  { m_led1.ledOn(); m_led2.ledOn(); }
  }
  void ledOn (uint32_t ticks) {
    if ( stdmode() )  { m_led1.ledOn(ticks); m_led2.ledOn(ticks); }
  }
  void ledOn (uint32_t ticks,uint32_t tacks) {
    if ( stdmode() )  { m_led1.ledOn(ticks); m_led2.ledOn(tacks); }
  }
  void ledOff () {
    if ( stdmode() )  { m_led1.ledOff(); m_led2.ledOff(); }
  }
  void set(TMODE stat) {
    if ( stdmode() )  { m_led1.set(stat,TLEDSTATES::dual1); m_led2.set(stat,TLEDSTATES::dual2); }
  }
  void invert (bool value) {
    if ( stdmode() )  { m_led1.invert(value); m_led2.invert(value); }
  }
  void enable (bool e) {
	if ( e==m_enable )   return;
    m_enable=e; stdenable();
  }
  bool enable() const {
    return m_enable;
  }
  // this should be part of asksin DualStatusLed to use DualStatusLed as base
  inline TLED& Led1() { return m_led1; }
  inline TLED& Led2() { return m_led2; }
  // SST ex
  //  prioritized methods for sst led modes
  bool stdmode() {
    return !(m_overcharge || m_menu || m_sleep);
  }
  void stdenable() {
	bool e=m_enable; if ( !stdmode() )  e=true;
	m_led1.enable(e); m_led2.enable(e);
  }
  inline void stop() {
    setEx( sst_ledmode_off );
  }
  void setEx( SSTLedModeEx stat ) {
	stdenable(); if ( stat<0 )   return;
    m_led1.set((TMODE)stat,m_blinkpattern_dual1_ex); m_led2.set((TMODE)stat,m_blinkpattern_dual2_ex);
  }
  void ledOnEx() {
	stdenable(); m_led1.ledOn(); m_led2.ledOn();
  }
  void ledOffEx() {
    m_led1.ledOff(); m_led2.ledOff();
  }
  void sleep( bool setsleep ) {
    m_sleep=setsleep; stdenable();
    if ( setsleep )  stop();
  }
  inline bool sleep() const {
    return m_sleep;
  }
  void menu( bool on ) {
    if ( on==m_menu )   return;
    m_menu=on; stdenable();
    if ( !on && m_overcharge==sst_ledovercharge_on )  ledOnEx();
  }
  inline bool menu() const {
    return m_menu;
  }
  void overcharge( SSTLedOverChargeMode omode ) {
    if ( omode==m_overcharge )   return;
    m_overcharge=omode; stdenable();
    if ( m_menu )  return;
    if ( omode==sst_ledovercharge_on )  ledOnEx();
    else                                stop();
  }
  inline SSTLedOverChargeMode overcharge() const {
    return (SSTLedOverChargeMode)m_overcharge;
  }
  void dump() {
    SSTPRINTF( "Led: enable %u, overcharge %u, sleep %u, menu %u\n", m_enable, m_overcharge, m_sleep, m_menu );
  }
};
#define SST_INIT_LED_BLINKPATTERN_DUAL1_EX \
  { /*red*/ \
    {0, 0, {0,0} },               /*off*/  \
    {0, 0, {0,0} },               /*signal_ok*/ \
    {4, 1, {0,2,6,0} },           /*signal_error*/ \
    {4, 1, {0,2,6,0} },           /*signal_debug*/ \
    {2, 1, {1,0} },               /*signal_running*/ \
    {0, 0, {0,0} },               /*exec (longpressed)*/ \
    {0, 0, {0,0} },               /*menu_startmenu*/ \
    {2, 255, {255,0} },           /*menu_startpairing*/ \
    {2, 255, {4,1} },             /*menu_setradiopower*/ \
    {2, 255, {255,0} },           /*menu_startfreqadjust*/ \
    {2, 255, {255,0} },           /*menu_resetdevice*/ \
    {2, 255, {255,0} },           /*menu_resetsoft*/ \
    {2, 255, {4,1} },             /*menu_switchoff*/ \
    {0, 0, {0,0} },               /*menu_modenormal*/ \
    {2, 255, {1,4} },             /*menu_modedebugvbat*/ \
    {6, 255, {0,2,2,0,2,0} },     /*menu showversion*/ \
    {2, 255, {1,5} },             /*menu radiopower1*/ \
    {2, 255, {3,3} },             /*menu radiopower2*/ \
    {2, 255, {5,1} },             /*menu radiopower3*/ \
  };
#define SST_INIT_LED_BLINKPATTERN_DUAL2_EX \
  { /*green*/ \
    {0, 0, {0,0} },               /*off*/  \
    {4, 1, {0,2,6,0} },           /*signal_ok*/ \
    {4, 1, {0,2,6,0} },           /*signal_debug*/ \
    {0, 0, {0,0} },               /*signal_error*/ \
    {2, 1, {1,0} },               /*signal_running*/ \
    {0, 0, {0,0} },               /*exec (longpressed)*/ \
    {2, 0, {4,1} },               /*menu_startmenu*/ \
    {2, 255, {255,0} },           /*menu_startpairing*/ \
    {2, 255, {255,0} },           /*menu_setradiopower*/ \
    {2, 255, {4,1} },             /*menu_startfreqadjust*/ \
    {0, 0, {0,0} },               /*menu_resetdevice*/ \
	{4, 255, {0,4,1,0} } ,        /*menu_resetsoft*/ \
    {4, 255, {0,4,1,0} },             /*menu_switchoff*/ \
	{2, 255, {255,0} },           /*menu_modenormal*/ \
	{2, 255, {255,0} },           /*menu_modedebugvbat*/ \
    {6, 255, {2,0,2,0,0,2} },     /*menu showversion*/ \
    {2, 255, {1,5} },             /*menu radiopower1*/ \
    {2, 255, {3,3} },             /*menu radiopower2*/ \
    {2, 255, {5,1} },             /*menu radiopower3*/ \
  };


// SST menu button
// TODO add button menu timeout (at the moment done by watchdog)
enum { SST_BUTTONMENU_END=0,
	   SST_BUTTONMENU_WRAP=1,
	   SST_BUTTONMENU_STARTMENU=5,
	   SST_BUTTONMENU_STARTPAIRING=10,
	   SST_BUTTONMENU_SETRADIOPOWER=11,
	   SST_BUTTONMENU_STARTRADIOREQADJUST=12,
	   SST_BUTTONMENU_RESETDEVICE=20,
	   SST_BUTTONMENU_RESETSOFT=21,
	   SST_BUTTONMENU_SWITCHOFF=22,
	   SST_BUTTONMENU_MODENORMAL=30,
	   SST_BUTTONMENU_MODEDEBUGVBAT=31,
	   SST_BUTTONMENU_SHOWVERSION=40,
	   SST_BUTTONMENU_RADIOPOWER_LOW=80,
	   SST_BUTTONMENU_RADIOPOWER_NORMAL=81,
	   SST_BUTTONMENU_RADIOPOWER_HIGH=82,
	   SST_BUTTONMENU_GOTO=128, // rem.: flag
	   SST_BUTTONMENU_NONE=255,
};
//
#define SST_BUTTONMENU_THSENSOR \
	{SST_BUTTONMENU_STARTPAIRING,SST_BUTTONMENU_SETRADIOPOWER, \
     SST_BUTTONMENU_RESETDEVICE,SST_BUTTONMENU_RESETSOFT,SST_BUTTONMENU_SWITCHOFF, \
     SST_BUTTONMENU_MODENORMAL,SST_BUTTONMENU_MODEDEBUGVBAT, \
	 SST_BUTTONMENU_SHOWVERSION, \
	 SST_BUTTONMENU_END, \
	 SST_BUTTONMENU_RADIOPOWER_LOW, SST_BUTTONMENU_RADIOPOWER_NORMAL, SST_BUTTONMENU_RADIOPOWER_HIGH, \
     SST_BUTTONMENU_GOTO|SST_BUTTONMENU_RADIOPOWER_LOW, \
	}
#define SST_BUTTONMENU_THSENSOR_DEVELOP \
	{SST_BUTTONMENU_STARTPAIRING, SST_BUTTONMENU_SETRADIOPOWER, ST_BUTTONMENU_STARTRADIOREQADJUST, \
	 SST_BUTTONMENU_RESETDEVICE,SST_BUTTONMENU_RESETSOFT,SST_BUTTONMENU_SWITCHOFF, \
	 SST_BUTTONMENU_MODENORMAL,SST_BUTTONMENU_MODEDEBUGVBAT, \
	 SST_BUTTONMENU_SHOWVERSION, \
	 SST_BUTTONMENU_END, \
     SST_BUTTONMENU_RADIOPOWER_LOW, SST_BUTTONMENU_RADIOPOWER_NORMAL, SST_BUTTONMENU_RADIOPOWER_HIGH, \
     SST_BUTTONMENU_GOTO|SST_BUTTONMENU_RADIOPOWER_LOW, \
    }
template <class DEVTYPE,class TBUTTONBASE,class LEDSTATES,const uint8_t version1,const uint8_t version2>  // TBUTTONBASE eg. StateButton<OFFSTATE,ONSTATE,MODE>
class SSTMenuButton : public TBUTTONBASE {
  DEVTYPE& m_device;
  const uint8_t *m_pmenudef;
  uint8_t m_menucnt;
  int8_t m_currentmenuoffset;
  SSTLedModeEx m_signal;
  enum { SST_MB_ACTION_NONE=0, SST_MB_ACTION_MEASURE=1, SST_MB_ACTION_SWITCHOFF=-1 };
  int8_t m_action;
public:
  SSTMenuButton (DEVTYPE& dev,const uint8_t* pmenuinit, uint8_t menucnt, uint32_t longpressticks=0) : m_device(dev) {
    m_pmenudef=pmenuinit; m_menucnt=menucnt, m_currentmenuoffset=-1;
    m_signal=sst_ledmode_none; m_action=SST_MB_ACTION_NONE;
    if ( longpressticks>0 )   this->setLongPressTime(longpressticks);
  }
  virtual ~SSTMenuButton () {
  }
  virtual void state (uint8_t s) {
    TBUTTONBASE::state(s);
    SST_Watchdog_Feed();
    if ( s==TBUTTONBASE::released )
      nextMenu();
    else if ( s==TBUTTONBASE::longpressed )
      updateLed();
    else if ( s==TBUTTONBASE::longreleased )
      exec();
  }
  void nextMenu() {
    // next menu item
    m_currentmenuoffset++; m_signal=sst_ledmode_none; uint8_t m=currentMenuItem();
    if ( m==SST_BUTTONMENU_END )   m_currentmenuoffset=-1; // done
    if ( m==SST_BUTTONMENU_WRAP )   m_currentmenuoffset=0; // wrap
    if ( m&SST_BUTTONMENU_GOTO )   m_currentmenuoffset=findMenu(m&(~SST_BUTTONMENU_GOTO)); // goto
    SSTPRINTF( "Menu current %i\n", m_currentmenuoffset );
    updateLed();
  }
  int8_t findMenu( uint8_t m ) const {
    for ( int8_t ofs=0; ofs<m_menucnt; ofs++ )
      if ( m_pmenudef[ofs]==m )
        return ofs;
    return -1;
  }
  void updateLed() {
    uint8_t s=TBUTTONBASE::state(); uint8_t m=currentMenuItem(); SSTLedModeEx ledmodeex=sst_ledmode_none;
    if ( s==TBUTTONBASE::longpressed )
      ledmodeex=sst_ledmode_exec; // signal longpressed (longreleased will exec selected menu item)
    else
      switch (m) {
        case SST_BUTTONMENU_NONE: ledmodeex=sst_ledmode_off; break;
        case SST_BUTTONMENU_STARTMENU: ledmodeex=sst_ledmode_menu_startmenu; break;
        case SST_BUTTONMENU_STARTPAIRING: ledmodeex=sst_ledmode_menu_startpairing; break;
        case SST_BUTTONMENU_SETRADIOPOWER: ledmodeex=sst_ledmode_menu_setradiopower; break;
        case SST_BUTTONMENU_STARTRADIOREQADJUST: ledmodeex=sst_ledmode_menu_startfreqadjust; break;
        case SST_BUTTONMENU_RESETDEVICE: ledmodeex=sst_ledmode_menu_resetdevice; break;
        case SST_BUTTONMENU_RESETSOFT: ledmodeex=sst_ledmode_menu_resetsoft; break;
        case SST_BUTTONMENU_SWITCHOFF: ledmodeex=sst_ledmode_menu_switchoff; break;
        case SST_BUTTONMENU_MODENORMAL: ledmodeex=sst_ledmode_menu_modenormal; break;
        case SST_BUTTONMENU_SHOWVERSION: ; ledmodeex=sst_ledmode_menu_showversion; break;
        case SST_BUTTONMENU_MODEDEBUGVBAT: ledmodeex=sst_ledmode_menu_modedebugvbat; break;
        case SST_BUTTONMENU_RADIOPOWER_LOW: ledmodeex=sst_ledmode_menu_radiopower1; break;
        case SST_BUTTONMENU_RADIOPOWER_NORMAL: ledmodeex=sst_ledmode_menu_radiopower2; break;
        case SST_BUTTONMENU_RADIOPOWER_HIGH: ledmodeex=sst_ledmode_menu_radiopower3; break;
      }
    m_device.led().menu( isActive() );
    if ( ledmodeex!=sst_ledmode_none )   m_device.led().setEx( ledmodeex );
  }
  inline void exec() {
    m_signal=sst_ledmode_none; uint8_t menuitem=currentMenuItem();
    SSTPRINTF( "Menu exec %i: %u\n", m_currentmenuoffset, menuitem );
    m_currentmenuoffset=-1; updateLed();
    switch (menuitem) {
      case SST_BUTTONMENU_STARTMENU: m_currentmenuoffset=0; updateLed(); break;
      case SST_BUTTONMENU_STARTPAIRING: m_device.startPairing(); break;
      case SST_BUTTONMENU_SETRADIOPOWER: m_currentmenuoffset=findMenu(getRadioPowerMenu()); updateLed(); break;
      case SST_BUTTONMENU_STARTRADIOREQADJUST: m_device.freqadjust(2); break;
      case SST_BUTTONMENU_RESETDEVICE: if (deviceresetdisabled()) m_signal=sst_ledmode_signal_error; else m_device.reset(); break;
      case SST_BUTTONMENU_RESETSOFT: SST_SystemReset(); break;
      case SST_BUTTONMENU_SWITCHOFF: m_signal=sst_ledmode_signal_ok; m_action=SST_MB_ACTION_SWITCHOFF; break;
      case SST_BUTTONMENU_MODENORMAL: SSTSetRunMode(SST_RUNMODE_NORMAL); m_signal=sst_ledmode_signal_ok; m_action=SST_MB_ACTION_MEASURE; break;
      case SST_BUTTONMENU_MODEDEBUGVBAT: SSTSetRunMode(SST_RUNMODE_DEBUG_VBAT); m_signal=sst_ledmode_signal_ok; m_action=SST_MB_ACTION_MEASURE; break;
      case SST_BUTTONMENU_SHOWVERSION: m_signal=showVersion(version1,version2);  break;
      case SST_BUTTONMENU_RADIOPOWER_LOW:
      case SST_BUTTONMENU_RADIOPOWER_NORMAL:
      case SST_BUTTONMENU_RADIOPOWER_HIGH: setRadioPowerMenu(menuitem); m_signal=sst_ledmode_signal_ok; break;
    }
    if ( m_signal!=sst_ledmode_none )
      m_device.led().setEx( m_signal );
  }
  uint8_t currentMenuItem() const {
    return (m_currentmenuoffset<0) ? SST_BUTTONMENU_NONE : (m_pmenudef[m_currentmenuoffset]);
  }
  inline bool deviceresetdisabled()  {
    return m_device.getList0().localResetDisable();
  }
  bool isActive()  {
    if ( m_signal!=sst_ledmode_none && !m_device.led().active() )  {
       m_signal=sst_ledmode_none; // signal ok/error done
    }
    if ( m_signal==sst_ledmode_none && m_action==SST_MB_ACTION_MEASURE ) {
       m_action=SST_MB_ACTION_NONE; measure(); // measure after signal
    }
    return (m_currentmenuoffset>=0) || (m_signal!=sst_ledmode_none) || (m_action>0) || (this->pinstate==LOW);
  }
  bool switchOff() const {
    return m_action==SST_MB_ACTION_SWITCHOFF;
  }
  virtual void measure() {
  }
  SSTLedModeEx showVersion(const uint8_t ver1, const uint8_t ver2) {
    m_device.led().setEx(sst_ledmode_off); m_device.led().ledOff();
    Delay(500);
    for ( uint8_t sub=0; sub<2; sub++ ) {
      uint8_t ver=(sub==0)?ver1:ver2;
      for ( uint8_t nr=0; nr<ver; nr++ ) {
        m_device.led().ledOnEx(); Delay(150);
        m_device.led().ledOff(); Delay(250);
      }
      Delay(600);
    }
    #ifdef NDEBUG
    return sst_ledmode_signal_ok;
    #else
    return sst_ledmode_signal_debug;
    #endif
  }
  uint8_t getRadioPowerMenu() {
    uint8_t p=m_device.getHal().config_getradiopower();
    if ( p==1 )  return SST_BUTTONMENU_RADIOPOWER_LOW;
    if ( p==3 )  return SST_BUTTONMENU_RADIOPOWER_HIGH;
    return SST_BUTTONMENU_RADIOPOWER_NORMAL;
  }
  void setRadioPowerMenu( uint8_t m ) {
	uint8_t p=0;
	if ( m==SST_BUTTONMENU_RADIOPOWER_LOW )          p=1;
	else if ( m==SST_BUTTONMENU_RADIOPOWER_NORMAL )  p=2;
	else if ( m==SST_BUTTONMENU_RADIOPOWER_HIGH )    p=3;
	if ( p==0 )  return;
    m_device.getHal().config_setradiopower(m_device.getConfigArea(), p);
  }
};



// TODO
//  development state: experimental
//  save best freq in storage
template<class DEVTYPEBASE, class DEVINFOTYPE, class MSGTYPE, class CLOCKTYPE, class ALARMTYPE>
class SSTRadioFreqAdjust : public DEVTYPEBASE {
protected:
  uint32_t m_RadioFreqAdjustDeviceID;
  uint8_t m_RadioFreqAdjustMode;
  int8_t m_RadioFreqAdjustTicks;
  uint16_t m_RadioFreqAdjustFreq;
  int16_t m_RadioFreqAdjustOfs;
  enum { SST_RFA_NONE=0, SST_RFA_INITRADIO, SST_RFA_SETFREQ, SST_RFA_SENDDEVINFO, SST_RFA_STOP, SST_RFA_SAVE };
  uint8_t m_RadioFreqAdjustAction;
private:
  uint8_t m_received, m_rssi, m_bestrssi;
  uint16_t m_bestfreq;
  SSTLedModeEx m_nextledmode;
public:
  enum { SST_RADIOFREQADJUST_NONE=0, SST_RADIOFREQADJUST_PAIR=1, SST_RADIOFREQADJUST_OPTIMIZE=2, SST_RADIOFREQADJUST_DONE=99 };
  enum { SST_CC1101_FREQ2=0x0D, SST_CC1101_FREQ1=0x0E, SST_CC1101_FREQ0=0x0F };
  SSTRadioFreqAdjust(const DEVINFOTYPE& i,uint16_t addr) : DEVTYPEBASE(i,addr) {
	  m_RadioFreqAdjustDeviceID=((uint32_t)i.DeviceID[0]) << 16 | ((uint16_t)i.DeviceID[1]) << 8 | i.DeviceID[0];
	  m_RadioFreqAdjustMode=SST_RADIOFREQADJUST_NONE; m_RadioFreqAdjustTicks=0;
	  m_RadioFreqAdjustFreq=0; m_RadioFreqAdjustOfs=0;
	  m_RadioFreqAdjustAction=SST_RFA_NONE;
	  m_received=0; m_rssi=0; m_bestfreq=0; m_bestrssi=0; m_nextledmode=sst_ledmode_none;
  }
  virtual ~SSTRadioFreqAdjust() {
  }
  virtual bool process(MSGTYPE& msg) {
	if ( m_RadioFreqAdjustMode==SST_RADIOFREQADJUST_OPTIMIZE && m_RadioFreqAdjustFreq!=0 )  {
	  uint32_t masterID=(DEVTYPEBASE::getList0().masterid());
	  uint32_t from=(uint32_t)msg.from();
	  if( from==masterID )
	  {
        if ( m_received<100 )   m_received++;
        m_rssi=max(m_rssi,DEVTYPEBASE::radio().rssi());
        // SSTPRINTF( "Received from %h cnt %u rssi %u masterid %h\n",from,m_received,m_rssi,masterID);
	  }
	}
	return DEVTYPEBASE::process(msg);
  }
  void toogleRadioFreqAdjust() {
	  if ( m_RadioFreqAdjustMode>0 )  stopRadioFreqAdjust();
	  else                            startRadioFreqAdjust();
  }
  void startRadioFreqAdjust() {
	  if ( m_RadioFreqAdjustMode>0 )  return;
	  // DEVTYPEBASE::activity().stayAwake( 20*SST_TICKS_PER_SECOND );
	  if ( isPaired() )  m_RadioFreqAdjustMode=SST_RADIOFREQADJUST_OPTIMIZE;
	  else               m_RadioFreqAdjustMode=SST_RADIOFREQADJUST_PAIR;
	  SSTPRINTF( "RFA start freq. adjust: mode %u\n",m_RadioFreqAdjustMode);
	  m_RadioFreqAdjustTicks=-1; m_RadioFreqAdjustFreq=0; m_RadioFreqAdjustOfs=0;
	  m_RadioFreqAdjustAction=SST_RFA_INITRADIO;
	  m_received=0; m_rssi=0; m_bestfreq=0; m_bestrssi=0;
      SST_SYSTICK_AddCallbackParam( radioFreqAdjustTickSecCB, 1000, this );
  }
  void stopRadioFreqAdjust() {
    if ( m_RadioFreqAdjustMode==SST_RADIOFREQADJUST_NONE )  return;
    if ( m_RadioFreqAdjustMode==SST_RADIOFREQADJUST_DONE )  return;
    SSTPRINTS( "RFA stop freq. adjust\n");
    SST_SYSTICK_RemoveCallback( (void*)radioFreqAdjustTickSecCB );
    // DEVTYPEBASE::activity().stayAwake( 2*SST_TICKS_PER_SECOND );
    m_RadioFreqAdjustMode=SST_RADIOFREQADJUST_DONE;
  }
  bool radioFreqAdjust() {
	switch ( m_RadioFreqAdjustAction ) {
	  case SST_RFA_INITRADIO:
          SSTPRINTF( "RFA init radio: masterid %h\n", (uint32_t)(DEVTYPEBASE::getList0().masterid()) );
		  // DEVTYPEBASE::activity().stayAwake( 100*SST_TICKS_PER_SECOND );
		  DEVTYPEBASE::radio().wakeup();
		  this->sendDeviceInfo();
		  break;
	  case SST_RFA_SETFREQ:
          SSTPRINTF( "RFA set freq. ofs %i\n",m_RadioFreqAdjustOfs);
		  // setFreq();
		  // DEVTYPEBASE::activity().stayAwake( 100*SST_TICKS_PER_SECOND );
		  break;
	  case SST_RFA_SENDDEVINFO:
		  setFreq();
	      SSTPRINTS( "RFA send dev info\n");
		  this->sendDeviceInfo(); break;
	  case SST_RFA_STOP:
		  stopRadioFreqAdjust();
		  DEVTYPEBASE::radio().setIdle();
		  // DEVTYPEBASE::activity().stayAwake( SST_TICKS_PER_SECOND );
		  break;
	  case SST_RFA_SAVE:
          SSTPRINTF( "Best freq. %h rssi %u\n ",m_bestfreq,m_bestrssi);
		  stopRadioFreqAdjust();
		  DEVTYPEBASE::radio().setIdle();
		  break;
	}
	m_RadioFreqAdjustAction=SST_RFA_NONE;
    if ( m_nextledmode!=sst_ledmode_none && !(DEVTYPEBASE::led().active()) ) {
      DEVTYPEBASE::led().setEx( m_nextledmode );
      m_nextledmode=sst_ledmode_none;
    }
    if ( m_RadioFreqAdjustMode==SST_RADIOFREQADJUST_DONE && !(DEVTYPEBASE::led().active()) )
      m_RadioFreqAdjustMode=SST_RADIOFREQADJUST_NONE;
    return (m_RadioFreqAdjustMode>0);
  }
protected:
  static void radioFreqAdjustTickSecCB( void *param )  {
     ((SSTRadioFreqAdjust*)param)->radioFreqAdjustTickSec();
  }
  void radioFreqAdjustTickSec()  {
    SSTPRINTS(".");
    // DEVTYPEBASE::led().sleep(false); DEVTYPEBASE::led().enable(); DEVTYPEBASE::led().ledOn( 5, 0 );
    if ( m_RadioFreqAdjustAction!=SST_RFA_NONE )  return;
    SST_Watchdog_Feed();
    if ( m_RadioFreqAdjustMode==SST_RADIOFREQADJUST_PAIR ) {
      m_RadioFreqAdjustTicks++; if ( m_RadioFreqAdjustTicks>5 )  m_RadioFreqAdjustTicks=0;
      if ( m_RadioFreqAdjustTicks!=0 )  return;
	  SSTPRINTS( "Next freq. adjust\n ");
	  if ( isPaired() ) {
        m_RadioFreqAdjustAction=SST_RFA_STOP;
	  } else {
	    bool done=nextStep();
	    if ( done )  m_RadioFreqAdjustAction=SST_RFA_STOP;
        else         m_RadioFreqAdjustAction=SST_RFA_SENDDEVINFO;
      }
    }
    if ( m_RadioFreqAdjustMode==SST_RADIOFREQADJUST_OPTIMIZE ) {
      if ( m_RadioFreqAdjustAction!=SST_RFA_NONE )  return;
      SST_Watchdog_Feed();
      if ( m_RadioFreqAdjustTicks<0 ) {
   	    SSTPRINTS( "RFA init steps\n");
        nextStep(); // init
        m_RadioFreqAdjustAction=SST_RFA_SETFREQ;
      }
      else if ( m_RadioFreqAdjustTicks>60 ) {
        m_RadioFreqAdjustAction=SST_RFA_SAVE;
        m_nextledmode=sst_ledmode_signal_error;
      }
      else if ( m_received>0 )  {
        m_RadioFreqAdjustTicks=0;
    	if ( m_rssi>m_bestrssi )  {
          m_bestfreq=m_RadioFreqAdjustFreq+m_RadioFreqAdjustOfs; m_bestrssi=m_rssi;
    	}
    	m_received=0; m_rssi=0;
    	bool done=nextStep();
        if ( done )  {
           m_RadioFreqAdjustAction=SST_RFA_SAVE;
        }
        else {
           m_RadioFreqAdjustAction=SST_RFA_SETFREQ;
           if ( m_nextledmode!=sst_ledmode_signal_error )
              m_nextledmode=sst_ledmode_signal_ok;
        }
      }
      else if ( m_nextledmode==sst_ledmode_none ) {
    	  m_nextledmode=sst_ledmode_signal_running;
      }
      m_RadioFreqAdjustTicks++;
    }
  }
  bool nextStep() {
	  int16_t step=(m_RadioFreqAdjustMode==SST_RADIOFREQADJUST_OPTIMIZE)?10:50;
	  int16_t maxofs=(m_RadioFreqAdjustMode==SST_RADIOFREQADJUST_OPTIMIZE)?0x150:0x300;
	  if ( m_RadioFreqAdjustFreq==0 ) {
		  m_RadioFreqAdjustFreq=0x656A; m_RadioFreqAdjustOfs=0;
	  } else {
		  if ( m_RadioFreqAdjustOfs==0 )       m_RadioFreqAdjustOfs=step;
		  else if ( m_RadioFreqAdjustOfs>0 )   m_RadioFreqAdjustOfs=-m_RadioFreqAdjustOfs;
		  else                                 m_RadioFreqAdjustOfs=-m_RadioFreqAdjustOfs+step;
	  }
	  return m_RadioFreqAdjustOfs>maxofs;
  }

  void setFreq() {
	uint16_t freq=m_RadioFreqAdjustFreq+m_RadioFreqAdjustOfs;
    SSTPRINTF( "AdjustFreq %h\n", freq );
    DEVTYPEBASE::radio().tuneFreq( 0x21, freq >> 8, freq & 0xff );
    // DEVTYPEBASE::radio().initReg(SST_CC1101_FREQ2, 0x21);
    // DEVTYPEBASE::radio().initReg(SST_CC1101_FREQ1, freq >> 8);
    // DEVTYPEBASE::radio().initReg(SST_CC1101_FREQ0, freq & 0xff);
  }
  bool isPaired() {
	return ((uint32_t)(DEVTYPEBASE::getList0().masterid()))!=0;
  }
  void received( uint8_t rssi ) {

  }

  /*
  virtual void trigger(CLOCKTYPE&) {

  }
  bool deviceIsPaired() {

  //if (sdev.getMasterID() == HMID::broadcast) { DPRINTLN(F("START PAIRING")); sdev.startPairing(); } // start pairing of no master id is present
  }
  void start() {

  }
  */
};

// SWO debug serial (asksin Debug.h, see arduino print class)
#if !defined(NDEBUG)
#define DEC 10
#define HEX 16
// #define OCT 8  not supported now
#ifdef BIN
#undef BIN
#endif
// #define BIN 2  not supported now
#if !defined(NDEBUG_SWO)
#define DINITSWO SSTInitSWODebug
#endif
#endif

#if defined(NDEBUG) || defined(NDEBUG_SWO)

#define DHWINFO()
#if defined(NDEBUG_SWO)
#define SST_SWODEBUG_BITRATE_STR(bitrate)
#define DSERIAL _SST_SWODebugSerial // static class for asksin Debug.h
#define DINITSWO(b,s)
class SST_SWODebugSerial {
public:
  void begin( uint32_t baudrate )  {}
  size_t print( const char *str )  { return 0; }
  // size_t print(const char[] ac )  {}
  size_t print(char c) { return 0; }
  size_t print(unsigned char c, int t=0 ) { return 0; }
  size_t print(short s, int t=0) { return 0; }
  size_t print(unsigned short s, int t=0) { return 0; }
  size_t print(long l, int t=0)	{ return 0; }
  size_t print(unsigned long u, int t=0) { return 0; }
  size_t print( int i, int t=0)	{ return 0; }
  // size_t print(double, int = 2);
};
extern SST_SWODebugSerial _SST_SWODebugSerial;
#endif

#else

#define DHWINFO() SSTWriteSWODebugHWInfo()
#define SST_SWODEBUG_BITRATE_STR(bitrate) "SWODebug (bitrate " SST_STRINGIZE(bitrate) ")"
#define DSERIAL _SST_SWODebugSerial // static class for asksin Debug.h
class SST_SWODebugSerial {
public:
  void begin( uint32_t baudrate )  {
    // init in setup() by SSTInitSWODebug( baudrate, mode );
  }
  size_t print( const char *str ) { return SSTWriteSWODebugString(str); }
  // size_t print(const char[] ac )  {}
  size_t print(char c)  { return SSTWriteSWODebugChar(c); }
  size_t print(unsigned char c, int t=DEC )  { return print((unsigned long)c,t); }
  size_t print(short s, int t=DEC)  {	return print( (long)s, t );	}
  size_t print(unsigned short s, int t=DEC)  { return print((unsigned long)s,t); }
  size_t print(long l, int t=DEC) {
    if ( t==DEC )  return SSTWriteSWODebugInt( l, 16 );
    if ( t==HEX )  return SSTWriteSWODebugInt( l, 16 );
    return 0;
  }
  size_t print(unsigned long u, int t=DEC)  {
    if ( t==DEC )  return SSTWriteSWODebugUInt( u, 10 );
    if ( t==HEX )  return SSTWriteSWODebugUInt( u, 16 );
    return 0;
  }
  size_t print( int i, int t=DEC)  { return print((long)i,t); }
  // size_t print(double, int = 2);
};
extern SST_SWODebugSerial _SST_SWODebugSerial;
#endif // #ifndef NDEBUG

#endif /* _SST_ARDUINO_GECKOSDK_H_ */
