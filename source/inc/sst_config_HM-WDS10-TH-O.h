// SST configuration file

// SST options
#define SST_OPT_BATTERYMONITOR       0           // 0=no external battery monitor, use internal vcc adc
                                                 // 1=ext. battery monitor assembled (IC4, R6, R7)
// define SST_OPT_BATTERYMONITOR_DIV 50          // voltage divider factor in 1/100
#define SST_OPT_POWERON_I2C          0           // 0=I2C power is always on (JP1 bridged)
                                                 // 1=I2C power switched (IC2 assembled)
                                                 // 2=auto detect (may not work in all assemblies)
#define SST_OPT_POWERON_TRX          1           // 0=SPI power is always on (JP3 bridged)
                                                 // 1=SPI power switched (IC7 assembled)
// #define SST_OPT_BITRATE_TRX          1000000   // SPI bitrate (default 1000000)
#define SST_OPT_EEPROM               2           // 0=no eeprom/use internal flash
                                                 // 1=M24M01
                                                 // 2=M24256
                                                 // 9=M24xxx compatible, define size and pagesize
// #define SST_OPT_EEPROM_I2CADDR    0xA0        // M24xxx compatible: i2c address
// #define SST_OPT_EEPROM_SIZE       32          // M24xxx compatible: eeprom size in kbytes                    (eg. M24M01: 128, M24256: 32)
// #define SST_OPT_EEPROM_PAGE       64          // M24xxx compatible: eeprom page size in bytes (default=256)  (eg. M24M01: 256, M24256: 64)
#define SST_OPT_SENSOR_SHT4X         1           // 0=not is assembled
                                                 // 1=IC53 SSHT4x (A-Type) sensor is assembled, i2c address 0x44
                                                 // 2=IC53 SSHT4x (B-Type) sensor is assembled, i2c address 0x45
// #define SST_OPT_SENSOR_SHT4X_ADDR 0x44        // define SSHT4x compatible device other I2C address
#define SST_OPT_DISABLE_WATCHDOG     0           // 1=Watchdog is disabled (eg. for debugging)
#define SST_OPT_SWO_DEBUG            1           // 1=SWO used for debug output (PF2 pin)
#define SST_OPT_SWO_DEBUG_BITRATE    30500       // SWO output bitrate see sst_arduino_geckosdk.cpp section "SWO debug"
#define SST_OPT_BATTERY              1           // 0=none
                                                 // 1=battery VL2020 is installed (Vanadium-Lithium full=3,3V, end of discharge 2,5V)
                                                 // 99=other (define your own battery parameters SST_OPT_BATTERY_xxx)
// #define SST_OPT_BATTERY_LOW          27          // battery parameter: low voltage 27 = 2,70V (check is on less, measured 2,6V will generate a battery low warning)
// #define SST_OPT_BATTERY_CRITICAL     26          // battery parameter: critical voltage 26 = 2,60V
// #define SST_OPT_BATTERY_OVERCHARGE   33          // battery parameter: end of charge voltage
// #define SST_OPT_BATTERY_INTERVAL     (12U*60*60) // battery measure interval in seconds or 0 (THSensor: battery is checked before every sensor measure)
#define SST_OPT_BATTERY_PROTECTION   1           // 1= software over charge protection
#define SST_OPT_BATTERY_DEBUGTEST    0           // depreciated: 1= debug/test battery charging by sending battery voltage as temperature value
#define SST_OPT_RADIO_FREQADJUST     0           // 1= option adjust CC1101 frequency by scanning CCU messages rssi

// asksin options
#define DEVICE_MODEL                 0x3d        // HM device model, 0x3D = HM-WDS10-TH-O
#define DEVICE_TYPE                  THSensor    // HM device type, see Device.h DeviceType
#define FIRMWARE_VERSION             0x10        // HM device firmware version, 0x10 = 1.0
#define SENSOR_ONLY                  1           // asksin option
#define EXTRAMILLIS                  120000U     // HM BatterySensor default measure interval is about 150 seconds +/- random, add extra time to measure every 4,5 minutes
// #define SIMPLE_CC1101_INIT           1           // HM radio-cc1101

