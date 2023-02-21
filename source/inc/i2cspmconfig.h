// I2CSPM driver config

#ifndef _I2CSPM_CONFIG_H__
#define _I2CSPM_CONFIG_H__

/* I2C SPM driver config. This default override only works if one I2C interface
   is in use. If multiple interfaces are in use, define the peripheral setup
   inside the application in a I2CSPM_Init_TypeDef and then pass the initialization
   struct to I2CSPM_Init(). */
#define I2CSPM_INIT_DEFAULT                                                    \
  { I2C0,                       /* Use I2C instance 0 */                       \
    gpioPortD,                  /* SCL port */                                 \
    15,                         /* SCL pin */                                  \
    gpioPortD,                  /* SDA port */                                 \
    14,                         /* SDA pin */                                  \
    3,                          /* Location */                                 \
    0,                          /* Use currently configured reference clock */ \
    I2C_FREQ_STANDARD_MAX,      /* Set to standard rate  */                    \
    i2cClockHLRStandard,        /* Set to use 4:4 low/high duty cycle */       \
  }

#define I2CSPM_TRANSFER_TIMEOUT 300000

#endif /* _I2CSPM_CONFIG_H__ */
