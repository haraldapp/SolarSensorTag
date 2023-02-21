// SST pin names
// xxx_PIN    pin number for gecko sdk gpio functions
// xxx_PORT   port number for gecko sdk gpio functions
// xxx_APIN   pin for arduino like api functions (see sst_arduino_gecksdk.h)

#ifndef _SST_PINS_H_
#define _SST_PINS_H_

#define GPIOPORTA 0
#define GPIOPORTB 1
#define GPIOPORTC 2
#define GPIOPORTD 3
#define GPIOPORTE 4
#define GPIOPORTF 5

#define INT0_PIN                       (0U)
#define INT0_PORT                      (gpioPortA)
#define INT0_APIN                      MAKE_APIN(GPIOPORTA,INT0_PIN)

#define INT1_PIN                       (1U)
#define INT1_PORT                      (gpioPortA)
#define INT1_APIN                      MAKE_APIN(GPIOPORTA,INT1_PIN)

#define I2C_PWR_PIN                    (2U)
#define I2C_PWR_PORT                   (gpioPortA)
#define I2C_PWR_APIN                   MAKE_APIN(GPIOPORTA,I2C_PWR_PIN)

#define LX1_PIN                        (7U)
#define LX1_PORT                       (gpioPortB)

#define LX2_PIN                        (8U)
#define LX2_PORT                       (gpioPortB)

#define PB0_PIN                        (11U)
#define PB0_PORT                       (gpioPortB)
#define PB0_APIN                       MAKE_APIN(GPIOPORTB,PB0_PIN)

#define X1_PIN                         (13U)
#define X1_PORT                        (gpioPortB)

#define X2_PIN                         (14U)
#define X2_PORT                        (gpioPortB)

#define I2C_SDA_PIN                    (0U)
#define I2C_SDA_PORT                   (gpioPortC)
#define I2C_SDA_APIN                   MAKE_APIN(GPIOPORTC,I2C_SDA_PIN)

#define I2C_SCL_PIN                    (1U)
#define I2C_SCL_PORT                   (gpioPortC)
#define I2C_SCL_APIN                   MAKE_APIN(GPIOPORTC,I2C_SCL_PIN)

#define SPI_PWR_PIN                    (13U)
#define SPI_PWR_PORT                   (gpioPortC)
#define SPI_PWR_APIN                   MAKE_APIN(GPIOPORTC,SPI_PWR_PIN)

#define SPI_SS_PIN                     (14U)
#define SPI_SS_PORT                    (gpioPortC)
#define SPI_SS_APIN                    MAKE_APIN(GPIOPORTC,SPI_SS_PIN)

#define SPI_SCK_PIN                    (15U)
#define SPI_SCK_PORT                   (gpioPortC)
#define SPI_SCK_APIN                   MAKE_APIN(GPIOPORTC,SPI_SCK_PIN)

#define ADC0_PIN                       (4U)
#define ADC0_PORT                      (gpioPortD)
#define ADC0_APIN                      MAKE_APIN(GPIOPORTD,ADC0_PIN)

#define VBAT_M_PIN                     (5U)
#define VBAT_M_PORT                    (gpioPortD)
#define VBAT_M_APIN                    MAKE_APIN(GPIOPORTD,VBAT_M_PIN)

#define SPI_MISO_PIN                   (6U)
#define SPI_MISO_PORT                  (gpioPortD)
#define SPI_MISO_APIN                  MAKE_APIN(GPIOPORTD,SPI_MISO_PIN)

#define SPI_MOSI_PIN                   (7U)
#define SPI_MOSI_PORT                  (gpioPortD)
#define SPI_MOSI_APIN                  MAKE_APIN(GPIOPORTD,SPI_MOSI_PIN)

#define LED_R_PIN                      (10U)
#define LED_R_PORT                     (gpioPortE)
#define LED_R_APORT                    4
#define LED_R_APIN                     MAKE_APIN(GPIOPORTE,LED_R_PIN)

#define LED_G_PIN                      (11U)
#define LED_G_PORT                     (gpioPortE)
#define LED_G_APORT                    4
#define LED_G_APIN                     MAKE_APIN(GPIOPORTE,LED_G_PIN)

#define VBAT_M_ON_PIN                  (12U)
#define VBAT_M_ON_PORT                 (gpioPortE)
#define VBAT_M_ON_APIN                 MAKE_APIN(GPIOPORTE,VBAT_M_ON_PIN)

#define PWR_ON_PIN                     (13U)
#define PWR_ON_PORT                    (gpioPortE)
#define PWR_ON_APIN                    MAKE_APIN(GPIOPORTE,PWR_ON_PIN)

#define SWO_PIN                        (2U)
#define SWO_PORT                       (gpioPortF)
#define SWO_APIN                       MAKE_APIN(GPIOPORTF,SWO_PIN)

#endif // #ifndef _SST_PINS_H_
