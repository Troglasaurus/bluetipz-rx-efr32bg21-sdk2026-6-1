#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

// $[CMU]
// [CMU]$

// $[LFXO]
// [LFXO]$

// $[PRS.ASYNCH0]
// [PRS.ASYNCH0]$

// $[PRS.ASYNCH1]
// [PRS.ASYNCH1]$

// $[PRS.ASYNCH2]
// [PRS.ASYNCH2]$

// $[PRS.ASYNCH3]
// [PRS.ASYNCH3]$

// $[PRS.ASYNCH4]
// [PRS.ASYNCH4]$

// $[PRS.ASYNCH5]
// [PRS.ASYNCH5]$

// $[PRS.ASYNCH6]
// [PRS.ASYNCH6]$

// $[PRS.ASYNCH7]
// [PRS.ASYNCH7]$

// $[PRS.ASYNCH8]
// [PRS.ASYNCH8]$

// $[PRS.ASYNCH9]
// [PRS.ASYNCH9]$

// $[PRS.ASYNCH10]
// [PRS.ASYNCH10]$

// $[PRS.ASYNCH11]
// [PRS.ASYNCH11]$

// $[PRS.SYNCH0]
// [PRS.SYNCH0]$

// $[PRS.SYNCH1]
// [PRS.SYNCH1]$

// $[PRS.SYNCH2]
// [PRS.SYNCH2]$

// $[PRS.SYNCH3]
// [PRS.SYNCH3]$

// $[GPIO]
// [GPIO]$

// $[TIMER0]
// [TIMER0]$

// $[TIMER1]
// [TIMER1]$

// $[TIMER2]
// [TIMER2]$

// $[TIMER3]
// [TIMER3]$

// $[USART0]
// USART0 RX on PA06
#ifndef USART0_RX_PORT                          
#define USART0_RX_PORT                           SL_GPIO_PORT_A
#endif
#ifndef USART0_RX_PIN                           
#define USART0_RX_PIN                            6
#endif

// USART0 TX on PA05
#ifndef USART0_TX_PORT                          
#define USART0_TX_PORT                           SL_GPIO_PORT_A
#endif
#ifndef USART0_TX_PIN                           
#define USART0_TX_PIN                            5
#endif

// [USART0]$

// $[USART1]
// [USART1]$

// $[USART2]
// [USART2]$

// $[I2C1]
// [I2C1]$

// $[LETIMER0]
// [LETIMER0]$

// $[IADC0]
// [IADC0]$

// $[ACMP0]
// [ACMP0]$

// $[ACMP1]
// [ACMP1]$

// $[I2C0]
// [I2C0]$

// $[PTI]
// [PTI]$

// $[MODEM]
// [MODEM]$

// $[CUSTOM_PIN_NAME]
#ifndef _PORT                                   
#define _PORT                                    SL_GPIO_PORT_A
#endif
#ifndef _PIN                                    
#define _PIN                                     0
#endif




#ifndef BUZZ_EN_PORT                            
#define BUZZ_EN_PORT                             SL_GPIO_PORT_A
#endif
#ifndef BUZZ_EN_PIN                             
#define BUZZ_EN_PIN                              4
#endif

#ifndef dbg_tx_PORT                             
#define dbg_tx_PORT                              SL_GPIO_PORT_A
#endif
#ifndef dbg_tx_PIN                              
#define dbg_tx_PIN                               5
#endif

#ifndef dbg_rx_PORT                             
#define dbg_rx_PORT                              SL_GPIO_PORT_A
#endif
#ifndef dbg_rx_PIN                              
#define dbg_rx_PIN                               6
#endif



#ifndef NTC_SENSOR_EN_PORT                      
#define NTC_SENSOR_EN_PORT                       SL_GPIO_PORT_C
#endif
#ifndef NTC_SENSOR_EN_PIN                       
#define NTC_SENSOR_EN_PIN                        0
#endif

#ifndef NTC_SENSOR_PORT                         
#define NTC_SENSOR_PORT                          SL_GPIO_PORT_C
#endif
#ifndef NTC_SENSOR_PIN                          
#define NTC_SENSOR_PIN                           1
#endif


#ifndef LED2_EN_PORT                            
#define LED2_EN_PORT                             SL_GPIO_PORT_C
#endif
#ifndef LED2_EN_PIN                             
#define LED2_EN_PIN                              3
#endif

#ifndef LED1_EN_PORT                            
#define LED1_EN_PORT                             SL_GPIO_PORT_C
#endif
#ifndef LED1_EN_PIN                             
#define LED1_EN_PIN                              4
#endif

#ifndef PWR_BTN_PORT                            
#define PWR_BTN_PORT                             SL_GPIO_PORT_C
#endif
#ifndef PWR_BTN_PIN                             
#define PWR_BTN_PIN                              5
#endif





#ifndef VIB_EN_PORT                             
#define VIB_EN_PORT                              SL_GPIO_PORT_D
#endif
#ifndef VIB_EN_PIN                              
#define VIB_EN_PIN                               4
#endif

// [CUSTOM_PIN_NAME]$


#endif // PIN_CONFIG_H


