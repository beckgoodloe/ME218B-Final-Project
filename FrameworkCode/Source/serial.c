/****************************************************************************
 Module
   Serial.c

 Revision
   1.0

 Description
   This is a module to provide SPI initilization and communication.

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "serial.h"
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "inc/hw_nvic.h"
#include "bitdefs.h"
#include "termio.h"

#include "ES_Configure.h"
#include "ES_Framework.h"
#include "spudFSM.h"


/*----------------------------- Module Defines ----------------------------*/
#define BitsPerNibble 4
#define SSI_PRESCALE 0x14
#define SCR_VALUE 0xCA00
/*---------------------------- Module Functions ---------------------------*/


/*---------------------------- Module Variables ---------------------------*/


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitSerial

 Parameters
     None

 Returns
     None
     
 Description
     Initializes SPI 0 and relevant interrupts.

****************************************************************************/
void InitSerial(void)
{
  // Enable the clock to the GPIO port
  HWREG(SYSCTL_RCGCGPIO) |= SYSCTL_RCGCGPIO_R0;
  // Enable the clock to the SSI module
  HWREG(SYSCTL_RCGCSSI) |= SYSCTL_RCGCGPIO_R0;
  // Wait for the GPIO port to be ready
  while ((HWREG(SYSCTL_PRGPIO) & SYSCTL_PRGPIO_R0) != SYSCTL_PRGPIO_R0)
  {}
  // Program the GPIO to use the alternate functions on the SSI pins
  HWREG(GPIO_PORTA_BASE + GPIO_O_AFSEL) |= (BIT2HI | BIT3HI | 
    BIT4HI | BIT5HI);
  // Set mux position in GPIOCTL to select the SSI use of the pins
  HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) |=
      (HWREG(GPIO_PORTA_BASE + GPIO_O_PCTL) & 0xff0000ff) +
      ((2 << (2*BitsPerNibble)) + (2 << (3*BitsPerNibble)) +
      (2 << (4*BitsPerNibble)) + (2 << (5*BitsPerNibble)));
  // Program the port lines for digital I/O
  HWREG(GPIO_PORTA_BASE + GPIO_O_DEN) |= (BIT2HI | BIT3HI | BIT4HI | BIT5HI);
  // Program the required data directions on the port lines
  // PA2: clock (out), PA3: SS (out), PA4: MISO (in), PA5: MOSI (out)
  HWREG(GPIO_PORTA_BASE + GPIO_O_DIR) |= (BIT2HI | BIT3HI | BIT5HI);
  HWREG(GPIO_PORTA_BASE + GPIO_O_DIR) &= BIT4LO;
  // Using SPI mode 3, so program the pull-up on the clock line
   HWREG(GPIO_PORTA_BASE + GPIO_O_PUR) |= BIT2HI;
  // Wait for the SSI0 to be ready
  while ((HWREG(SYSCTL_PRSSI) & SYSCTL_PRSSI_R0) != SYSCTL_PRSSI_R0)
  {}
  // Make sure that the SSI is disable before programming mode bits
  HWREG(SSI0_BASE + SSI_O_CR1) &= ~SSI_CR1_SSE;
  // Select master mode (MS) & TXRIS indicating end of transmit
  HWREG(SSI0_BASE + SSI_O_CR1) &= ~SSI_CR1_MS;
  HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_EOT;
  // Configure the SSI clock source to the system clock
  HWREG(SSI0_BASE + SSI_O_CC) = SSI_CC_CS_SYSPLL;
  // Configure the clock pre-scaler. We choose a rate of 910kHz.
  // Set SCR = 0, so CPSDVSR = 44.
  HWREG(SSI0_BASE + SSI_O_CPSR) &= 0xffffff00;
  HWREG(SSI0_BASE + SSI_O_CPSR) |= SSI_PRESCALE;
  // Configure clock rate (SCR), phase and polarity (SPH, SPO),
  // mode (FRF), data size (DSS)
  // SCR = 0, SPH = 1, SPO = 1
  HWREG(SSI0_BASE + SSI_O_CR0) &= 0xffff00ff;
  HWREG(SSI0_BASE + SSI_O_CR0) |= SCR_VALUE;
  HWREG(SSI0_BASE + SSI_O_CR0) |= (SSI_CR0_SPH | SSI_CR0_SPO);
  HWREG(SSI0_BASE + SSI_O_CR0) &= ~SSI_CR0_FRF_M;
  HWREG(SSI0_BASE + SSI_O_CR0) |= SSI_CR0_DSS_8;
  // Locally enable interrupts (TXIM in SSIIM)
  HWREG(SSI0_BASE + SSI_O_IM) |= (SSI_IM_TXIM);
  // Make sure that the SSI is enable for operation
  HWREG(SSI0_BASE + SSI_O_CR1) |= SSI_CR1_SSE;
  // Enable the NVIC interrupt for the SSI when starting to transmit
  HWREG(NVIC_EN0) = BIT7HI;
  printf("SPI0 Configured!\r\n");
}

/****************************************************************************
 Function
     TransmitSerial

 Parameters
     mesage: byte packet to transmit via serial

 Returns
     None

 Description
     Transmits message via SPI

****************************************************************************/
void TransmitSerial(uint8_t message)
{
  // Disable interrupt
  HWREG(SSI0_BASE + SSI_O_IM) &= ~SSI_IM_TXIM;
  // Load message into data register
  HWREG(SSI0_BASE + SSI_O_DR) = message; // Send read register command
  //HWREG(SSI0_BASE + SSI_O_DR) = 0x00; // Follow with two bytes of 0x00
  //HWREG(SSI0_BASE + SSI_O_DR) = 0x00; 
  // Re-enable interrupt
  HWREG(SSI0_BASE + SSI_O_IM) |= SSI_IM_TXIM;
}

/****************************************************************************
 Function
    ReceiveISR

 Parameters
   None

 Returns
   message, most recent packet received over serial

 Description
   Returns most recent packet received from SPI

****************************************************************************/
void ReceiveSerial(void)
{
  // Disable interrupt
  HWREG(SSI0_BASE + SSI_O_IM) &= ~SSI_IM_TXIM;
  
  // Read value from data register
  uint8_t ReturnValue = HWREG(SSI0_BASE + SSI_O_DR);
  ReturnValue = HWREG(SSI0_BASE + SSI_O_DR);
  ReturnValue = HWREG(SSI0_BASE + SSI_O_DR);
  
  // Post final received value to SPUD service
  ES_Event_t ThisEvent;
  ThisEvent.EventType = BYTE_RECEIVED;
  ThisEvent.EventParam = ReturnValue;
  PostSPUDFSM(ThisEvent);
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

