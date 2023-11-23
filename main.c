///* COMPMEC 462 001 *///
///* ALockwood *///
///* Anton Salah *///
///* Testing the HMC5883L Compass *///

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/softi2c.h"
#include "inc/tm4c123gh6pm.h"
#include "i_trig.h"

// TIME
#define one_micro_second 16
#define one_sec 16000000
#define one_tenth_sec 1600000
#define one_milli_sec 16000
#define SPEED_OF_SOUND_CM_PER_MS 343  // Speed of sound in cm/ms
#define US_TO_S 1000000  // Microseconds to seconds conversion factor

// Proof of life LEDs
#define LED_ON1 0x02
#define LED_ON2 0x04
#define LED_ON3 0x08
static bool redLEDon = false;

// Proof of Life function
void ProofOfLifeLEDsToggle(void){
    if (redLEDon){
        GPIO_PORTF_DATA_R &= ~LED_ON1;  // Turn off the red LED
        GPIO_PORTF_DATA_R |= LED_ON3;   // Turn on the green LED
    }
    else{
        GPIO_PORTF_DATA_R |= LED_ON1;   // Turn on the red LED
        GPIO_PORTF_DATA_R &= ~LED_ON3;  // Turn off the green LED
    }
    // Toggle the state for the next call
    redLEDon = !redLEDon;
}


//Anton Salah
void PrintDirection(int heading){

    if ((heading >= 338)||(heading < 23)){
        UARTprintf("North (N)\n");
    }else if((heading >=23) && (heading < 68)){
        UARTprintf("Northeast (NE)\n");
    }else if((heading >= 68) && (heading < 113)){
        UARTprintf("East (E)\n");
    }else if((heading >=113) && (heading < 158)){
        UARTprintf("Southeast (SE)\n");
    }else if((heading >=158) && (heading < 203)){
        UARTprintf("South (S)\n");
    }else if((heading >=203) &&(heading < 248)){
        UARTprintf("Southwest (SW)\n");
    }else if((heading >=248) &&(heading < 293)){
        UARTprintf("West (W)\n");
    }else if((heading >=293) &&(heading < 338)){
        UARTprintf("Northwest (NW)\n");
    }else{
        UARTprintf("Unknown\n");
    }

}

// Configure UART
void ConfigureUART(void){
    // Enable the GPIO Peripheral used by the UART.
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    // Enable UART0
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Configure GPIO Pins for UART mode.
    MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
    MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
    MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // Initialize the UART for console I/O.
    // Assumes uP clock is at 16 Mhz
    UARTStdioConfig(0, 115200, 16000000);
}


// Delay
void Delay(uint32_t count){
  volatile unsigned /*register*/ long i;
  for(i=count;i>0;i--);
}

// Initialize SysTick
void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0; // disable SysTick
  NVIC_ST_RELOAD_R = 0x00ffffff;
  NVIC_ST_CURRENT_R = 0;
  NVIC_ST_CTRL_R = 0x05; // use core clock
}

void SetClocks(void){
    // Set the clocking to run directly from the crystal.
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
}

void InitializePins(void){
    UARTprintf("Initializing");
    // configure ports
    // Initialize PORT A
    SYSCTL_RCGCGPIO_R |= 0x01;            // enable clock for PORTA
    GPIO_PORTA_DEN_R  |= 0x30;            // enable digital bits 4,5 on PORTA
    GPIO_PORTA_DIR_R  |= 0x30;            // make bits 4,5 output pins
    UARTprintf(".");
    // Initialize port B
    SYSCTL_RCGCGPIO_R |=  0x02;           // enable clock for PORTB
    GPIO_PORTB_DEN_R  |=  0xc0;           // digital enable bits 6,7 on PORTB
    GPIO_PORTB_DIR_R  &= ~0xc0;           // make bits 6,7 as input pins
    UARTprintf(".");
    // Initialize PORTF
    SYSCTL_RCGCGPIO_R |= 0x20;            // enable clock for PORTF
    GPIO_PORTF_DEN_R  |= 0x0e;            // digital enable bits 1..3 on PORTF
    GPIO_PORTF_DIR_R  |= 0x0e;            // make bits 1..3 as output pins
    UARTprintf(".");
    // Initialize port E:
    SYSCTL_RCGCGPIO_R |= 0x10;           // enable clock for PORTE
    GPIO_PORTE_DEN_R   |= 0x3f;           // enable digital bits 0..5 on PORTE
    GPIO_PORTE_DIR_R   |= 0x3f;           // make bits 0..5 output pins
    //GPIO_PORTF_DR8R_R |= 0x3f;          // select 8 ma pin drive
    UARTprintf(".");
    // Unlock Port F
    SYSCTL_RCGCGPIO_R |= 0x20;            // enable clock for PORTF
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= 0x01;
    GPIO_PORTF_LOCK_R = 0;
    UARTprintf(".");
    // Initialize port F
    GPIO_PORTF_DEN_R  |= 0x1f;            // digital enable bits 0..4 on PORTF
    GPIO_PORTF_DIR_R  |= 0x0e;            // make bits 1..3 as output pins
    GPIO_PORTF_PUR_R  |= 0x11;            // select bits 0,4 pullup
    UARTprintf(".");

    #define GPIO_PORTF_CLK_EN 0x20

    // We are finished initializing.
    UARTprintf(".");
    UARTprintf("\nInitialization Complete!\n");

}




/////////////////////////////////////COMPASS///////////////////////////////////////////////////////
// Assuming that each axis data is 2 bytes and we are reading all three axes
#define COMPASS_I2C_ADDRESS 0x1E  // Assuming the I2C address of the compass is 0x1E
#define COMPASS_REGISTER 0x03      // Assuming we are reading from register 0x03
#define DATA_BYTES_PER_AXIS 2
#define TOTAL_DATA_BYTES (3 * DATA_BYTES_PER_AXIS)


void InitializeI2C(void){
    UARTprintf("\nInitializing I2C...\n");
    // Enable the peripherals for I2C1 and GPIOA
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Wait for the peripherals to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1) || !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    UARTprintf("Peripherals ready...\n");

    // Corrected pin configuration for I2C1
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);

    // Initialize and configure the I2C1 master module
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);  // false = standard mode (100kbps)
    UARTprintf("I2C Initialization Complete...\n");
}



void ConfigureCompass(void){

    // Set the slave address and specify that the next operation is a write
    I2CMasterSlaveAddrSet(I2C1_BASE, COMPASS_I2C_ADDRESS, false);

    // Write to Configuration Register A
    I2CMasterDataPut(I2C1_BASE, 0x00);  // Address of Configuration Register A
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C1_BASE));
    I2CMasterDataPut(I2C1_BASE, 0x70);  // Configuration data
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C1_BASE));

    // Write to Configuration Register B
    I2CMasterDataPut(I2C1_BASE, 0x01);  // Address of Configuration Register B
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C1_BASE));
    I2CMasterDataPut(I2C1_BASE, 0x20);  // Configuration data
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C1_BASE));

    // Write to Mode Register
    I2CMasterDataPut(I2C1_BASE, 0x02);  // Address of Mode Register
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C1_BASE));
    I2CMasterDataPut(I2C1_BASE, 0x00);  // Configuration data
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while(I2CMasterBusy(I2C1_BASE));
}



void ReadCompass(int16_t *x, int16_t *y, int16_t *z){
    uint8_t data[TOTAL_DATA_BYTES];
    int index = 0;

    // Set the slave address and specify that the next operation is a write
    I2CMasterSlaveAddrSet(I2C1_BASE, COMPASS_I2C_ADDRESS, false);

    // Place the register to be read into the data register
    I2CMasterDataPut(I2C1_BASE, COMPASS_REGISTER);

    // Perform a single send operation to the slave
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);

    // Wait for the send operation to complete
    while(I2CMasterBusy(I2C1_BASE));

    // Check for errors after writing to the register
    if(I2CMasterErr(I2C1_BASE) != I2C_MASTER_ERR_NONE){
        UARTprintf("I2C Error: %u\n", I2CMasterErr(I2C1_BASE));
        return;
    }

    // Set the slave address and specify that the next operation is a read
    I2CMasterSlaveAddrSet(I2C1_BASE, COMPASS_I2C_ADDRESS, true);

    // Perform a burst receive, starting with the first byte
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

    // Wait for the receive operation to complete
    while(I2CMasterBusy(I2C1_BASE));

    // Read the first byte
    data[index++] = I2CMasterDataGet(I2C1_BASE);

    // Read the middle bytes
    for(; index < TOTAL_DATA_BYTES - 1; ++index){
        // Continue the burst receive
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
        while(I2CMasterBusy(I2C1_BASE));
        data[index] = I2CMasterDataGet(I2C1_BASE);
    }

    // Finish the burst receive
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(I2CMasterBusy(I2C1_BASE));
    data[index] = I2CMasterDataGet(I2C1_BASE);

    // Check for errors after reading the data
    if(I2CMasterErr(I2C1_BASE) != I2C_MASTER_ERR_NONE){
        UARTprintf("I2C Error: %u\n", I2CMasterErr(I2C1_BASE));
        return;
    }

    // Assume the data is in the format X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
    *x = (data[0] << 8) | data[1];
    *z = (data[2] << 8) | data[3];
    *y = (data[4] << 8) | data[5];
}

int main(void){


    SetClocks();
    ConfigureUART();
    InitializeI2C();
    ConfigureCompass();
    InitializePins();
    MAP_FPULazyStackingEnable();

  UARTprintf("\nTesting the HMC5883L Compass response...\n");

  //IMPORTANT OFFSET WILL CALIBRATE THE COMPASS. LEADING TO BETTER ACCURACY
  const int x_offset = 326;
  const int y_offset = -189;

  while(1) {
      ProofOfLifeLEDsToggle();

      int16_t x, y, z;
      ReadCompass(&x, &y, &z);  // Read data from the compass

      //adjust the x and y values

      UARTprintf("Raw Compass Data: X: %d, Y: %d \n", x,y);

      x+= x_offset;
      y+= y_offset;

      // Calculate heading in degrees
      int heading = i_atan2(x, y);

      // Normalize the heading to 0-359 degrees
      if (heading < 0) {
          heading += 360;
      }

      PrintDirection(heading);

      // Print the heading to UART
      UARTprintf("Heading: %d degrees\n", heading);

      // Print the received data to UART
      UARTprintf("Compass Data: X: %d, Y: %d, Z: %d\n", x, y, z);


      SysCtlDelay(100 * one_milli_sec); // delay 109 ms

  }
} // fin
