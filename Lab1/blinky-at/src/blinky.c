#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

//***************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//***************************************************
#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line) {
  while (1)
    ;
}
#endif

// Initialise Linked List Variables
volatile struct led *head;
volatile struct led *currLED = NULL;
// Create LED Struct
struct led {
  struct led *nextLED;
  struct led *prevLED;
  uint32_t BASE;
  uint32_t PIN;
};

// Add LED to linked Loop List
int addLED(struct led *child) {
  GPIOPinTypeGPIOOutput(child->BASE, child->PIN);
  if (currLED == NULL) {
    currLED = child; // Sets first element added as the head
  }
  child->nextLED = currLED->nextLED; // Sets the new elements' Next LED to current led's next led (should be the head)
  child->prevLED = currLED; // set the current led to the new elements previous led
  head->prevLED = child; // set the heads previous led to the new element (as this is the end of the list)
  currLED->nextLED = child; // set current led next led to the new elemnt beening added
  currLED = child; // set the new element as the current led
  return 1; // return successed
}

volatile bool toggle = true;
// Interupt Handler
void PortJIntHander(void) {
  uint32_t intStatus = GPIOIntStatus(GPIO_PORTJ_BASE, true); // Get the GPIO pin that caused the interupt
  GPIOIntClear(GPIO_PORTJ_BASE, intStatus); // Clears the Int Status register, for the next time

  if (intStatus & GPIO_INT_PIN_0) {
    toggle = false;
  }
  if (intStatus & GPIO_INT_PIN_1) {
    toggle = true;
  }
}

int main(void) {
  volatile uint32_t ui32Loop;
  //
  // Enable the GPIO port that is used for the on-board LED.
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
  //
  // Check if the peripheral access is enabled.
  //
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION) &&
         !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF) &&
         !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOJ)) {
  }

  // Initialising all the nodes for the looped linked list
  struct led ledD1;
  head = &ledD1;
  ledD1.BASE = GPIO_PORTN_BASE;
  ledD1.PIN = GPIO_PIN_1;
  struct led ledD2;
  ledD2.BASE = GPIO_PORTN_BASE;
  ledD2.PIN = GPIO_PIN_0;
  struct led ledD3;
  ledD3.BASE = GPIO_PORTF_BASE;
  ledD3.PIN = GPIO_PIN_4;
  struct led ledD4;
  ledD4.BASE = GPIO_PORTF_BASE;
  ledD4.PIN = GPIO_PIN_0;
  // Join them together (This also set the pins to output)
  addLED(&ledD1);
  addLED(&ledD2);
  addLED(&ledD3);
  addLED(&ledD4);

  // Set GPIO to a Input
  GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
  // Enables Pull up resistor on GPIO Pins
  GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA,
                   GPIO_PIN_TYPE_STD_WPU);
  // Set Two GPIO Switchs to Interupt Pins
  GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_FALLING_EDGE);
  // Register the handler function when the interupt calls
  GPIOIntRegister(GPIO_PORTJ_BASE, PortJIntHander);
  // Now all setup, Enable Interupts for these Pins
  GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  while (1) {
    // Sets the Current Led to on
    GPIOPinWrite(currLED->BASE, currLED->PIN, currLED->PIN);
    for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {
    } // Delay
    // Sets the Current Led to off
    GPIOPinWrite(currLED->BASE, currLED->PIN, 0x0);
    for (ui32Loop = 0; ui32Loop < 200000; ui32Loop++) {
    } // Delay
    // Depending on the direction, set the new Current led
    if (toggle) {
      currLED = currLED->nextLED;
    } else {
      currLED = currLED->prevLED;
    }
  }
}
