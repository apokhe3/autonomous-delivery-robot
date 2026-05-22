// Ultrasonic Sensor Driver - TM4C123GH6PM
// TRIG = PB6, ECHO = PA4
// Measures distance using RCWL-1601 / HC-SR04 ultrasonic sensor
// Outputs distance over UART at ~10Hz

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"

#define TRIG_PORT       GPIO_PORTB_BASE
#define TRIG_PIN        GPIO_PIN_6      // PB6

#define ECHO_PORT       GPIO_PORTA_BASE
#define ECHO_PIN        GPIO_PIN_4      // PA4

static uint32_t g_sysClockHz;
static uint32_t g_delay1usTicks;

void UART0_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE,
                        g_sysClockHz,
                        9600,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE);
}

void UART0_SendString(const char *s)
{
    while (*s)
    {
        UARTCharPut(UART0_BASE, *s++);
    }
}

void delay_us(uint32_t us)
{
    SysCtlDelay(g_delay1usTicks * us);
}

void Ultrasonic_GPIO_Init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));

    // TRIG (PB6) as output
    GPIOPinTypeGPIOOutput(TRIG_PORT, TRIG_PIN);

    // ECHO (PA4) as input
    GPIOPinTypeGPIOInput(ECHO_PORT, ECHO_PIN);

    // Make sure TRIG starts LOW
    GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);
}

uint32_t Ultrasonic_MeasureEchoUS(void)
{
    uint32_t timeout = 30000;
    uint32_t count = 0;

    // Wait for ECHO to go HIGH
    while ((GPIOPinRead(ECHO_PORT, ECHO_PIN) == 0) && (count < timeout))
    {
        delay_us(1);
        count++;
    }
    if (count >= timeout) return 0;

    // Measure how long ECHO stays HIGH
    count = 0;
    while ((GPIOPinRead(ECHO_PORT, ECHO_PIN) != 0) && (count < timeout))
    {
        delay_us(1);
        count++;
    }

    return count; // pulse width in microseconds
}

float Ultrasonic_GetDistanceCM(void)
{
    uint32_t echoTime;

    // Send 10us trigger pulse on PB6
    GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);
    delay_us(2);
    GPIOPinWrite(TRIG_PORT, TRIG_PIN, TRIG_PIN);
    delay_us(10);
    GPIOPinWrite(TRIG_PORT, TRIG_PIN, 0);

    echoTime = Ultrasonic_MeasureEchoUS();
    if (echoTime == 0) return -1.0f;

    // distance_cm = time_us / 58
    float distance_cm = echoTime / 58.0f;
    return distance_cm;
}

int main(void)
{
    char buffer[64];

    SysCtlClockSet(SYSCTL_SYSDIV_1      |
                   SYSCTL_USE_OSC       |
                   SYSCTL_OSC_MAIN      |
                   SYSCTL_XTAL_16MHZ);

    g_sysClockHz = SysCtlClockGet();
    g_delay1usTicks = g_sysClockHz / 3 / 1000000;

    UART0_Init();
    Ultrasonic_GPIO_Init();

    UART0_SendString("\r\nUltrasonic RCWL-1601 (TRIG=PB6, ECHO=PA4)\r\n");

    while (1)
    {
        float d = Ultrasonic_GetDistanceCM();

        if (d < 0.0f)
            snprintf(buffer, sizeof(buffer), "No echo / out of range\r\n");
        else
            snprintf(buffer, sizeof(buffer), "Distance: %.2f cm\r\n", d);

        UART0_SendString(buffer);

        // ~100ms between measurements
        SysCtlDelay(g_sysClockHz / 3 / 10);
    }
}
