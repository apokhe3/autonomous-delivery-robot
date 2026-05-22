

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_qei.h"
#include "inc/hw_pwm.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "utils/uartstdio.h"

#define SYSCLK_HZ           80000000UL
#define LOOP_HZ             1000
#define PRINT_HZ            10

#define WHEEL_RADIUS_M      0.127f
#define TRACK_WIDTH_M       0.616f
#define CPR_OUTPUT          1425.1f
#define MM_PER_COUNT        ((2.0f * 3.1415926535f * (WHEEL_RADIUS_M * 1000.0f)) / (CPR_OUTPUT))

#define PWM_FREQ_HZ         20000
#define PWM_DIV             SYSCTL_PWMDIV_16
#define PWM_PERIOD_TICKS    (5000000UL / PWM_FREQ_HZ)

#define VEL_LPF_ALPHA       0.25f
#define KP_L                0.00085f
#define KI_L                0.0045f
#define KP_R                0.00025f
#define KI_R                0.0080f

// ==== IMPORTANT: cap max duty to ~70%  ====
#define DUTY_MIN            0.25f
#define DUTY_MAX            0.90f

// ===== Skip-PI-at-zero command (simple + quiet) =====
#define ZERO_CMD_EPS_MMPS   1.0f   // treat |target| <= 1 mm/s as zero (skip PI)

// Encoders: LEFT -> QEI0 PD6/PD7, RIGHT -> QEI1 PC5/PC6
// Driver pins: LEFT PF2=RPWM_L (M1PWM6), PF3=LPWM_L (M1PWM7)
//              RIGHT PD0=RPWM_R (M1PWM0), PD1=LPWM_R (M1PWM1)

// From your verified mapping: forward = Left uses LPWM, Right uses RPWM
#define LEFT_MOTOR_POL      (-1)
#define RIGHT_MOTOR_POL     (-1)

// Encoders forward-positive
#define LEFT_ENCODER_POL    (+1)
#define RIGHT_ENCODER_POL   (+1)

typedef struct {
    float target_mmps;
    volatile int32_t pos;
    volatile int32_t pos_prev;
    volatile int32_t dcounts;
    float cps;
    float mmps;
    float mmps_f;
    float integ;
    float duty_cmd;
    float err;
    float u_raw;
    float duty_applied;
} wheel_t;

static volatile wheel_t L = {0}, R = {0};
static volatile uint32_t g_tick = 0;
static uint32_t g_pwmPeriod = PWM_PERIOD_TICKS;

// ===== Mission parameters =====
#define FWD_DIST_M      10.0f
#define V_CMD_MMPS      900.0f
#define WAIT_SECONDS    40

typedef enum { M_INIT=0, M_FWD, M_BACK, M_WAIT } mission_state_t;
static volatile mission_state_t mission = M_INIT;
static volatile float seg_dist_m = 0.0f;
static volatile uint32_t wait_ticks_left = 0;

// ===== UART =====
static void UART0_Init_115200(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_SYSTEM);
    UARTStdioConfig(0, 115200, SysCtlClockGet());
}
static void print_fixed4(float v){
    int32_t whole = (int32_t)v;
    float fracf = v - (float)whole;
    if (fracf < 0) fracf = -fracf;
    int32_t frac = (int32_t)(fracf*10000.0f + 0.5f);
    UARTprintf("%d.%04d", whole, frac);
}

// ===== QEI =====
static void QEI_Init(void){
    // LEFT: QEI0 on PD6/PD7
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI0));

    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR)   |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    QEIDisable(QEI0_BASE);
    QEIConfigure(QEI0_BASE,
        QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE |
        QEI_CONFIG_NO_RESET | QEI_CONFIG_SWAP,   // left swapped so forward is +ve
        0xFFFFFFFF);
    QEIPositionSet(QEI0_BASE, 0);
    QEIEnable(QEI0_BASE);

    // RIGHT: QEI1 on PC5/PC6
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_QEI1));

    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6,
                     GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    QEIDisable(QEI1_BASE);
    QEIConfigure(QEI1_BASE,
        QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE |
        QEI_CONFIG_NO_RESET | QEI_CONFIG_NO_SWAP,
        0xFFFFFFFF);
    QEIPositionSet(QEI1_BASE, 0);
    QEIEnable(QEI1_BASE);

    UARTprintf("[QEI0_CTL]=0x%08x  [QEI1_CTL]=0x%08x\r\n",
               HWREG(QEI0_BASE + QEI_O_CTL), HWREG(QEI1_BASE + QEI_O_CTL));
}

// ===== PWM =====
static inline uint32_t duty_to_ticks(float duty_abs){
    if (duty_abs < 0.0f) duty_abs = 0.0f;
    if (duty_abs > 1.0f) duty_abs = 1.0f;
    return (uint32_t)(duty_abs * (float)g_pwmPeriod + 0.5f);
}
static void PWM_Init(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM1));

    SysCtlPWMClockSet(PWM_DIV); // 5 MHz

    // LEFT (PF2/PF3) -> Gen3
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, g_pwmPeriod);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 0);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 0);
    PWMOutputInvert(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, false);
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    // RIGHT (PD0/PD1) -> Gen0
    GPIOPinConfigure(GPIO_PD0_M1PWM0);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, g_pwmPeriod);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 0);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 0);
    PWMOutputInvert(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, false);
    PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);
    PWMGenEnable(PWM1_BASE, PWM_GEN_0);
}

// ===== Quiet stop helpers (coast) =====
static inline void MotorCoast_Left(void){
    // both channels ~0% → coast, quiet
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 1);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 1);
}
static inline void MotorCoast_Right(void){
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1);
}

// ===== Motor write (signed duty → RPWM/LPWM) =====
static void MotorWrite_Left(float duty){
    L.duty_cmd = duty;
    duty *= LEFT_MOTOR_POL;

    float d = duty;
    if (d >  1.0f) d =  1.0f;
    if (d < -1.0f) d = -1.0f;

    // deadband for overcoming friction
    if (fabsf(d) > 0.0f && fabsf(d) < DUTY_MIN) d = (d > 0 ? DUTY_MIN : -DUTY_MIN);
    if (d >  DUTY_MAX)  d =  DUTY_MAX;
    if (d < -DUTY_MAX) d = -DUTY_MAX;

    if (d < 0.0f) { // Forward = RPWM HIGH (PF2), LPWM LOW (PF3)
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, duty_to_ticks(-d)); // RPWM (PF2) gets PWM
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, 1);                 // LPWM (PF3) ~0%
    } else if (d > 0.0f) { // Reverse = LPWM HIGH (PF3), RPWM LOW (PF2)
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, 1);                 // RPWM (PF2) ~0%
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, duty_to_ticks(d));  // LPWM (PF3) gets PWM
    } else { // Stop/Coast (quiet)
        MotorCoast_Left();
    }
    L.duty_applied = d;
}
static void MotorWrite_Right(float duty){
    R.duty_cmd = duty;
    duty *= RIGHT_MOTOR_POL;

    float d = duty;
    if (d >  1.0f) d =  1.0f;
    if (d < -1.0f) d = -1.0f;
    if (fabsf(d) > 0.0f && fabsf(d) < DUTY_MIN) d = (d > 0 ? DUTY_MIN : -DUTY_MIN);
    if (d >  DUTY_MAX)  d =  DUTY_MAX;
    if (d < -DUTY_MAX) d = -DUTY_MAX;

    // d < 0.0f is FORWARD for the right motor (based on mapping)
    if (d < 0.0f) { // Forward = LPWM HIGH (PD1), RPWM LOW (PD0)
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);                 // RPWM (PD0) ~0%
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, duty_to_ticks(-d)); // LPWM (PD1) PWM
    } else if (d > 0.0f) { // Reverse = RPWM HIGH (PD0), LPWM LOW (PD1)
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, duty_to_ticks(d));  // RPWM (PD0) PWM
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1);                 // LPWM (PD1) ~0%
    } else { // Stop/Coast (quiet)
        MotorCoast_Right();
    }
    R.duty_applied = d;
}

// ===== Command: (v,w) → per-wheel targets =====
static void set_cmd(float v_mmps, float w_rads){
    float half_track_mm = (TRACK_WIDTH_M * 1000.0f) * 0.5f;
    float vL = v_mmps - (w_rads * half_track_mm);
    float vR = v_mmps + (w_rads * half_track_mm);
    L.target_mmps = vL;
    R.target_mmps = vR;
}

// ===== Control core =====
static void vel_est_and_pi_step(wheel_t* W, float KP, float KI){
    int32_t dcounts = W->pos - W->pos_prev;
    W->pos_prev = W->pos;

    if (W == &L) dcounts *= LEFT_ENCODER_POL; else dcounts *= RIGHT_ENCODER_POL;

    W->dcounts = dcounts;
    W->cps     = (float)dcounts * (float)LOOP_HZ;
    W->mmps    = W->cps * MM_PER_COUNT;
    W->mmps_f  = (1.0f - VEL_LPF_ALPHA)*W->mmps_f + VEL_LPF_ALPHA*W->mmps;

    // --- SKIP PI ENTIRELY when command is (near) zero; force quiet coast ---
    if (fabsf(W->target_mmps) <= ZERO_CMD_EPS_MMPS){
        W->err   = 0.0f;
        W->u_raw = 0.0f;
        W->integ = 0.0f;
        if (W == &L) MotorCoast_Left(); else MotorCoast_Right();
        return;
    }

    // PI control
    float err = W->target_mmps - W->mmps_f;
    W->integ += err * (1.0f / (float)LOOP_HZ);
    if (W->integ > 1000.0f)  W->integ = 1000.0f;
    if (W->integ < -1000.0f) W->integ = -1000.0f;

    float u = KP * err + KI * W->integ;

    // store for telemetry
    W->err  = err;
    W->u_raw= u;

    // drive hardware
    if (W == &L) MotorWrite_Left(u); else MotorWrite_Right(u);
}

void SysTickIntHandler(void){
    g_tick++;

    // Read QEI
    L.pos = QEIPositionGet(QEI0_BASE);
    R.pos = QEIPositionGet(QEI1_BASE);

    // PI update
    vel_est_and_pi_step((wheel_t*)&L, KP_L, KI_L);
    vel_est_and_pi_step((wheel_t*)&R, KP_R, KI_R);

    // ---- distance this tick (meters) from encoders
    float dL_m = ((float)L.dcounts * MM_PER_COUNT) / 1000.0f;
    float dR_m = ((float)R.dcounts * MM_PER_COUNT) / 1000.0f;
    float ds   = 0.5f * (dL_m + dR_m);

    seg_dist_m += fabsf(ds);

    // ---- simple mission
    switch (mission){
        case M_INIT:
            seg_dist_m = 0.0f;
            wait_ticks_left = 0;
            set_cmd(0.0f, 0.0f);
            mission = M_FWD;
            break;

        case M_FWD:
            set_cmd(V_CMD_MMPS, 0.0f);
            if (seg_dist_m >= FWD_DIST_M){
                seg_dist_m = 0.0f;
                mission = M_BACK;
            }
            break;

        case M_BACK:
            set_cmd(-V_CMD_MMPS, 0.0f);
            if (seg_dist_m >= FWD_DIST_M){
                set_cmd(0.0f, 0.0f);                 // triggers skip-PI → quiet coast
                wait_ticks_left = WAIT_SECONDS * LOOP_HZ;
                mission = M_WAIT;
            }
            break;

        case M_WAIT:
            // PI is skipped automatically because target is zero
            if (wait_ticks_left > 0) {
                wait_ticks_left--;
            } else {
                seg_dist_m = 0.0f;
                mission = M_FWD;   // repeat
            }
            break;
    }

    // Telemetry (updated to print err/u)
    if ((g_tick % (LOOP_HZ / PRINT_HZ)) == 0){
        UARTprintf("L t=%d v=", (int)L.target_mmps); print_fixed4(L.mmps_f);
        UARTprintf(" e="); print_fixed4(L.err);
        UARTprintf(" I="); print_fixed4(L.integ);
        UARTprintf(" u="); print_fixed4(L.u_raw);
        UARTprintf(" d="); print_fixed4(L.duty_applied);

        UARTprintf(" | R t=%d v=", (int)R.target_mmps); print_fixed4(R.mmps_f);
        UARTprintf(" e="); print_fixed4(R.err);
        UARTprintf(" I="); print_fixed4(R.integ);
        UARTprintf(" u="); print_fixed4(R.u_raw);
        UARTprintf(" d="); print_fixed4(R.duty_applied);

        UARTprintf(" | s=%d d=", (int)mission); print_fixed4(seg_dist_m);
        UARTprintf("\r\n");
    }
}

// ===== main =====
int main(void){
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL |
                   SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    UART0_Init_115200();
    UARTprintf("\r\nDual Motor Velocity PI | CPR=%.1f | PWM=%lukHz | DUTY_MAX=%.2f\r\n",
               CPR_OUTPUT, (unsigned long)(PWM_FREQ_HZ/1000), (double)DUTY_MAX);

    QEI_Init();
    PWM_Init();

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    SysCtlDelay(SysCtlClockGet()/6);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
    SysCtlDelay(SysCtlClockGet()/6);

    SysTickIntRegister(SysTickIntHandler);
    SysTickPeriodSet(SysCtlClockGet() / LOOP_HZ);
    SysTickIntEnable();
    IntMasterEnable();
    SysTickEnable();

    // Mission starts in SysTick (M_INIT→M_FWD)
    while(1){ }
}
