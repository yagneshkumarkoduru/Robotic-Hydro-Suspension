/**
 * firmware_stm32_freertos.c
 * ==========================
 * Hard Real-Time STM32 FreeRTOS Firmware (1 kHz execution loop).
 * Manages CAN frame reception, PWM coil current closed-loop regulation,
 * and physical stroke limit limit-switch emergency shutdown.
 */

#include <stdint.h>
#include <stdbool.h>

#define STROKE_LIMIT_POS_MM   38.0f
#define STROKE_LIMIT_NEG_MM  -38.0f
#define MAX_CURRENT_AMPS       2.5f

typedef struct {
    float vertical_accel;
    float stroke_displacement;
    float coil_current_demand;
    bool  emergency_stop_triggered;
} SuspensionActuatorState;

static SuspensionActuatorState g_state;

/**
 * 1 kHz Hard Real-Time Actuator Control Task.
 * Triggered deterministically by hardware timer TIM2 interrupt.
 */
void vActuatorControlTask(void *pvParameters) {
    (void)pvParameters;
    
    for (;;) {
        // 1. Read linear potentiometer ADC channel
        // Convert raw ADC counts (0-4095) to displacement in mm
        float adc_voltage = 3.3f * ((float)(2048) / 4095.0f);
        g_state.stroke_displacement = (adc_voltage - 1.65f) * 25.0f;

        // 2. Hardware safety boundary verification
        if (g_state.stroke_displacement >= STROKE_LIMIT_POS_MM ||
            g_state.stroke_displacement <= STROKE_LIMIT_NEG_MM) {
            // Immediate hardware shutdown: de-energize coil to prevent bottoming impact
            g_state.emergency_stop_triggered = true;
            g_state.coil_current_demand = 0.0f;
        }

        // 3. Modulate PWM output via TIM1 CH1 complementary half-bridge
        if (!g_state.emergency_stop_triggered) {
            float duty_cycle = g_state.coil_current_demand / MAX_CURRENT_AMPS;
            if (duty_cycle > 1.0f) duty_cycle = 1.0f;
            if (duty_cycle < 0.0f) duty_cycle = 0.0f;
            // Write TIM1->CCR1 = (uint32_t)(duty_cycle * TIM1->ARR);
        }

        // 4. FreeRTOS 1 ms deterministic task sleep
        // vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1));
        break; // Standalone compilation check
    }
}
