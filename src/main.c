#include <Arduino.h>
#include <esp_attr.h>
#include <xtensa/hal.h>

// ============================================================================
// CONFIGURACIÓN DE HARDWARE Y PINES (IdeaBoard L9110S & GPIOs)
// ============================================================================
constexpr uint8_t PIN_MOTOR_IN1 = 12; // Entrada IN1 del Puente H L9110S
constexpr uint8_t PIN_MOTOR_IN2 = 14; // Entrada IN2 del Puente H L9110S
constexpr uint8_t PIN_EMERGENCY = 33; // Botón Paro de Emergencia

// Configuración de LEDC (Hardware PWM)
constexpr uint8_t LEDC_CHANNEL_IN1 = 0;
constexpr uint8_t LEDC_CHANNEL_IN2 = 1;
constexpr uint32_t LEDC_FREQ_HZ    = 1000;
constexpr uint8_t LEDC_RESOLUTION_BITS = 10; // 0 - 1023
constexpr uint32_t MOTOR_SPEED_DUTY = 768;   // 75% duty cycle

// Parámetros de Debouncing por CCOUNT (25 ms a 160 MHz = 4,000,000 ciclos)
constexpr uint32_t DEBOUNCE_CYCLES = 4000000;

// ============================================================================
// VARIABLES GLOBALES DE INTERRUPCIÓN Y CONCURRENCIA
// ============================================================================
static portMUX_TYPE g_spinlock = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t g_last_ccount = 0;
static volatile bool g_btn_event_flag  = false;
static volatile uint32_t g_press_count = 0;

// ============================================================================
// CAPA DE ABSTRACCIÓN DE HARDWARE (HAL) - CONTROL DE MOTOR
// ============================================================================
void HAL_Motor_Init() {
    ledcSetup(LEDC_CHANNEL_IN1, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
    ledcSetup(LEDC_CHANNEL_IN2, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
    
    ledcAttachPin(PIN_MOTOR_IN1, LEDC_CHANNEL_IN1);
    ledcAttachPin(PIN_MOTOR_IN2, LEDC_CHANNEL_IN2);
    
    ledcWrite(LEDC_CHANNEL_IN1, 0);
    ledcWrite(LEDC_CHANNEL_IN2, 0);
}

void HAL_Motor_Forward(uint32_t duty) {
    ledcWrite(LEDC_CHANNEL_IN1, duty);
    ledcWrite(LEDC_CHANNEL_IN2, 0);
}

void HAL_Motor_Brake() {
    uint32_t max_duty = (1 << LEDC_RESOLUTION_BITS) - 1;
    ledcWrite(LEDC_CHANNEL_IN1, max_duty);
    ledcWrite(LEDC_CHANNEL_IN2, max_duty);
}

void HAL_Motor_Coast() {
    ledcWrite(LEDC_CHANNEL_IN1, 0);
    ledcWrite(LEDC_CHANNEL_IN2, 0);
}

// ============================================================================
// RUTINA DE SERVICIO DE INTERRUPCIÓN (ISR) CON DEBOUNCING POR CCOUNT
// ============================================================================
void IRAM_ATTR ISR_Emergency_Button() {
    uint32_t current_ccount = XTHAL_GET_CCOUNT();
    
    // Verificación con aritmética modular de 32 bits
    if ((current_ccount - g_last_ccount) >= DEBOUNCE_CYCLES) {
        g_last_ccount = current_ccount;
        
        // Sección crítica dentro de la ISR mediante Spinlock SMP
        portENTER_CRITICAL_ISR(&g_spinlock);
        g_btn_event_flag = true;
        g_press_count++;
        portEXIT_CRITICAL_ISR(&g_spinlock);
    }
}

// ============================================================================
// CONFIGURACIÓN INICIAL Y BUCLE PRINCIPAL
// ============================================================================
void setup() {
    Serial.begin(115200);
    HAL_Motor_Init();

    // Configuración del GPIO con Resistencia Pull-Up interna
    pinMode(PIN_EMERGENCY, INPUT_PULLUP);

    // Adjuntar la interrupción por flanco de bajada (FALLING)
    attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY), ISR_Emergency_Button, FALLING);

    Serial.println("[INIT] Sistema de Interrupción e ISR Inicializado Correctamente.");
}

void loop() {
    bool local_btn_flag = false;
    uint32_t local_press_count = 0;

    // Lectura atómica de variables compartidas en el loop()
    portENTER_CRITICAL(&g_spinlock);
    if (g_btn_event_flag) {
        g_btn_event_flag = false;
        local_btn_flag = true;
        local_press_count = g_press_count;
    }
    portEXIT_CRITICAL(&g_spinlock);

    // Si se detectó una pulsación válida
    if (local_btn_flag) {
        Serial.printf("[ISR EVENT] Pulsacion filtrada detectada (#%u). Deteniendo motor...\n", local_press_count);
        HAL_Motor_Brake();
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}