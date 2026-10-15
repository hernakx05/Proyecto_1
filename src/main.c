#include <Arduino.h>

// ============================================================================
// CONFIGURACIÓN DE HARDWARE Y PINES (IdeaBoard L9110S)
// ============================================================================
constexpr uint8_t PIN_MOTOR_IN1 = 12; // Entrada IN1 del Puente H L9110S
constexpr uint8_t PIN_MOTOR_IN2 = 14; // Entrada IN2 del Puente H L9110S

// Configuración de LEDC (Hardware PWM)
constexpr uint8_t LEDC_CHANNEL_IN1 = 0;
constexpr uint8_t LEDC_CHANNEL_IN2 = 1;
constexpr uint32_t LEDC_FREQ_HZ    = 1000;      // 1 kHz
constexpr uint8_t LEDC_RESOLUTION_BITS = 10;   // 10 bits (0 - 1023)
constexpr uint32_t MOTOR_SPEED_DUTY = 768;     // 75% duty cycle

// ============================================================================
// CAPA DE ABSTRACCIÓN DE HARDWARE (HAL) - CONTROL DE MOTOR
// ============================================================================
void HAL_Motor_Init() {
    ledcSetup(LEDC_CHANNEL_IN1, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
    ledcSetup(LEDC_CHANNEL_IN2, LEDC_FREQ_HZ, LEDC_RESOLUTION_BITS);
    
    ledcAttachPin(PIN_MOTOR_IN1, LEDC_CHANNEL_IN1);
    ledcAttachPin(PIN_MOTOR_IN2, LEDC_CHANNEL_IN2);
    
    // Estado inicial: Detenido por inercia
    ledcWrite(LEDC_CHANNEL_IN1, 0);
    ledcWrite(LEDC_CHANNEL_IN2, 0);
}

void HAL_Motor_Forward(uint32_t duty) {
    ledcWrite(LEDC_CHANNEL_IN1, duty);
    ledcWrite(LEDC_CHANNEL_IN2, 0);
}

void HAL_Motor_Brake() {
    // Frenado dinámico: Ambos pines a HIGH (1023)
    uint32_t max_duty = (1 << LEDC_RESOLUTION_BITS) - 1;
    ledcWrite(LEDC_CHANNEL_IN1, max_duty);
    ledcWrite(LEDC_CHANNEL_IN2, max_duty);
}

void HAL_Motor_Coast() {
    // Parada por inercia: Ambos pines a LOW (0)
    ledcWrite(LEDC_CHANNEL_IN1, 0);
    ledcWrite(LEDC_CHANNEL_IN2, 0);
}

void setup() {
    Serial.begin(115200);
    HAL_Motor_Init();
    Serial.println("[HAL] Motor HAL Inicializada Correctamente.");
}

void loop() {
    // Prueba de funcionamiento
    Serial.println("[TEST] Marcha adelante (75%)...");
    HAL_Motor_Forward(MOTOR_SPEED_DUTY);
    delay(2000);

    Serial.println("[TEST] Frenado dinamico...");
    HAL_Motor_Brake();
    delay(1000);

    Serial.println("[TEST] Parada por inercia...");
    HAL_Motor_Coast();
    delay(2000);
}
