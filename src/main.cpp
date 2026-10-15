#include <Arduino.h>
#include <esp_attr.h>

// ============================================================================
// LECTURA DIRECTA DE CCOUNT (Ensamblador Xtensa LX6)
// ============================================================================
static inline uint32_t get_ccount(void) {
    uint32_t ccount;
    __asm__ __volatile__("rsr %0, ccount" : "=r"(ccount));
    return ccount;
}

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
// DEFINICIÓN DE ESTADOS Y EVENTOS (FSM)
// ============================================================================
enum class State : uint8_t {
    IDLE = 0,
    RUNNING,
    FAULT,
    NUM_STATES
};

enum class Event : uint8_t {
    NONE = 0,
    BTN_PRESS,
    RESET,
    NUM_EVENTS
};

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
    uint32_t current_ccount = get_ccount();
    
    if ((current_ccount - g_last_ccount) >= DEBOUNCE_CYCLES) {
        g_last_ccount = current_ccount;
        
        portENTER_CRITICAL_ISR(&g_spinlock);
        g_btn_event_flag = true;
        g_press_count++;
        portEXIT_CRITICAL_ISR(&g_spinlock);
    }
}

// ============================================================================
// HANDLERS Y TRANSICIONES DE LA FSM O(1)
// ============================================================================
typedef void (*StateHandlerFunc)();

void Handle_Idle_None() {
    HAL_Motor_Coast();
}

void Handle_Idle_To_Running() {
    Serial.println("[FSM] -> Transicion: IDLE a RUNNING. Arrancando Banda Transportadora...");
    HAL_Motor_Forward(MOTOR_SPEED_DUTY);
}

void Handle_Running_None() {
    HAL_Motor_Forward(MOTOR_SPEED_DUTY);
}

void Handle_Running_To_Fault() {
    Serial.println("[FSM] -> ALERTA DE EMERGENCIA: Transicion a FAULT. Aplicando Frenado Dinamico!");
    HAL_Motor_Brake();
}

void Handle_Fault_None() {
    HAL_Motor_Brake();
}

void Handle_Fault_To_Idle() {
    Serial.println("[FSM] -> RESET: Sistema restaurado de FAULT a IDLE.");
    HAL_Motor_Coast();
}

void Handle_Ignore() {
    // Evento no válido para el estado actual
}

// Matriz de Handlers O(1)
constexpr StateHandlerFunc FSM[static_cast<uint8_t>(State::NUM_STATES)][static_cast<uint8_t>(Event::NUM_EVENTS)] = {
    // Event:  NONE                BTN_PRESS                RESET
    /* IDLE    */ { Handle_Idle_None,     Handle_Idle_To_Running,  Handle_Ignore },
    /* RUNNING */ { Handle_Running_None,  Handle_Running_To_Fault, Handle_Ignore },
    /* FAULT   */ { Handle_Fault_None,    Handle_Ignore,           Handle_Fault_To_Idle }
};

// Matriz de Próximo Estado O(1)
constexpr State StateTable[static_cast<uint8_t>(State::NUM_STATES)][static_cast<uint8_t>(Event::NUM_EVENTS)] = {
    // Event:  NONE           BTN_PRESS      RESET
    /* IDLE    */ { State::IDLE,    State::RUNNING, State::IDLE },
    /* RUNNING */ { State::RUNNING, State::FAULT,   State::RUNNING },
    /* FAULT   */ { State::FAULT,   State::FAULT,   State::IDLE }
};

// ============================================================================
// CONFIGURACIÓN INICIAL Y BUCLE PRINCIPAL
// ============================================================================
static State g_current_state = State::IDLE;

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    Serial.println("\n=================================================");
    Serial.println("  INICIALIZANDO CONTROLADOR BANDA TRANSPORTADORA ");
    Serial.println("=================================================");

    HAL_Motor_Init();
    pinMode(PIN_EMERGENCY, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(PIN_EMERGENCY), ISR_Emergency_Button, FALLING);

    Serial.println("[INIT] Sistema listo. Estado Inicial: IDLE");
}

void loop() {
    Event current_event = Event::NONE;

    // Lectura atómica de eventos desde la ISR
    portENTER_CRITICAL(&g_spinlock);
    if (g_btn_event_flag) {
        g_btn_event_flag = false;
        current_event = Event::BTN_PRESS;
    }
    portEXIT_CRITICAL(&g_spinlock);

    // Despacho O(1) indexando la matriz directamente
    uint8_t state_idx = static_cast<uint8_t>(g_current_state);
    uint8_t event_idx = static_cast<uint8_t>(current_event);

    FSM[state_idx][event_idx]();
    g_current_state = StateTable[state_idx][event_idx];

    vTaskDelay(pdMS_TO_TICKS(10));
}