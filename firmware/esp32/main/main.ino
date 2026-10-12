/*
  Firmware principal del ESP32.

  Recibe comandos de la Jetson por el puerto serial, mueve los motores y la
  dirección, mide la distancia con el ultrasónico y reporta su estado.
  El protocolo y las reglas puras están en protocol.h (ADR-0001).

  Seguridad: si pasan WATCHDOG_MS sin un mensaje válido, corta los motores y
  centra la dirección, aunque la Jetson haya dejado de funcionar.

  Soporte ESP32 versión 3.x: ledcAttach(pin, freq, bits) y ledcWrite(pin, duty).
*/
#include "protocol.h"

// --- Pines ---------------------------------------------------------------
// Motor y puente H (L298N): los mismos de tests_componentes/test_motor.
const int PIN_IN1 = 14;
const int PIN_IN2 = 12;
const int PIN_ENA = 13;
// Ultrasónico (HC-SR04): los mismos de tests_componentes/test_ultrasonico.
const int PIN_TRIG = 26;
const int PIN_ECHO = 27;
// Servo de dirección. [asumido] Ningún sketch de prueba lo define: confirmar el pin.
const int PIN_SERVO = 25;

// --- Configuración -------------------------------------------------------
const char* VERSION_FW = "0.1.0";
const uint32_t BAUDIOS = 115200;            // [asumido] ADR-0001
const uint32_t WATCHDOG_MS = 300;           // [asumido] ADR-0001
const uint32_t PERIODO_ULTRA_MS = 50;       // 20 Hz
const uint32_t PERIODO_STATUS_MS = 500;     // 2 Hz
const int32_t V_MAX_MM_S = 500;             // [asumido] ADR-0001
const int32_t S_MAX_DECIMAS = 250;          // [asumido] ADR-0001: +-25,0 grados

const int PWM_MOTOR_FREQ = 5000;
const int PWM_MOTOR_BITS = 8;               // 0-255

const int SERVO_FREQ = 50;
const int SERVO_BITS = 14;
const int32_t SERVO_CENTRO_US = 1500;       // [asumido] calibrar en el auto
const int32_t SERVO_AMPLITUD_US = 400;      // [asumido] negativo si el servo está invertido
const uint32_t SERVO_PERIODO_US = 20000;

const uint32_t ULTRA_TIMEOUT_US = 25000;    // ~4 m
const size_t LARGO_MAX_LINEA = 128;

// --- Estado --------------------------------------------------------------
aura::Watchdog watchdog(WATCHDOG_MS);
bool watchdog_cortado = false;
uint32_t ultima_seq = 0;

char linea[LARGO_MAX_LINEA];
size_t largo_linea = 0;
bool descartando_linea = false;

uint32_t t_ultra = 0;
uint32_t t_status = 0;

// --- Actuadores ----------------------------------------------------------
void escribir_servo_us(int32_t pulso_us) {
  const uint32_t duty_max = (1UL << SERVO_BITS) - 1;
  ledcWrite(PIN_SERVO, static_cast<uint32_t>(pulso_us) * duty_max / SERVO_PERIODO_US);
}

void detener() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_ENA, 0);
  escribir_servo_us(SERVO_CENTRO_US);
}

void aplicar_drive(int32_t v_mm_s, int32_t s_decimas) {
  const uint8_t pwm = aura::velocidad_a_pwm(v_mm_s, V_MAX_MM_S);
  if (v_mm_s > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (v_mm_s < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }
  ledcWrite(PIN_ENA, pwm);
  escribir_servo_us(aura::direccion_a_pulso_us(s_decimas, S_MAX_DECIMAS, SERVO_CENTRO_US,
                                               SERVO_AMPLITUD_US));
}

void frenar() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_ENA, 0);
}

// --- Reportes a la Jetson ------------------------------------------------
void enviar_status() {
  char buf[96];
  const int n = aura::formatear_status(buf, sizeof(buf), VERSION_FW, watchdog_cortado, ultima_seq);
  if (n > 0) Serial.write(reinterpret_cast<const uint8_t*>(buf), n);
}

void enviar_ultra() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  const unsigned long duracion = pulseIn(PIN_ECHO, HIGH, ULTRA_TIMEOUT_US);
  const bool valida = duracion != 0;
  const int32_t dist_cm = valida ? static_cast<int32_t>(duracion * 0.0343f / 2.0f + 0.5f) : 0;

  char buf[96];
  const int n = aura::formatear_ultra(buf, sizeof(buf), dist_cm, valida);
  if (n > 0) Serial.write(reinterpret_cast<const uint8_t*>(buf), n);
}

// --- Entrada serial ------------------------------------------------------
void procesar_linea(const char* texto) {
  aura::Mensaje m;
  if (!aura::parsear_linea(texto, &m)) return;  // inválido: se descarta sin alimentar el watchdog

  watchdog.alimentar(millis());
  watchdog_cortado = false;
  ultima_seq = m.seq;

  if (m.tipo == aura::TipoMensaje::Drive) {
    aplicar_drive(m.v_mm_s, m.s_decimas);
  } else if (m.tipo == aura::TipoMensaje::Brake) {
    frenar();
  }
}

void leer_serial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\n') {
      if (!descartando_linea) {
        linea[largo_linea] = '\0';
        procesar_linea(linea);
      }
      largo_linea = 0;
      descartando_linea = false;
    } else if (largo_linea < LARGO_MAX_LINEA - 1) {
      linea[largo_linea++] = c;
    } else {
      descartando_linea = true;  // línea demasiado larga: se ignora hasta el próximo '\n'
    }
  }
}

// --- Arduino -------------------------------------------------------------
void setup() {
  Serial.begin(BAUDIOS);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  ledcAttach(PIN_ENA, PWM_MOTOR_FREQ, PWM_MOTOR_BITS);
  ledcAttach(PIN_SERVO, SERVO_FREQ, SERVO_BITS);
  detener();  // el auto arranca detenido y con la dirección centrada

  enviar_status();
}

void loop() {
  leer_serial();

  const uint32_t ahora = millis();

  if (watchdog.vencido(ahora) && !watchdog_cortado) {
    detener();
    watchdog_cortado = true;
    enviar_status();
  }

  if (ahora - t_ultra >= PERIODO_ULTRA_MS) {
    t_ultra = ahora;
    enviar_ultra();
  }
  if (ahora - t_status >= PERIODO_STATUS_MS) {
    t_status = ahora;
    enviar_status();
  }
}
