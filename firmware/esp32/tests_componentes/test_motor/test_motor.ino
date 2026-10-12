/*
  TEST AISLADO: Motor + puente H (ej. L298N)
  Objetivo: confirmar sentido de giro y control de velocidad
  ANTES de integrarlo a la logica de control del vehiculo.
 
  Conexion (un motor, canal A del L298N):
    IN1 -> pin digital (GPIO 14)
    IN2 -> pin digital (GPIO 12)
    ENA -> pin PWM     (GPIO 13)
    Motor -> salidas OUT1/OUT2 del L298N
    Alimentacion de motores -> fuente separada de la logica (NO el 5V del ESP32)
*/
 
const int PIN_IN1 = 14;
const int PIN_IN2 = 12;
const int PIN_ENA = 13;
 
const int PWM_FREQ = 5000;
const int PWM_RES_BITS = 8; // 0-255
 
void setup() {
  Serial.begin(115200);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
 
  ledcAttach(PIN_ENA, PWM_FREQ, PWM_RES_BITS);
 
  Serial.println("Test motor iniciado.");
}
 
void avanzar(int velocidad) { // velocidad: 0-255
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_ENA, velocidad);
}
 
void retroceder(int velocidad) {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  ledcWrite(PIN_ENA, velocidad);
}
 
void frenar() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(PIN_ENA, 0);
}
 
void loop() {
  Serial.println("Avanzando lento (velocidad 100)");
  avanzar(100);
  delay(2000);
 
  Serial.println("Frenando");
  frenar();
  delay(1000);
 
  Serial.println("Avanzando rapido (velocidad 220)");
  avanzar(220);
  delay(2000);
 
  Serial.println("Frenando");
  frenar();
  delay(1000);
 
  Serial.println("Retrocediendo (velocidad 150)");
  retroceder(150);
  delay(2000);
 
  Serial.println("Frenando");
  frenar();
  delay(3000); // pausa larga antes de repetir el ciclo
}
 
/*
  CHECKLIST DE VALIDACION MANUAL:
  [ ] ¿Gira en el sentido esperado como "avanzar" (rueda hacia adelante)?
  [ ] ¿"retroceder" efectivamente invierte el sentido?
  [ ] ¿A velocidad baja (ej. 80) el motor arranca, o se queda sin moverse
      (motor "trabado" por debajo de cierto PWM minimo)? Anotar ese umbral.
  [ ] ¿El freno detiene realmente, o el motor sigue girando por inercia
      un tiempo notable?
  [ ] Con el vehiculo LEVANTADO del piso (ruedas al aire), confirmar
      todo esto ANTES de probarlo con el vehiculo apoyado.
*/
