/*
  TEST AISLADO: Sensor ultrasónico (ej. HC-SR04)
  Objetivo: confirmar que el sensor mide distancias correctas
  ANTES de integrarlo a la lógica de decisión del vehículo.
 
  Conexión típica:
    TRIG -> pin digital (ej. GPIO 5)
    ECHO -> pin digital (ej. GPIO 18)
    VCC  -> 5V
    GND  -> GND
*/
 
const int PIN_TRIG = 26;
const int PIN_ECHO = 27;
 
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  delay(500);
  Serial.println("Test ultrasonico iniciado. Distancia en cm:");
}
 
void loop() {
  // Disparar el pulso
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
 
  // Medir tiempo de eco (timeout de 30ms ~ 5m aprox)
  long duracion = pulseIn(PIN_ECHO, HIGH, 30000);
 
  if (duracion == 0) {
    Serial.println("Sin eco (fuera de rango o sensor desconectado)");
  } else {
    float distancia_cm = duracion * 0.0343 / 2.0;
    Serial.print("Distancia: ");
    Serial.print(distancia_cm);
    Serial.println(" cm");
  }
 
  delay(300); // frecuencia de medicion, ajustable
}
 
/*
  CHECKLIST DE VALIDACION MANUAL:
  [ ] Con la mano a 10 cm, ¿marca ~10 cm (+/- 1-2 cm)?
  [ ] Con la mano a 50 cm, ¿marca ~50 cm?
  [ ] Sin obstaculo delante, ¿da timeout o un valor absurdo (no un valor falso bajo)?
  [ ] ¿Las lecturas son estables (no saltan erraticamente entre valores muy distintos)?
  [ ] Probar con una superficie angulada (no perpendicular al sensor):
      anotar si la lectura falla o se vuelve inestable.
*/
