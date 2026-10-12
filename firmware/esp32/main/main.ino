/*
  Firmware principal del ESP32.
  Por ahora es un placeholder minimo: solo confirma que compila y sube
  correctamente. La logica real (protocolo serie con la Jetson, lectura
  de sensores, control de motores) se implementa en tareas posteriores.
*/
 
void setup() {
  Serial.begin(115200);
  Serial.println("main.ino iniciado correctamente (placeholder)");
}
 
void loop() {
  delay(1000);
}
