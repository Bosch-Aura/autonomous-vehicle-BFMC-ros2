/*
  Protocolo serial Jetson <-> ESP32 (ADR-0001) y reglas puras del firmware.

  Este archivo NO depende de Arduino: se puede compilar y testear en la
  computadora (ver tests_host/). El sketch main.ino solo conecta estas
  funciones con los pines y el puerto serial.

  Formato: un objeto JSON por línea, terminado en '\n'.

  Jetson -> ESP32
    {"t":"drive","seq":12,"v":250,"s":-30}   v: mm/s, s: décimas de grado (+ = izquierda)
    {"t":"brake","seq":13}

  ESP32 -> Jetson
    {"t":"ultra","id":"front","dist_cm":87,"valid":true}
    {"t":"status","fw":"0.1.0","wd":false,"last_seq":12}
*/
#ifndef AURA_PROTOCOL_H
#define AURA_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace aura {

// ----------------------------------------------------------------------------
// Mensajes de la Jetson
// ----------------------------------------------------------------------------

enum class TipoMensaje : uint8_t { Invalido, Drive, Brake };

struct Mensaje {
  TipoMensaje tipo = TipoMensaje::Invalido;
  uint32_t seq = 0;
  int32_t v_mm_s = 0;      // solo en Drive
  int32_t s_decimas = 0;   // solo en Drive
};

namespace detalle {

inline const char* saltar_espacios(const char* p) {
  while (*p == ' ' || *p == '\t' || *p == '\r') ++p;
  return p;
}

// Busca "clave" y devuelve un puntero al valor que la sigue, o nullptr.
// La clave se busca entre comillas: "s" no coincide con "seq".
inline const char* buscar_valor(const char* linea, const char* clave) {
  char patron[16];
  const int n = snprintf(patron, sizeof(patron), "\"%s\"", clave);
  if (n <= 0 || n >= static_cast<int>(sizeof(patron))) return nullptr;

  const char* p = strstr(linea, patron);
  if (p == nullptr) return nullptr;
  p = saltar_espacios(p + n);
  if (*p != ':') return nullptr;
  return saltar_espacios(p + 1);
}

// Lee un entero decimal (con signo opcional). Rechaza vacío, más de 9 dígitos
// y cualquier carácter que no continúe con ',' '}' o espacio.
inline bool leer_entero(const char* p, int32_t* salida) {
  if (p == nullptr) return false;
  bool negativo = false;
  if (*p == '-') {
    negativo = true;
    ++p;
  }
  int digitos = 0;
  int64_t valor = 0;
  while (*p >= '0' && *p <= '9') {
    valor = valor * 10 + (*p - '0');
    ++p;
    if (++digitos > 9) return false;
  }
  if (digitos == 0) return false;
  p = saltar_espacios(p);
  if (*p != ',' && *p != '}') return false;
  *salida = static_cast<int32_t>(negativo ? -valor : valor);
  return true;
}

// Compara el valor de tipo string que empieza en p con `esperado`.
inline bool es_cadena(const char* p, const char* esperado) {
  if (p == nullptr || *p != '"') return false;
  const size_t n = strlen(esperado);
  return strncmp(p + 1, esperado, n) == 0 && p[1 + n] == '"';
}

}  // namespace detalle

// Interpreta una línea (sin el '\n'). Devuelve false si no es un mensaje válido:
// quien llama debe descartarla sin afectar el estado ni el watchdog.
inline bool parsear_linea(const char* linea, Mensaje* salida) {
  *salida = Mensaje();
  if (linea == nullptr) return false;

  const char* inicio = detalle::saltar_espacios(linea);
  if (*inicio != '{') return false;
  const size_t largo = strlen(inicio);
  if (largo < 2 || largo > 120) return false;
  const char* fin = inicio + largo - 1;
  while (fin > inicio && (*fin == ' ' || *fin == '\t' || *fin == '\r')) --fin;
  if (*fin != '}') return false;

  int32_t seq = 0;
  if (!detalle::leer_entero(detalle::buscar_valor(inicio, "seq"), &seq) || seq < 0) {
    return false;
  }

  const char* tipo = detalle::buscar_valor(inicio, "t");
  Mensaje m;
  m.seq = static_cast<uint32_t>(seq);

  if (detalle::es_cadena(tipo, "drive")) {
    if (!detalle::leer_entero(detalle::buscar_valor(inicio, "v"), &m.v_mm_s)) return false;
    if (!detalle::leer_entero(detalle::buscar_valor(inicio, "s"), &m.s_decimas)) return false;
    m.tipo = TipoMensaje::Drive;
  } else if (detalle::es_cadena(tipo, "brake")) {
    m.tipo = TipoMensaje::Brake;
  } else {
    return false;
  }
  *salida = m;
  return true;
}

// ----------------------------------------------------------------------------
// Mensajes hacia la Jetson. Devuelven el largo escrito (sin el '\0'), o -1 si no entra.
// ----------------------------------------------------------------------------

inline int formatear_ultra(char* buf, size_t n, int32_t dist_cm, bool valida) {
  const int r = snprintf(buf, n, "{\"t\":\"ultra\",\"id\":\"front\",\"dist_cm\":%ld,\"valid\":%s}\n",
                         static_cast<long>(valida ? dist_cm : 0), valida ? "true" : "false");
  return (r < 0 || static_cast<size_t>(r) >= n) ? -1 : r;
}

inline int formatear_status(char* buf, size_t n, const char* version, bool watchdog,
                            uint32_t ultima_seq) {
  const int r = snprintf(buf, n, "{\"t\":\"status\",\"fw\":\"%s\",\"wd\":%s,\"last_seq\":%lu}\n",
                         version, watchdog ? "true" : "false", static_cast<unsigned long>(ultima_seq));
  return (r < 0 || static_cast<size_t>(r) >= n) ? -1 : r;
}

// ----------------------------------------------------------------------------
// Watchdog: no lee el reloj, recibe el instante (ms) como argumento.
// La resta sin signo hace que funcione aunque millis() dé la vuelta (cada ~49 días).
// ----------------------------------------------------------------------------

class Watchdog {
 public:
  explicit Watchdog(uint32_t timeout_ms) : timeout_ms_(timeout_ms) {}

  void alimentar(uint32_t ahora_ms) {
    ultimo_ms_ = ahora_ms;
    alimentado_ = true;
  }

  // Antes del primer mensaje válido no se considera vencido: los motores ya
  // arrancan detenidos.
  bool vencido(uint32_t ahora_ms) const {
    return alimentado_ && static_cast<uint32_t>(ahora_ms - ultimo_ms_) >= timeout_ms_;
  }

 private:
  uint32_t timeout_ms_;
  uint32_t ultimo_ms_ = 0;
  bool alimentado_ = false;
};

// ----------------------------------------------------------------------------
// Conversión de unidades del protocolo a señales de los actuadores
// ----------------------------------------------------------------------------

inline int32_t limitar(int32_t valor, int32_t maximo) {
  if (valor > maximo) return maximo;
  if (valor < -maximo) return -maximo;
  return valor;
}

// Magnitud de la velocidad (0..255) para el PWM del puente H. El signo lo da
// el sentido de giro, no este valor.
inline uint8_t velocidad_a_pwm(int32_t v_mm_s, int32_t v_max_mm_s) {
  if (v_max_mm_s <= 0) return 0;
  const int32_t magnitud = limitar(v_mm_s, v_max_mm_s);
  const int32_t absoluto = magnitud < 0 ? -magnitud : magnitud;
  return static_cast<uint8_t>((static_cast<int64_t>(absoluto) * 255) / v_max_mm_s);
}

// Ancho de pulso (us) del servo de dirección. `amplitud_us` es la desviación del
// centro cuando la dirección llega a su máximo; negativo si el servo está invertido.
inline int32_t direccion_a_pulso_us(int32_t s_decimas, int32_t s_max_decimas, int32_t centro_us,
                                    int32_t amplitud_us) {
  if (s_max_decimas <= 0) return centro_us;
  const int32_t s = limitar(s_decimas, s_max_decimas);
  return centro_us + static_cast<int32_t>((static_cast<int64_t>(s) * amplitud_us) / s_max_decimas);
}

}  // namespace aura

#endif  // AURA_PROTOCOL_H
