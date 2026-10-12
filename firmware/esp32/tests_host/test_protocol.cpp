// Tests de protocol.h en la computadora (sin Arduino ni ESP32).
// Cómo correrlos: ./run_tests.sh
#include <climits>
#include <cstdio>
#include <cstring>

#include "protocol.h"

using namespace aura;

static int fallos = 0;
static int pruebas = 0;

#define CHECK(cond)                                                      \
  do {                                                                   \
    ++pruebas;                                                           \
    if (!(cond)) {                                                       \
      ++fallos;                                                          \
      std::printf("  FALLO %s:%d  %s\n", __FILE__, __LINE__, #cond);     \
    }                                                                    \
  } while (0)

#define TEST(nombre) static void nombre()
#define CORRER(nombre)                       \
  do {                                       \
    const int antes = fallos;                \
    nombre();                                \
    std::printf("%s %s\n", fallos == antes ? "ok   " : "FALLA", #nombre); \
  } while (0)

static Mensaje parsear(const char* linea, bool* ok) {
  Mensaje m;
  *ok = parsear_linea(linea, &m);
  return m;
}

// --- parser: mensajes válidos -------------------------------------------------

TEST(drive_valido_se_interpreta) {
  bool ok;
  Mensaje m = parsear("{\"t\":\"drive\",\"seq\":12,\"v\":250,\"s\":-30}", &ok);
  CHECK(ok);
  CHECK(m.tipo == TipoMensaje::Drive);
  CHECK(m.seq == 12u);
  CHECK(m.v_mm_s == 250);
  CHECK(m.s_decimas == -30);
}

TEST(brake_valido_se_interpreta) {
  bool ok;
  Mensaje m = parsear("{\"t\":\"brake\",\"seq\":13}", &ok);
  CHECK(ok);
  CHECK(m.tipo == TipoMensaje::Brake);
  CHECK(m.seq == 13u);
}

TEST(el_orden_de_las_claves_no_importa) {
  bool ok;
  Mensaje m = parsear("{\"s\":5,\"v\":-100,\"seq\":7,\"t\":\"drive\"}", &ok);
  CHECK(ok);
  CHECK(m.v_mm_s == -100);
  CHECK(m.s_decimas == 5);
  CHECK(m.seq == 7u);
}

TEST(se_toleran_espacios_y_retorno_de_carro) {
  bool ok;
  Mensaje m = parsear("  { \"t\" : \"drive\" , \"seq\" : 1 , \"v\" : 10 , \"s\" : 0 }\r", &ok);
  CHECK(ok);
  CHECK(m.v_mm_s == 10);
}

TEST(la_clave_s_no_se_confunde_con_seq) {
  bool ok;
  Mensaje m = parsear("{\"t\":\"drive\",\"seq\":99,\"v\":1,\"s\":2}", &ok);
  CHECK(ok);
  CHECK(m.seq == 99u);
  CHECK(m.s_decimas == 2);
}

// --- parser: mensajes inválidos se descartan --------------------------------------

TEST(los_mensajes_invalidos_se_rechazan) {
  const char* invalidos[] = {
      "",
      "hola",
      "{}",
      "{",
      "}",
      "{\"t\":\"drive\"",                                  // truncado
      "{\"t\":\"drive\",\"seq\":1,\"v\":10}",              // falta s
      "{\"t\":\"drive\",\"seq\":1,\"s\":10}",              // falta v
      "{\"t\":\"drive\",\"v\":10,\"s\":1}",                // falta seq
      "{\"t\":\"drive\",\"seq\":1,\"v\":\"rapido\",\"s\":0}",  // v no es número
      "{\"t\":\"drive\",\"seq\":1,\"v\":,\"s\":0}",        // v vacío
      "{\"t\":\"volar\",\"seq\":1}",                       // tipo desconocido
      "{\"seq\":1,\"v\":1,\"s\":1}",                       // sin tipo
      "{\"t\":\"brake\"}",                                 // falta seq
      "{\"t\":\"brake\",\"seq\":-5}",                      // seq negativo
      "{\"t\":\"drive\",\"seq\":1,\"v\":1234567890123,\"s\":0}",  // desborda
      "{\"t\":\"drivee\",\"seq\":1,\"v\":1,\"s\":1}",      // tipo con sufijo
  };
  for (const char* linea : invalidos) {
    bool ok;
    Mensaje m = parsear(linea, &ok);
    CHECK(!ok);
    CHECK(m.tipo == TipoMensaje::Invalido);
  }
}

TEST(un_puntero_nulo_se_rechaza) {
  Mensaje m;
  CHECK(!parsear_linea(nullptr, &m));
}

TEST(una_linea_demasiado_larga_se_rechaza) {
  char larga[300];
  std::memset(larga, 'a', sizeof(larga));
  larga[0] = '{';
  larga[sizeof(larga) - 2] = '}';
  larga[sizeof(larga) - 1] = '\0';
  Mensaje m;
  CHECK(!parsear_linea(larga, &m));
}

TEST(un_mensaje_invalido_no_modifica_la_salida) {
  Mensaje m;
  m.seq = 77;
  CHECK(!parsear_linea("basura", &m));
  CHECK(m.tipo == TipoMensaje::Invalido);
}

// --- mensajes hacia la Jetson ----------------------------------------------------

TEST(ultra_valida_tiene_el_formato_del_adr) {
  char buf[96];
  const int n = formatear_ultra(buf, sizeof(buf), 87, true);
  CHECK(n > 0);
  CHECK(std::strcmp(buf, "{\"t\":\"ultra\",\"id\":\"front\",\"dist_cm\":87,\"valid\":true}\n") == 0);
}

TEST(ultra_invalida_reporta_distancia_cero) {
  char buf[96];
  formatear_ultra(buf, sizeof(buf), 55, false);
  CHECK(std::strcmp(buf, "{\"t\":\"ultra\",\"id\":\"front\",\"dist_cm\":0,\"valid\":false}\n") == 0);
}

TEST(status_tiene_el_formato_del_adr) {
  char buf[96];
  const int n = formatear_status(buf, sizeof(buf), "0.1.0", false, 12);
  CHECK(n > 0);
  CHECK(std::strcmp(buf, "{\"t\":\"status\",\"fw\":\"0.1.0\",\"wd\":false,\"last_seq\":12}\n") == 0);
}

TEST(status_reporta_el_watchdog_cortado) {
  char buf[96];
  formatear_status(buf, sizeof(buf), "0.1.0", true, 0);
  CHECK(std::strstr(buf, "\"wd\":true") != nullptr);
}

TEST(un_buffer_chico_devuelve_error_en_lugar_de_truncar) {
  char buf[10];
  CHECK(formatear_ultra(buf, sizeof(buf), 87, true) == -1);
  CHECK(formatear_status(buf, sizeof(buf), "0.1.0", false, 1) == -1);
}

TEST(lo_que_envia_el_esp32_es_una_linea_json_cerrada) {
  char buf[96];
  const int n = formatear_ultra(buf, sizeof(buf), 100, true);
  CHECK(buf[n - 1] == '\n');
  CHECK(buf[0] == '{' && buf[n - 2] == '}');
}

// --- watchdog ------------------------------------------------------------------

TEST(antes_del_primer_mensaje_el_watchdog_no_esta_vencido) {
  Watchdog w(300);
  CHECK(!w.vencido(0));
  CHECK(!w.vencido(100000));
}

TEST(el_watchdog_no_vence_antes_del_tiempo_limite) {
  Watchdog w(300);
  w.alimentar(1000);
  CHECK(!w.vencido(1299));
}

TEST(el_watchdog_vence_al_cumplirse_el_tiempo_limite) {
  Watchdog w(300);
  w.alimentar(1000);
  CHECK(w.vencido(1300));
  CHECK(w.vencido(5000));
}

TEST(alimentar_reinicia_la_cuenta) {
  Watchdog w(300);
  w.alimentar(1000);
  w.alimentar(1250);
  CHECK(!w.vencido(1549));
  CHECK(w.vencido(1550));
}

TEST(el_watchdog_funciona_cuando_millis_da_la_vuelta) {
  Watchdog w(300);
  w.alimentar(UINT32_MAX - 100);          // 100 ms antes de la vuelta
  CHECK(!w.vencido(150));                  // 251 ms después
  CHECK(w.vencido(200));                   // 301 ms después
}

// --- conversión de unidades -----------------------------------------------------

TEST(limitar_recorta_en_ambos_sentidos) {
  CHECK(limitar(100, 500) == 100);
  CHECK(limitar(900, 500) == 500);
  CHECK(limitar(-900, 500) == -500);
}

TEST(velocidad_a_pwm_es_proporcional_y_siempre_no_negativa) {
  CHECK(velocidad_a_pwm(0, 500) == 0);
  CHECK(velocidad_a_pwm(500, 500) == 255);
  CHECK(velocidad_a_pwm(-500, 500) == 255);
  CHECK(velocidad_a_pwm(250, 500) == 127);
}

TEST(velocidad_a_pwm_recorta_lo_que_excede_el_maximo) {
  CHECK(velocidad_a_pwm(5000, 500) == 255);
  CHECK(velocidad_a_pwm(-5000, 500) == 255);
}

TEST(velocidad_a_pwm_con_maximo_invalido_devuelve_cero) {
  CHECK(velocidad_a_pwm(100, 0) == 0);
  CHECK(velocidad_a_pwm(100, -5) == 0);
}

TEST(direccion_centrada_da_el_pulso_central) {
  CHECK(direccion_a_pulso_us(0, 250, 1500, 400) == 1500);
}

TEST(direccion_en_los_extremos_da_la_amplitud) {
  CHECK(direccion_a_pulso_us(250, 250, 1500, 400) == 1900);
  CHECK(direccion_a_pulso_us(-250, 250, 1500, 400) == 1100);
}

TEST(direccion_fuera_de_rango_se_recorta) {
  CHECK(direccion_a_pulso_us(9999, 250, 1500, 400) == 1900);
  CHECK(direccion_a_pulso_us(-9999, 250, 1500, 400) == 1100);
}

TEST(direccion_con_servo_invertido_invierte_el_sentido) {
  CHECK(direccion_a_pulso_us(250, 250, 1500, -400) == 1100);
}

TEST(direccion_con_rango_invalido_devuelve_el_centro) {
  CHECK(direccion_a_pulso_us(100, 0, 1500, 400) == 1500);
}

int main() {
  CORRER(drive_valido_se_interpreta);
  CORRER(brake_valido_se_interpreta);
  CORRER(el_orden_de_las_claves_no_importa);
  CORRER(se_toleran_espacios_y_retorno_de_carro);
  CORRER(la_clave_s_no_se_confunde_con_seq);
  CORRER(los_mensajes_invalidos_se_rechazan);
  CORRER(un_puntero_nulo_se_rechaza);
  CORRER(una_linea_demasiado_larga_se_rechaza);
  CORRER(un_mensaje_invalido_no_modifica_la_salida);
  CORRER(ultra_valida_tiene_el_formato_del_adr);
  CORRER(ultra_invalida_reporta_distancia_cero);
  CORRER(status_tiene_el_formato_del_adr);
  CORRER(status_reporta_el_watchdog_cortado);
  CORRER(un_buffer_chico_devuelve_error_en_lugar_de_truncar);
  CORRER(lo_que_envia_el_esp32_es_una_linea_json_cerrada);
  CORRER(antes_del_primer_mensaje_el_watchdog_no_esta_vencido);
  CORRER(el_watchdog_no_vence_antes_del_tiempo_limite);
  CORRER(el_watchdog_vence_al_cumplirse_el_tiempo_limite);
  CORRER(alimentar_reinicia_la_cuenta);
  CORRER(el_watchdog_funciona_cuando_millis_da_la_vuelta);
  CORRER(limitar_recorta_en_ambos_sentidos);
  CORRER(velocidad_a_pwm_es_proporcional_y_siempre_no_negativa);
  CORRER(velocidad_a_pwm_recorta_lo_que_excede_el_maximo);
  CORRER(velocidad_a_pwm_con_maximo_invalido_devuelve_cero);
  CORRER(direccion_centrada_da_el_pulso_central);
  CORRER(direccion_en_los_extremos_da_la_amplitud);
  CORRER(direccion_fuera_de_rango_se_recorta);
  CORRER(direccion_con_servo_invertido_invierte_el_sentido);
  CORRER(direccion_con_rango_invalido_devuelve_el_centro);

  std::printf("\n%d verificaciones, %d fallos\n", pruebas, fallos);
  return fallos == 0 ? 0 : 1;
}
