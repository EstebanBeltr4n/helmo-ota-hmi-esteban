/**
 * @file helmo_nodo_ph.ino
 * @brief HELMO · Firmware 1 de 2: nodo de pH con LoRaWAN Clase A, OTA híbrida segura y enlace HMI.
 * @author Esteban Eduardo Escarraga
 * @version 2.0.0
 * @date 2026-11
 *
 * @details
 * **HELMO** (Hybrid LoRa Environment for Water Monitoring and Optimization).
 *
 * - **Hardware:** Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262), sensor de pH alimentado por MOSFET
 *   y, opcionalmente, el módulo HMI Arduino Nano 33 BLE Sense por UART (ver helmo_hmi_nano.ino).
 * - **Entorno:** Arduino IDE, placa "Heltec WiFi LoRa 32(V3)",
 *   particiones "8M with spiffs (3MB APP/1.5MB SPIFFS)".
 *
 * @par Funciones principales
 * -# Telemetría por LoRaWAN **Clase A** hacia TTN Sandbox (US915, sub-banda 2).
 * -# **OTA híbrida:** el servidor ordena la actualización con un único downlink por TTN (puerto 10);
 *    el nodo descarga la imagen por el Wi-Fi de mantenimiento y lo apaga al terminar.
 * -# **Seguridad de la OTA:** manifiesto firmado con ECDSA P-256, SHA-256 progresivo,
 *    anti-downgrade, watchdog de transferencia y rollback si la versión nueva no se une a la red.
 * -# **Interfaz de voz y gestos:** recibe eventos del módulo HMI y le envía el pH y su estado.
 *
 * @par Puertos LoRaWAN (todos caben en DR0 de US915, máximo 11 bytes; enteros big-endian)
 * @verbatim
  Uplink 3  pH  (9 B): [nodo][pH x100 2B][mV 2B][fw M][fw m][fw p][flags]
                       flags: bit0-1 estado pH (0 normal, 1 aviso, 2 alarma)
                              bit2 HMI conectado | bit3 firmware en periodo de prueba
  Uplink 4  HMI (5 B): [nodo][tipo 1=voz 2=gesto][código][confianza %][acción]
  Uplink 5  OTA (7 B): [nodo][trabajo][resultado][fw M][fw m][fw p][detalle]
                       resultado: 1 EXITO, 2 ERROR, 3 ROLLBACK, 4 YA_AL_DIA
  Downlink 10 comandos:
     01 [trabajo][M][m][p]  -> actualizar por Wi-Fi a la versión M.m.p
     02 [min 2B]            -> cambiar intervalo de envío (1..1440 min)
     03                      -> medir pH ya
     04                      -> reiniciar
 @endverbatim
 *
 * @par Prueba visual (LED blanco, GPIO 35)
 * FW_MINOR par = parpadeo lento · FW_MINOR impar = triple destello · durante la OTA = parpadeo rápido.
 *
 * @note Las credenciales están en `secretos.h`, que **no** se publica ni se documenta.
 *       Usa `secretos.ejemplo.h` como plantilla.
 * @see helmo_hmi_nano.ino, firmar_firmware.py, helmo_ota.py, ttn_formatter.js
 */

#include "LoRaWan_APP.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "mbedtls/sha256.h"
#include "mbedtls/pk.h"
#include "mbedtls/base64.h"

#include "secretos.h"            // copia de secretos.ejemplo.h con tus llaves (NO se publica)
#include "clave_publica_ota.h"   // la genera servidor/firmar_firmware.py

/* -----------------------------------------------------------------------------------------
 * 0. VERSIÓN
 * ----------------------------------------------------------------------------------------- */
#define FW_MAJOR 2
#define FW_MINOR 0
#define FW_PATCH 0
#define FW_PRODUCTO "helmo-ph-node"   // debe coincidir con el manifiesto firmado
#define FW_CANAL    "ph-node"         // carpeta en el servidor de distribución

/* -----------------------------------------------------------------------------------------
 * 1. HARDWARE Y NODO
 * ----------------------------------------------------------------------------------------- */
#define NODE_ID 1
const char* NODO_ALIAS = "ph-lab-01";

#define ADC_PH_PIN 1
#define MOSFET_PIN 6
#define LED_PIN    35

#define HMI_HABILITADO 1     // 0 si el nodo trabaja sin el módulo de voz/gestos
#define HMI_RX_PIN 47        // <- TX (D1) del Nano 33 BLE Sense
#define HMI_TX_PIN 48        // -> RX (D0) del Nano 33 BLE Sense
#define HMI_BAUD   115200
#define HMI_TIMEOUT_MS 30000 // sin mensajes en este tiempo = HMI desconectado

/* -----------------------------------------------------------------------------------------
 * 2. CALIBRACIÓN DE pH (2 puntos) Y SEMÁFORO
 *    Rango aceptable para agua de consumo humano en Colombia: 6,5 - 9,0
 * ----------------------------------------------------------------------------------------- */
#define PH4_BUFFER_VALUE 4.00
#define PH7_BUFFER_VALUE 7.00
#define PH4_VOLTAGE 2.560
#define PH7_VOLTAGE 2.250
#define PH_M ((PH7_BUFFER_VALUE - PH4_BUFFER_VALUE) / (PH7_VOLTAGE - PH4_VOLTAGE))
#define PH_B (PH7_BUFFER_VALUE - (PH_M * PH7_VOLTAGE))

#define PH_LIMITE_MIN   6.5
#define PH_LIMITE_MAX   9.0
#define PH_MARGEN_AVISO 0.3   // a menos de 0,3 del límite = aviso (amarillo)

/* -----------------------------------------------------------------------------------------
 * 3. TIEMPOS
 *    Uso justo de TTN: ~30 s de aire/día por nodo. Con 9 bytes en DR3, 5 min ≈ 17 s/día.
 *    Para pruebas de laboratorio se puede bajar a 60 s por poco tiempo (o con el downlink 02).
 * ----------------------------------------------------------------------------------------- */
#define INTERVALO_TX_MS       300000UL   // 5 min
#define SEPARACION_MIN_TX_MS   20000UL   // separación mínima entre uplinks forzados
#define WIFI_TIMEOUT_MS        30000UL
#define OTA_TIMEOUT_DATOS_MS   10000UL
#define PRUEBA_TIMEOUT_MS    (15UL * 60UL * 1000UL)
#define PRUEBA_MAX_ARRANQUES   3

/* -----------------------------------------------------------------------------------------
 * 4. PARÁMETROS LoRaWAN (nombres exigidos por la librería Heltec)
 * ----------------------------------------------------------------------------------------- */
uint8_t nwkSKey[] = { 0x00 };
uint8_t appSKey[] = { 0x00 };
uint32_t devAddr = 0x00;

uint16_t userChannelsMask[6] = { 0xFF00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };  // US915 FSB2
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;
DeviceClass_t loraWanClass = CLASS_A;    // bajo consumo; la OTA ya no necesita Clase C
uint32_t appTxDutyCycle = INTERVALO_TX_MS;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool keepNet = false;     // tras reiniciar hace JOIN de nuevo: el Join Accept prueba el enlace
bool isTxConfirmed = false;
uint8_t appPort = 3;
uint8_t confirmedNbTrials = 4;

#define PUERTO_PH  3
#define PUERTO_HMI 4
#define PUERTO_OTA 5
#define PUERTO_CMD 10

enum : uint8_t { CMD_OTA_WIFI = 0x01, CMD_INTERVALO = 0x02, CMD_MEDIR_YA = 0x03, CMD_REINICIAR = 0x04 };
enum : uint8_t { OTA_EXITO = 1, OTA_ERROR = 2, OTA_ROLLBACK = 3, OTA_YA_AL_DIA = 4 };
enum : uint8_t {
  E_NINGUNO = 0, E_WIFI = 1, E_MANIF_HTTP = 2, E_MANIF_JSON = 3, E_PRODUCTO = 4, E_VERSION = 5,
  E_FIRMA = 6, E_DESCARGA_HTTP = 7, E_TAMANO = 8, E_INTERRUMPIDA = 9, E_FLASH = 10,
  E_SHA256 = 11, E_IMAGEN = 12, E_OCUPADO = 13
};

/* -----------------------------------------------------------------------------------------
 * 5. ESTADO GLOBAL
 * ----------------------------------------------------------------------------------------- */
Preferences prefs;
bool unido = false;
unsigned long ultimoUplinkMs = 0;
volatile bool phForzado = false;
volatile bool reinicioSolicitado = false;

float ultimoPH = -1.0;
uint8_t estadoPH = 0;          // 0 normal, 1 aviso, 2 alarma
bool alertaReconocida = false;

/** @brief Orden de actualización recibida por downlink (trabajo y versión M.m.p). */
struct OrdenOTA { volatile bool pendiente; uint8_t trabajo, M, m, p; };
OrdenOTA ordenOTA = { false, 0, 0, 0, 0 };

bool reporteOTAPendiente = false;
uint8_t repTrabajo = 0, repResultado = 0, repDetalle = 0;

bool enPrueba = false;
unsigned long pruebaInicioMs = 0;
uint8_t trabajoEnPrueba = 0;

/** @brief Evento de voz o gesto pendiente de enviar por el puerto 4. */
struct EventoHMI { uint8_t tipo, codigo, confianza, accion; };
#define COLA_HMI_MAX 4
EventoHMI colaHMI[COLA_HMI_MAX];
uint8_t nColaHMI = 0;
unsigned long ultimoMensajeHMIms = 0;
bool otaEnCurso = false;

/* -----------------------------------------------------------------------------------------
 * 6. LED DE VERSIÓN
 * ----------------------------------------------------------------------------------------- */
#if (FW_MINOR % 2 == 0)
const uint16_t patronLED[] = { 1000, 1000 };                    // lento
#else
const uint16_t patronLED[] = { 80, 150, 80, 150, 80, 1400 };     // triple destello
#endif
const uint8_t pasosPatron = sizeof(patronLED) / sizeof(patronLED[0]);
uint8_t pasoLED = 0;
unsigned long ultimoCambioLED = 0;
bool estadoLED = false;

/** @brief Enciende o apaga el LED blanco y recuerda su estado. @param on true = encendido. */
void escribirLED(bool on) { estadoLED = on; digitalWrite(LED_PIN, on ? HIGH : LOW); }

/** @brief Avanza el patrón de parpadeo que identifica la versión, sin bloquear el bucle. */
void actualizarLED() {
  if (otaEnCurso) return;
  if (millis() - ultimoCambioLED >= patronLED[pasoLED]) {
    pasoLED = (pasoLED + 1) % pasosPatron;
    ultimoCambioLED = millis();
    escribirLED(pasoLED % 2 == 0);
  }
}

/* -----------------------------------------------------------------------------------------
 * 7. UTILIDADES
 * ----------------------------------------------------------------------------------------- */
/**
 * @brief Empaqueta una versión semántica en un entero comparable.
 * @param M mayor  @param m menor  @param p parche
 * @return (M<<16)|(m<<8)|p
 */
uint32_t versionNum(uint8_t M, uint8_t m, uint8_t p) { return ((uint32_t)M << 16) | ((uint32_t)m << 8) | p; }
/** @brief Versión del firmware en ejecución como entero comparable. */
uint32_t versionActual() { return versionNum(FW_MAJOR, FW_MINOR, FW_PATCH); }

/**
 * @brief Convierte un texto "M.m.p" en sus tres componentes.
 * @param s texto de la versión
 * @param[out] M mayor  @param[out] m menor  @param[out] p parche
 * @return true si el formato es válido (cada parte entre 0 y 255).
 */
bool parsearVersion(const char* s, uint8_t& M, uint8_t& m, uint8_t& p) {
  int a, b, c;
  if (!s || sscanf(s, "%d.%d.%d", &a, &b, &c) != 3) return false;
  if (a < 0 || a > 255 || b < 0 || b > 255 || c < 0 || c > 255) return false;
  M = a; m = b; p = c;
  return true;
}

/** @brief Calcula el SHA-256 de un texto. @param texto entrada @param[out] salida 32 bytes del hash. */
void sha256Texto(const String& texto, uint8_t salida[32]) {
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);
  mbedtls_sha256_update(&ctx, (const unsigned char*)texto.c_str(), texto.length());
  mbedtls_sha256_finish(&ctx, salida);
  mbedtls_sha256_free(&ctx);
}

/** @brief Convierte bytes a texto hexadecimal en minúsculas. */
String aHex(const uint8_t* h, size_t n) {
  static const char* dig = "0123456789abcdef";
  String s; s.reserve(n * 2);
  for (size_t i = 0; i < n; i++) { s += dig[h[i] >> 4]; s += dig[h[i] & 0x0F]; }
  return s;
}

/** @brief Comprueba que un texto sea un hash SHA-256 en hexadecimal (64 caracteres). */
bool esHex64(const char* h) {
  if (!h || strlen(h) != 64) return false;
  for (int i = 0; i < 64; i++) if (!isxdigit((unsigned char)h[i])) return false;
  return true;
}

/* Verifica la firma ECDSA P-256 (DER en base64) del mensaje canónico del manifiesto */
/**
 * @brief Verifica la firma ECDSA P-256 del manifiesto con la clave pública de HELMO.
 * @param mensaje texto canónico "HELMO-OTA|producto|versión|tamaño|sha256"
 * @param firmaB64 firma DER codificada en base64
 * @return true solo si la firma es auténtica.
 */
bool verificarFirma(const String& mensaje, const char* firmaB64) {
  if (!firmaB64) return false;
  uint8_t firma[128];
  size_t lenFirma = 0;
  if (mbedtls_base64_decode(firma, sizeof(firma), &lenFirma,
                            (const unsigned char*)firmaB64, strlen(firmaB64)) != 0) return false;

  uint8_t hash[32];
  sha256Texto(mensaje, hash);

  mbedtls_pk_context pk;
  mbedtls_pk_init(&pk);
  int r = mbedtls_pk_parse_public_key(&pk, (const unsigned char*)OTA_CLAVE_PUBLICA_PEM,
                                      strlen(OTA_CLAVE_PUBLICA_PEM) + 1);
  if (r != 0) {
    Serial.printf("[OTA] Clave pública inválida (-0x%04X). ¿Generaste clave_publica_ota.h?\n", -r);
  } else {
    r = mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, hash, sizeof(hash), firma, lenFirma);
  }
  mbedtls_pk_free(&pk);
  return r == 0;
}

/* -----------------------------------------------------------------------------------------
 * 8. ENLACE CON EL MÓDULO HMI (Nano 33 BLE Sense)
 *    Nano -> Heltec:  HOLA,<ver> | PING | EVT,V,<etiqueta>,<conf%> | EVT,G,<etiqueta>,<conf%>
 *    Heltec -> Nano:  PH,<pH x100>,<estado>,<reconocida> | OTA,<1|0> | ACK,<etiqueta>
 * ----------------------------------------------------------------------------------------- */
enum : uint8_t { ACC_INCIDENTE = 1, ACC_MEDIR = 2, ACC_RECONOCER = 3, ACC_MOSTRAR = 4 };

/** @brief Regla que asocia una palabra o gesto con una acción del nodo. */
struct ReglaHMI { uint8_t tipo; const char* etiqueta; uint8_t codigo; uint8_t accion; bool uplink; };
// Ajusta aquí el significado de cada palabra/gesto. "uplink" = se reporta al servidor HELMO.
/** @brief Significado de cada palabra y gesto. Edita esta tabla para cambiar el comportamiento. */
const ReglaHMI REGLAS_HMI[] = {
  { 1, "Rojo",     1, ACC_INCIDENTE, true  },   // operario reporta un incidente
  { 1, "Amarillo", 2, ACC_MEDIR,     false },   // pide medición inmediata (el pH ya viaja por el puerto 3)
  { 1, "Verde",    3, ACC_RECONOCER, true  },   // operario reconoce la alarma activa
  { 2, "punch",    1, ACC_MEDIR,     false },   // gesto alternativo para planta ruidosa
  { 2, "flex",     2, ACC_MOSTRAR,   false },   // muestra el último pH en el LED del HMI
};
const uint8_t N_REGLAS = sizeof(REGLAS_HMI) / sizeof(REGLAS_HMI[0]);

/** @brief Envía una línea de texto al módulo HMI por UART (Serial1). */
void enviarHMI(const String& linea) {
#if HMI_HABILITADO
  Serial1.print(linea);
  Serial1.print('\n');
#endif
}

/** @brief Envía al HMI el último pH, su estado de semáforo y si la alarma fue reconocida. */
void enviarPHaHMI() {
  if (ultimoPH < 0) return;
  enviarHMI("PH," + String((int)(ultimoPH * 100.0 + 0.5)) + "," + String(estadoPH) + "," +
            String(alertaReconocida ? 1 : 0));
}

/** @brief Indica si el módulo HMI respondió en los últimos HMI_TIMEOUT_MS. */
bool hmiConectado() {
#if HMI_HABILITADO
  return ultimoMensajeHMIms != 0 && (millis() - ultimoMensajeHMIms) < HMI_TIMEOUT_MS;
#else
  return false;
#endif
}

/** @brief Guarda un evento de voz o gesto para enviarlo por el puerto 4. */
void encolarEventoHMI(uint8_t tipo, uint8_t codigo, uint8_t conf, uint8_t accion) {
  if (nColaHMI >= COLA_HMI_MAX) { Serial.println("[HMI] Cola llena, evento descartado"); return; }
  colaHMI[nColaHMI++] = { tipo, codigo, conf, accion };
}

/**
 * @brief Interpreta un mensaje del HMI (HOLA, PING o EVT) y ejecuta la acción de REGLAS_HMI.
 * @param linea mensaje sin salto de línea; se modifica al separar los campos.
 */
void procesarLineaHMI(char* linea) {
  ultimoMensajeHMIms = millis();
  char* campo[5] = { nullptr };
  uint8_t n = 0;
  for (char* t = strtok(linea, ","); t && n < 5; t = strtok(nullptr, ",")) campo[n++] = t;
  if (n == 0) return;

  if (!strcmp(campo[0], "HOLA")) {
    Serial.printf("[HMI] Módulo conectado (v%s)\n", n > 1 ? campo[1] : "?");
    enviarPHaHMI();
    return;
  }
  if (!strcmp(campo[0], "PING")) return;
  if (strcmp(campo[0], "EVT") || n < 4) return;

  uint8_t tipo = (campo[1][0] == 'V') ? 1 : (campo[1][0] == 'G') ? 2 : 0;
  int conf = constrain(atoi(campo[3]), 0, 100);

  static unsigned long ultimoEventoMs = 0;
  static uint8_t ultimoCodigo = 0, ultimoTipo = 0;

  for (uint8_t i = 0; i < N_REGLAS; i++) {
    const ReglaHMI& r = REGLAS_HMI[i];
    if (r.tipo != tipo || strcasecmp(r.etiqueta, campo[2]) != 0) continue;

    // antirrebote: el mismo evento dentro de 3 s se ignora
    if (r.codigo == ultimoCodigo && r.tipo == ultimoTipo && millis() - ultimoEventoMs < 3000) return;
    ultimoEventoMs = millis(); ultimoCodigo = r.codigo; ultimoTipo = r.tipo;

    Serial.printf("[HMI] %s '%s' (%d%%)\n", tipo == 1 ? "Voz" : "Gesto", r.etiqueta, conf);
    enviarHMI(String("ACK,") + r.etiqueta);

    switch (r.accion) {
      case ACC_MEDIR:     phForzado = true; break;
      case ACC_RECONOCER: if (estadoPH == 2) alertaReconocida = true; enviarPHaHMI(); break;
      case ACC_MOSTRAR:   enviarPHaHMI(); break;
      case ACC_INCIDENTE: break;
    }
    if (r.uplink) encolarEventoHMI(r.tipo, r.codigo, conf, r.accion);
    return;
  }
  Serial.printf("[HMI] Etiqueta sin regla: %s\n", campo[2]);
}

/** @brief Lee el UART del HMI sin bloquear y procesa cada línea completa. */
void atenderHMI() {
#if HMI_HABILITADO
  static char buf[72];
  static uint8_t len = 0;
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') { buf[len] = 0; if (len) procesarLineaHMI(buf); len = 0; }
    else if (c != '\r' && len < sizeof(buf) - 1) buf[len++] = c;
  }
#endif
}

/* -----------------------------------------------------------------------------------------
 * 9. MEDICIÓN DE pH
 * ----------------------------------------------------------------------------------------- */
/** @brief Alimenta el sensor de pH por el MOSFET; al encender espera 3 s de estabilización. */
void energizarSensorPH(bool on) { digitalWrite(MOSFET_PIN, on ? HIGH : LOW); if (on) delay(3000); }

/** @brief Toma 10 lecturas del ADC, descarta los extremos y promedia las 6 centrales. @return voltaje en V. */
float leerVoltajePHFiltrado() {
  const int N = 10;
  int m[N];
  for (int i = 0; i < N; i++) { m[i] = analogRead(ADC_PH_PIN); delay(20); }
  for (int i = 0; i < N - 1; i++)
    for (int j = i + 1; j < N; j++)
      if (m[i] > m[j]) { int t = m[i]; m[i] = m[j]; m[j] = t; }
  long suma = 0;
  for (int i = 2; i < 8; i++) suma += m[i];
  return (suma / 6.0) * (3.3 / 4095.0);
}

/** @brief Clasifica el pH según la norma de agua potable. @return 0 normal, 1 aviso, 2 alarma. */
uint8_t clasificarPH(float ph) {
  if (ph < PH_LIMITE_MIN || ph > PH_LIMITE_MAX) return 2;
  if (ph < PH_LIMITE_MIN + PH_MARGEN_AVISO || ph > PH_LIMITE_MAX - PH_MARGEN_AVISO) return 1;
  return 0;
}

/** @brief Mide el pH, actualiza el semáforo y arma el payload del puerto 3. */
void prepararUplinkPH() {
  energizarSensorPH(true);
  float voltaje = leerVoltajePHFiltrado();
  energizarSensorPH(false);

  float ph = constrain(PH_M * voltaje + PH_B, 0.0, 14.0);
  uint8_t nuevoEstado = clasificarPH(ph);
  if (nuevoEstado == 2 && estadoPH != 2) alertaReconocida = false;   // alarma nueva
  ultimoPH = ph;
  estadoPH = nuevoEstado;

  uint16_t phInt = (uint16_t)(ph * 100.0 + 0.5);
  uint16_t mv = (uint16_t)(voltaje * 1000.0 + 0.5);
  uint8_t flags = (estadoPH & 0x03) | (hmiConectado() ? 0x04 : 0) | (enPrueba ? 0x08 : 0);

  appPort = PUERTO_PH;
  appDataSize = 9;
  appData[0] = NODE_ID;
  appData[1] = phInt >> 8;  appData[2] = phInt & 0xFF;
  appData[3] = mv >> 8;     appData[4] = mv & 0xFF;
  appData[5] = FW_MAJOR;    appData[6] = FW_MINOR;  appData[7] = FW_PATCH;
  appData[8] = flags;

  Serial.printf("[pH] v%d.%d.%d | pH %.2f | %.3f V | estado %d%s\n", FW_MAJOR, FW_MINOR, FW_PATCH,
                ph, voltaje, estadoPH, hmiConectado() ? " | HMI" : "");
  enviarPHaHMI();
}

/** @brief Saca el evento HMI más antiguo de la cola y arma el payload del puerto 4. */
void prepararUplinkHMI() {
  EventoHMI e = colaHMI[0];
  for (uint8_t i = 1; i < nColaHMI; i++) colaHMI[i - 1] = colaHMI[i];
  nColaHMI--;
  appPort = PUERTO_HMI;
  appDataSize = 5;
  appData[0] = NODE_ID; appData[1] = e.tipo; appData[2] = e.codigo;
  appData[3] = e.confianza; appData[4] = e.accion;
  Serial.printf("[LoRa] Evento HMI tipo %d código %d\n", e.tipo, e.codigo);
}

/** @brief Deja pendiente un reporte de resultado OTA para el puerto 5. */
void programarReporteOTA(uint8_t trabajo, uint8_t resultado, uint8_t detalle) {
  repTrabajo = trabajo; repResultado = resultado; repDetalle = detalle;
  reporteOTAPendiente = true;
}

/** @brief Arma el payload del puerto 5 con el resultado del trabajo OTA. */
void prepararUplinkOTA() {
  appPort = PUERTO_OTA;
  appDataSize = 7;
  appData[0] = NODE_ID; appData[1] = repTrabajo; appData[2] = repResultado;
  appData[3] = FW_MAJOR; appData[4] = FW_MINOR; appData[5] = FW_PATCH; appData[6] = repDetalle;
  reporteOTAPendiente = false;
  Serial.printf("[LoRa] Reporte OTA trabajo %d resultado %d detalle %d\n",
                repTrabajo, repResultado, repDetalle);
}

/* -----------------------------------------------------------------------------------------
 * 10. PERIODO DE PRUEBA Y ROLLBACK
 * ----------------------------------------------------------------------------------------- */
// Evita que el núcleo Arduino marque la imagen como válida apenas arranca:
// la marcamos nosotros cuando el nodo demuestra que se une a la red.
/**
 * @brief Evita que el núcleo Arduino confirme la imagen al arrancar.
 * @details La confirmación la hace confirmarFirmware() cuando el nodo logra unirse a la red.
 */
extern "C" bool verifyRollbackLater() { return true; }

/** @brief Vuelve a la partición anterior (rollback) y reinicia. @param motivo texto para el registro serie. */
void revertirFirmware(const char* motivo) {
  Serial.printf("[ROLLBACK] %s. Volviendo a la versión anterior...\n", motivo);
  esp_ota_mark_app_invalid_rollback_and_reboot();   // si el bootloader lo soporta, no retorna

  // Respaldo por software (bootloader sin rollback habilitado)
  prefs.begin("helmo-ota", true);
  String anterior = prefs.getString("anterior", "");
  prefs.end();
  const esp_partition_t* p = esp_partition_find_first(ESP_PARTITION_TYPE_APP,
                                                      ESP_PARTITION_SUBTYPE_ANY, anterior.c_str());
  if (p && esp_ota_set_boot_partition(p) == ESP_OK) {
    delay(200);
    ESP.restart();
  }
  Serial.println("[ROLLBACK] No fue posible revertir; se continúa con la versión actual.");
  enPrueba = false;
}

/**
 * @brief Al arrancar, detecta si viene de una OTA: inicia el periodo de prueba,
 *        cuenta reinicios o reporta un rollback ya ocurrido.
 */
void revisarArranqueTrasOTA() {
  prefs.begin("helmo-ota", false);
  bool prueba = prefs.getBool("prueba", false);
  if (!prueba) { prefs.end(); esp_ota_mark_app_valid_cancel_rollback(); return; }

  String anterior = prefs.getString("anterior", "");
  uint8_t trabajo = prefs.getUChar("trabajo", 0);
  const esp_partition_t* corriendo = esp_ota_get_running_partition();

  if (corriendo && anterior.length() && anterior == corriendo->label) {
    // Arrancamos en la partición vieja: la versión nueva falló y hubo rollback
    prefs.putBool("prueba", false);
    prefs.end();
    esp_ota_mark_app_valid_cancel_rollback();
    programarReporteOTA(trabajo, OTA_ROLLBACK, 0);
    Serial.println("[OTA] Se detectó ROLLBACK a la versión anterior");
    return;
  }

  uint8_t arranques = prefs.getUChar("arranques", 0) + 1;
  prefs.putUChar("arranques", arranques);
  prefs.end();
  Serial.printf("[OTA] Versión nueva en PERIODO DE PRUEBA (arranque %d de %d)\n",
                arranques, PRUEBA_MAX_ARRANQUES);
  if (arranques > PRUEBA_MAX_ARRANQUES) revertirFirmware("Demasiados reinicios");

  enPrueba = true;
  pruebaInicioMs = millis();
  trabajoEnPrueba = trabajo;
}

/** @brief Marca la versión nueva como válida tras el Join Accept y programa el reporte EXITO. */
void confirmarFirmware() {
  esp_ota_mark_app_valid_cancel_rollback();
  prefs.begin("helmo-ota", false);
  prefs.putBool("prueba", false);
  prefs.putUChar("arranques", 0);
  prefs.end();
  enPrueba = false;
  programarReporteOTA(trabajoEnPrueba, OTA_EXITO, 0);
  Serial.printf("[OTA] Versión %d.%d.%d CONFIRMADA (se unió a la red)\n", FW_MAJOR, FW_MINOR, FW_PATCH);
}

/* -----------------------------------------------------------------------------------------
 * 11. OTA POR WI-FI ORDENADA DESDE LoRaWAN
 * ----------------------------------------------------------------------------------------- */
WiFiClient clienteHttp;
WiFiClientSecure clienteHttps;

/** @brief Prepara un cliente HTTP o HTTPS (para ngrok) con tiempo límite y redirecciones. */
bool iniciarHTTP(HTTPClient& http, const String& url) {
  bool ok;
  if (url.startsWith("https://")) {
    // La autenticidad la garantiza la firma ECDSA; TLS solo protege el canal.
    clienteHttps.setInsecure();
    ok = http.begin(clienteHttps, url);
  } else {
    ok = http.begin(clienteHttp, url);
  }
  if (ok) {
    http.addHeader("ngrok-skip-browser-warning", "1");
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    http.setTimeout(OTA_TIMEOUT_DATOS_MS);
  }
  return ok;
}

/** @brief Intenta conectarse a las redes de mantenimiento definidas en secretos.h. @return true si hay conexión. */
bool conectarWiFiMantenimiento() {
  const char* ssids[] = { WIFI_MANT_SSID_1, WIFI_MANT_SSID_2 };
  const char* claves[] = { WIFI_MANT_PASS_1, WIFI_MANT_PASS_2 };
  WiFi.mode(WIFI_STA);
  for (int i = 0; i < 2; i++) {
    if (strlen(ssids[i]) == 0) continue;
    Serial.printf("[WiFi] Conectando a %s ", ssids[i]);
    WiFi.begin(ssids[i], claves[i]);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_TIMEOUT_MS / 2) {
      delay(250); Serial.print("."); escribirLED(!estadoLED);
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\n[WiFi] Conectado. IP %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
    Serial.println(" sin conexión");
    WiFi.disconnect(true);
  }
  return false;
}

/** @brief Desconecta y apaga el Wi-Fi para volver al bajo consumo. */
void apagarWiFi() { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); }

/* Devuelve un código E_*; si todo sale bien reinicia y no retorna */
/**
 * @brief Descarga, verifica e instala una imagen firmada.
 * @details Pasos: manifiesto → firma ECDSA → anti-downgrade → descarga con SHA-256 progresivo
 *          → Update.end() → periodo de prueba → reinicio.
 * @param trabajo número del trabajo asignado por el servidor
 * @param M,m,p versión ordenada
 * @return código E_* de error; si tiene éxito reinicia y no retorna.
 */
uint8_t ejecutarOTA(uint8_t trabajo, uint8_t M, uint8_t m, uint8_t p) {
  if (!conectarWiFiMantenimiento()) return E_WIFI;

  String base = String(OTA_BASE_URL) + "/" + FW_CANAL + "/";

  // --- 1. Manifiesto ---
  HTTPClient http;
  if (!iniciarHTTP(http, base + "manifest.json")) return E_MANIF_HTTP;
  int code = http.GET();
  if (code != HTTP_CODE_OK) { Serial.printf("[OTA] Manifiesto HTTP %d\n", code); http.end(); return E_MANIF_HTTP; }
  String json = http.getString();
  http.end();

  DynamicJsonDocument doc(1536);
  if (deserializeJson(doc, json)) return E_MANIF_JSON;
  const char* producto = doc["producto"];
  const char* version  = doc["version"];
  const char* archivo  = doc["archivo"];
  const char* sha      = doc["sha256"];
  const char* firma    = doc["firma"];
  uint32_t tamano      = doc["tamano"] | 0;
  if (!producto || !version || !archivo || !esHex64(sha) || !firma || tamano == 0) return E_MANIF_JSON;

  if (strcmp(producto, FW_PRODUCTO) != 0) return E_PRODUCTO;

  uint8_t vM, vm, vp;
  if (!parsearVersion(version, vM, vm, vp)) return E_MANIF_JSON;
  // Anti-downgrade: debe ser exactamente la versión ordenada y mayor que la actual
  if (versionNum(vM, vm, vp) != versionNum(M, m, p) || versionNum(vM, vm, vp) <= versionActual())
    return E_VERSION;

  // --- 2. Firma del manifiesto (antes de tocar la flash) ---
  String shaMin = String(sha); shaMin.toLowerCase();
  String canonico = String("HELMO-OTA|") + producto + "|" + version + "|" + String(tamano) + "|" + shaMin;
  if (!verificarFirma(canonico, firma)) {
    Serial.println("[ALERTA SEGURIDAD] Firma del manifiesto INVÁLIDA. Actualización rechazada.");
    return E_FIRMA;
  }
  Serial.printf("[OTA] Manifiesto firmado OK: %s v%s (%u bytes)\n", producto, version, tamano);

  // --- 3. Descarga con SHA-256 progresivo ---
  if (!iniciarHTTP(http, base + archivo)) return E_DESCARGA_HTTP;
  code = http.GET();
  if (code != HTTP_CODE_OK) { http.end(); return E_DESCARGA_HTTP; }
  int largo = http.getSize();
  if (largo > 0 && (uint32_t)largo != tamano) { http.end(); return E_TAMANO; }

  const esp_partition_t* activa = esp_ota_get_running_partition();
  if (!Update.begin(tamano)) {
    Serial.printf("[OTA] Update.begin: %s\n", Update.errorString());
    http.end();
    return E_TAMANO;
  }

  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  mbedtls_sha256_starts(&ctx, 0);

  WiFiClient* stream = http.getStreamPtr();
  static uint8_t buffer[1024];
  uint32_t recibidos = 0;
  unsigned long ultimoDato = millis();
  int siguientePct = 0;
  bool falloFlash = false;

  while (recibidos < tamano) {
    size_t disp = stream->available();
    if (disp > 0) {
      size_t aLeer = min(min(disp, sizeof(buffer)), (size_t)(tamano - recibidos));
      int n = stream->readBytes(buffer, aLeer);
      if (n <= 0) continue;
      mbedtls_sha256_update(&ctx, buffer, n);
      if (Update.write(buffer, n) != (size_t)n) { falloFlash = true; break; }
      recibidos += n;
      ultimoDato = millis();
      escribirLED(!estadoLED);
      int pct = (int)((uint64_t)recibidos * 100 / tamano);
      if (pct >= siguientePct) { Serial.printf("[OTA] %u/%u (%d%%)\n", recibidos, tamano, pct); siguientePct = (pct / 10 + 1) * 10; }
    } else {
      if (!http.connected()) break;
      if (millis() - ultimoDato > OTA_TIMEOUT_DATOS_MS) { Serial.println("[OTA] TIMEOUT sin datos"); break; }
      delay(1);
    }
  }
  http.end();

  uint8_t hash[32];
  mbedtls_sha256_finish(&ctx, hash);
  mbedtls_sha256_free(&ctx);

  if (falloFlash) { Update.abort(); return E_FLASH; }
  if (recibidos < tamano) {
    Update.abort();
    Serial.printf("[RESILIENCIA] Interrumpida en %u/%u bytes. Partición %s intacta.\n",
                  recibidos, tamano, activa ? activa->label : "?");
    return E_INTERRUMPIDA;
  }

  String calculado = aHex(hash, 32);
  Serial.printf("[CRIPTO] SHA-256 calculado: %s\n[CRIPTO] SHA-256 firmado : %s\n", calculado.c_str(), shaMin.c_str());
  if (calculado != shaMin) { Update.abort(); Serial.println("[ALERTA SEGURIDAD] Imagen alterada"); return E_SHA256; }

  if (!Update.end(true)) { Serial.printf("[OTA] Update.end: %s\n", Update.errorString()); return E_IMAGEN; }

  // --- 4. Periodo de prueba en el próximo arranque ---
  prefs.begin("helmo-ota", false);
  prefs.putBool("prueba", true);
  prefs.putString("anterior", activa ? activa->label : "");
  prefs.putUChar("trabajo", trabajo);
  prefs.putUChar("arranques", 0);
  prefs.end();

  Serial.printf("[OTA] v%s verificada y marcada para arranque. Reiniciando...\n", version);
  apagarWiFi();
  enviarHMI("OTA,0");
  delay(1000);
  ESP.restart();
  return E_NINGUNO;   // no se alcanza
}

/** @brief Atiende una orden OTA recibida por downlink y programa el reporte del resultado. */
void procesarOrdenOTA() {
  uint8_t trabajo = ordenOTA.trabajo, M = ordenOTA.M, m = ordenOTA.m, p = ordenOTA.p;
  ordenOTA.pendiente = false;

  Serial.printf("[OTA] Orden recibida: trabajo %d -> v%d.%d.%d (actual %d.%d.%d)\n",
                trabajo, M, m, p, FW_MAJOR, FW_MINOR, FW_PATCH);

  if (versionNum(M, m, p) <= versionActual()) { programarReporteOTA(trabajo, OTA_YA_AL_DIA, 0); return; }
  if (enPrueba) { programarReporteOTA(trabajo, OTA_ERROR, E_OCUPADO); return; }

  otaEnCurso = true;
  enviarHMI("OTA,1");
  uint8_t err = ejecutarOTA(trabajo, M, m, p);   // solo retorna si falló
  apagarWiFi();
  otaEnCurso = false;
  enviarHMI("OTA,0");
  Serial.printf("[OTA] Actualización abortada (error %d). Se sigue con la versión actual.\n", err);
  programarReporteOTA(trabajo, OTA_ERROR, err);
}

/* -----------------------------------------------------------------------------------------
 * 12. DOWNLINKS (callback de la librería: solo se guardan banderas)
 * ----------------------------------------------------------------------------------------- */
/**
 * @brief Callback de la librería Heltec para los downlinks.
 * @details Solo guarda banderas; el trabajo pesado se hace en loop() para no bloquear la pila LoRaWAN.
 */
void downLinkDataHandle(McpsIndication_t* ind) {
  if (ind->Port != PUERTO_CMD || ind->BufferSize == 0) return;
  const uint8_t* d = ind->Buffer;
  uint8_t n = ind->BufferSize;
  switch (d[0]) {
    case CMD_OTA_WIFI:
      if (n >= 5) {
        ordenOTA.trabajo = d[1]; ordenOTA.M = d[2]; ordenOTA.m = d[3]; ordenOTA.p = d[4];
        ordenOTA.pendiente = true;
      }
      break;
    case CMD_INTERVALO:
      if (n >= 3) {
        uint16_t minutos = ((uint16_t)d[1] << 8) | d[2];
        if (minutos >= 1 && minutos <= 1440) appTxDutyCycle = (uint32_t)minutos * 60000UL;
      }
      break;
    case CMD_MEDIR_YA:  phForzado = true; break;
    case CMD_REINICIAR: reinicioSolicitado = true; break;
  }
  Serial.printf("[DL] Comando 0x%02X (%d bytes)\n", d[0], n);
}

/* -----------------------------------------------------------------------------------------
 * 13. SETUP Y LOOP
 * ----------------------------------------------------------------------------------------- */
/** @brief Indica si hay un reporte OTA, un evento HMI o una medición forzada pendiente. */
bool hayUplinkUrgente() { return reporteOTAPendiente || nColaHMI > 0 || phForzado; }

/** @brief Inicializa pines, UART del HMI, revisión post-OTA, ADC y la pila LoRaWAN. */
void setup() {
  Serial.begin(115200);
  delay(500);
  pinMode(MOSFET_PIN, OUTPUT); digitalWrite(MOSFET_PIN, LOW);
  pinMode(LED_PIN, OUTPUT); escribirLED(true);
#if HMI_HABILITADO
  Serial1.begin(HMI_BAUD, SERIAL_8N1, HMI_RX_PIN, HMI_TX_PIN);
#endif

  const esp_partition_t* corriendo = esp_ota_get_running_partition();
  Serial.printf("\n=== HELMO %s | %s v%d.%d.%d | partición %s ===\n", NODO_ALIAS, FW_PRODUCTO,
                FW_MAJOR, FW_MINOR, FW_PATCH, corriendo ? corriendo->label : "?");

  revisarArranqueTrasOTA();

  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PH_PIN, ADC_11db);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  deviceState = DEVICE_STATE_INIT;
}

/** @brief Máquina de estados LoRaWAN de la librería Heltec + tareas no bloqueantes (LED, HMI, OTA). */
void loop() {
  actualizarLED();
  atenderHMI();

  if (enPrueba && millis() - pruebaInicioMs > PRUEBA_TIMEOUT_MS) revertirFirmware("No se unió a la red a tiempo");

  bool radioLibre = (deviceState == DEVICE_STATE_SLEEP) && (millis() - ultimoUplinkMs > 8000);

  if (reinicioSolicitado && radioLibre) { Serial.println("[CMD] Reinicio remoto"); delay(200); ESP.restart(); }
  if (ordenOTA.pendiente && radioLibre && unido) procesarOrdenOTA();

  // Envío anticipado de eventos urgentes respetando la separación mínima
  if (unido && deviceState == DEVICE_STATE_SLEEP && hayUplinkUrgente() &&
      millis() - ultimoUplinkMs >= SEPARACION_MIN_TX_MS) {
    deviceState = DEVICE_STATE_SEND;
  }

  switch (deviceState) {
    case DEVICE_STATE_INIT:
      LoRaWAN.init(loraWanClass, loraWanRegion);
      break;

    case DEVICE_STATE_JOIN:
      LoRaWAN.join();
      break;

    case DEVICE_STATE_SEND:
      if (!unido) {
        unido = true;   // la librería solo llega aquí tras el Join Accept
        Serial.println("[LoRa] Unido a la red");
        if (enPrueba) confirmarFirmware();
      }
      if (reporteOTAPendiente)  prepararUplinkOTA();
      else if (nColaHMI > 0)    prepararUplinkHMI();
      else                      { phForzado = false; prepararUplinkPH(); }
      LoRaWAN.send();
      ultimoUplinkMs = millis();
      deviceState = DEVICE_STATE_CYCLE;
      break;

    case DEVICE_STATE_CYCLE:
      txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
      LoRaWAN.cycle(txDutyCycleTime);
      deviceState = DEVICE_STATE_SLEEP;
      break;

    case DEVICE_STATE_SLEEP:
#if HMI_HABILITADO
      Radio.IrqProcess();             // la MCU queda despierta para atender el UART del HMI
#else
      LoRaWAN.sleep(loraWanClass);    // sin HMI: bajo consumo
#endif
      break;

    default:
      deviceState = DEVICE_STATE_INIT;
      break;
  }
}
