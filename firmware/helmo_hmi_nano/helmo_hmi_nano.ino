/**
 * @file helmo_hmi_nano.ino
 * @brief HELMO · Firmware 2 de 2: módulo HMI de voz y gestos para el nodo de pH.
 * @author Esteban Eduardo Escarraga
 * @version 1.0.0
 * @date 2026-11
 *
 * @details
 * - **Hardware:** Arduino Nano 33 BLE Sense (Rev1 o Rev2), conectado por UART al nodo Heltec V3.
 * - **Entorno:** Arduino IDE, placa "Arduino Nano 33 BLE" (paquete Arduino Mbed OS Nano Boards).
 *
 * @par Funciones principales
 * -# **Voz:** reconocimiento continuo con el modelo de Edge Impulse
 *    `speech_recognition_Esteban` (Amarillo, Rojo, Verde).
 * -# **Gestos:** clasificador de movimiento con el IMU (punch, flex). Usa el TensorFlow Lite Micro
 *    incluido en la librería de Edge Impulse para no tener dos copias de TensorFlow.
 * -# Envía al nodo cada detección confiable; el nodo decide la acción y si la reporta a TTN.
 * -# Muestra el pH recibido como **semáforo** en el LED RGB y cada detección con su color durante 1,5 s.
 *
 * @par Semáforo del LED RGB
 * verde = normal · amarillo = cerca del límite · rojo parpadeando = alarma sin reconocer ·
 * rojo fijo = alarma reconocida · azul parpadeando = OTA en curso · azul tenue = esperando al nodo.
 *
 * @par Conexión (lógica de 3,3 V en ambas placas)
 * @verbatim
  Nano D1 (TX) -> Heltec GPIO 47 (RX)
  Nano D0 (RX) <- Heltec GPIO 48 (TX)
  GND          -- GND
 @endverbatim
 *
 * @par Protocolo UART (texto, 115200 baud, una línea por mensaje)
 * @verbatim
  Nano -> Heltec:  HOLA,<ver> | PING | EVT,V,<etiqueta>,<conf%> | EVT,G,<etiqueta>,<conf%>
  Heltec -> Nano:  PH,<pH x100>,<estado>,<reconocida> | OTA,<1|0> | ACK,<etiqueta>
 @endverbatim
 *
 * @pre Instalar el .zip de Edge Impulse y copiar el `model.h` de gestos con el array
 *      renombrado a `modelo_gestos`. Ajustar IMU_REV2 según la placa.
 * @see helmo_nodo_ph.ino
 */

// --- Edge Impulse: audio continuo ---
#define EIDSP_QUANTIZE_FILTERBANK 0
#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW 4
#include <PDM.h>
#include <speech_recognition_Esteban_inferencing.h>

// --- IMU: 0 = Rev1 (LSM9DS1) | 1 = Rev2 (BMI270/BMM150) ---
#define IMU_REV2 0
#if IMU_REV2
  #include <Arduino_BMI270_BMM150.h>
#else
  #include <Arduino_LSM9DS1.h>
#endif

// --- TensorFlow Lite Micro incluido en la librería de Edge Impulse ---
#include "edge-impulse-sdk/tensorflow/lite/micro/micro_interpreter.h"
#include "edge-impulse-sdk/tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "edge-impulse-sdk/tensorflow/lite/schema/schema_generated.h"
#include "model.h"   // modelo de gestos (array renombrado a modelo_gestos)

#define HMI_VERSION "1.0.0"
#define UART_BAUD 115200

/* -----------------------------------------------------------------------------------------
 * 1. AJUSTES DE DETECCIÓN
 * ----------------------------------------------------------------------------------------- */
const float UMBRAL_VOZ = 0.80;                 // probabilidad mínima para aceptar una palabra
const float UMBRAL_GESTO = 0.85;
const unsigned long ENFRIAMIENTO_MS = 2000;    // pausa tras cada evento (evita repeticiones)
const char* ETIQUETAS_IGNORADAS[] = { "ruido", "noise", "desconocido", "unknown", "_noise", "_unknown" };

// Gestos: MISMO ORDEN que la lista GESTURES del notebook de Colab
const char* GESTOS[] = { "punch", "flex" };
#define NUM_GESTOS (sizeof(GESTOS) / sizeof(GESTOS[0]))

const float UMBRAL_MOVIMIENTO_G = 2.5;
const int MUESTRAS_GESTO = 119;

/* -----------------------------------------------------------------------------------------
 * 2. LED RGB (activo en BAJO)
 * ----------------------------------------------------------------------------------------- */
/** @brief Colores disponibles en el LED RGB. */
enum Color : uint8_t { APAGADO, ROJO, VERDE, AZUL, AMARILLO, CIAN, MAGENTA, BLANCO };

/** @brief Enciende el LED RGB (activo en bajo) con el color indicado. */
void ponerColor(Color c) {
  bool r = (c == ROJO || c == AMARILLO || c == MAGENTA || c == BLANCO);
  bool g = (c == VERDE || c == AMARILLO || c == CIAN || c == BLANCO);
  bool b = (c == AZUL || c == CIAN || c == MAGENTA || c == BLANCO);
  digitalWrite(LEDR, r ? LOW : HIGH);
  digitalWrite(LEDG, g ? LOW : HIGH);
  digitalWrite(LEDB, b ? LOW : HIGH);
}

/** @brief Color con el que se muestra cada palabra o gesto detectado. */
Color colorDeEtiqueta(const char* e) {
  if (!strcasecmp(e, "Rojo")) return ROJO;
  if (!strcasecmp(e, "Verde")) return VERDE;
  if (!strcasecmp(e, "Amarillo")) return AMARILLO;
  if (!strcasecmp(e, "punch")) return CIAN;
  if (!strcasecmp(e, "flex")) return MAGENTA;
  return BLANCO;
}

// Estado recibido del nodo
int phX100 = -1;
uint8_t estadoPH = 0;
bool alertaReconocida = false;
bool otaEnNodo = false;
unsigned long ultimoMensajeNodoMs = 0;

// Retroalimentación temporal de una detección
Color colorFeedback = APAGADO;
unsigned long feedbackHastaMs = 0;

/** @brief Muestra un color temporal que tiene prioridad sobre el semáforo. @param c color @param ms duración. */
void mostrarFeedback(Color c, unsigned long ms = 1500) { colorFeedback = c; feedbackHastaMs = millis() + ms; }

/** @brief Decide el color del LED: retroalimentación, OTA, espera o semáforo de pH. */
void actualizarLED() {
  unsigned long t = millis();
  if ((long)(feedbackHastaMs - t) > 0) { ponerColor(colorFeedback); return; }
  if (otaEnNodo) { ponerColor((t / 150) % 2 ? AZUL : APAGADO); return; }
  bool nodoVivo = ultimoMensajeNodoMs && (t - ultimoMensajeNodoMs < 15UL * 60UL * 1000UL);
  if (phX100 < 0 || !nodoVivo) { ponerColor((t % 2000) < 80 ? AZUL : APAGADO); return; }
  switch (estadoPH) {
    case 0:  ponerColor(VERDE); break;
    case 1:  ponerColor(AMARILLO); break;
    default: ponerColor(alertaReconocida ? ROJO : ((t / 300) % 2 ? ROJO : APAGADO)); break;
  }
}

/* -----------------------------------------------------------------------------------------
 * 3. ENLACE UART CON EL NODO DE pH
 * ----------------------------------------------------------------------------------------- */
unsigned long ultimoEventoMs = 0;

/**
 * @brief Envía una detección al nodo por UART con el formato EVT,tipo,etiqueta,confianza.
 * @param tipo 'V' voz o 'G' gesto
 * @param etiqueta clase detectada
 * @param confianza probabilidad entre 0 y 1
 */
void enviarEvento(char tipo, const char* etiqueta, float confianza) {
  int pct = (int)(confianza * 100.0 + 0.5);
  Serial1.print("EVT,"); Serial1.print(tipo); Serial1.print(',');
  Serial1.print(etiqueta); Serial1.print(','); Serial1.print(pct); Serial1.print('\n');
  Serial.print(">>> "); Serial.print(tipo == 'V' ? "VOZ " : "GESTO ");
  Serial.print(etiqueta); Serial.print(" ("); Serial.print(pct); Serial.println("%) enviado al nodo");
  ultimoEventoMs = millis();
  mostrarFeedback(colorDeEtiqueta(etiqueta));
}

/** @brief Interpreta los mensajes del nodo (PH, OTA, ACK). @param linea mensaje sin salto de línea. */
void procesarLineaNodo(char* linea) {
  ultimoMensajeNodoMs = millis();
  char* c[4] = { nullptr };
  uint8_t n = 0;
  for (char* t = strtok(linea, ","); t && n < 4; t = strtok(nullptr, ",")) c[n++] = t;
  if (n == 0) return;

  if (!strcmp(c[0], "PH") && n >= 4) {
    phX100 = atoi(c[1]);
    estadoPH = atoi(c[2]);
    alertaReconocida = atoi(c[3]) == 1;
    Serial.print("[NODO] pH "); Serial.print(phX100 / 100.0, 2);
    Serial.print(" | estado "); Serial.println(estadoPH);
  } else if (!strcmp(c[0], "OTA") && n >= 2) {
    otaEnNodo = atoi(c[1]) == 1;
    Serial.println(otaEnNodo ? "[NODO] Actualizando firmware..." : "[NODO] OTA terminada");
  } else if (!strcmp(c[0], "ACK") && n >= 2) {
    Serial.print("[NODO] Confirmó "); Serial.println(c[1]);
  }
}

/** @brief Lee el UART sin bloquear y envía un PING (o HOLA si aún no hay pH) cada 10 s. */
void atenderUART() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial1.available()) {
    char ch = Serial1.read();
    if (ch == '\n') { buf[len] = 0; if (len) procesarLineaNodo(buf); len = 0; }
    else if (ch != '\r' && len < sizeof(buf) - 1) buf[len++] = ch;
  }
  static unsigned long ultimoPing = 0;
  if (millis() - ultimoPing > 10000) {
    ultimoPing = millis();
    // mientras no tenga un pH, se vuelve a presentar para que el nodo le envíe el último valor
    Serial1.print(phX100 < 0 ? "HOLA," HMI_VERSION "\n" : "PING\n");
  }
}

/* -----------------------------------------------------------------------------------------
 * 4. VOZ (Edge Impulse, audio continuo con doble buffer)
 * ----------------------------------------------------------------------------------------- */
/** @brief Doble buffer de audio para la inferencia continua de Edge Impulse. */
typedef struct {
  signed short* buffers[2];
  volatile unsigned char buf_select;
  volatile unsigned char buf_ready;
  volatile unsigned int buf_count;
  unsigned int n_samples;
} inference_t;

static inference_t inference;
static volatile bool record_ready = false;
static signed short* sampleBuffer;
static int print_results = -(EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW);

/** @brief Callback del micrófono PDM: copia muestras al buffer activo y alterna al llenarse. */
static void pdm_data_ready_inference_callback(void) {
  int bytesAvailable = PDM.available();
  int bytesRead = PDM.read((char*)&sampleBuffer[0], bytesAvailable);
  if (!record_ready) return;
  for (int i = 0; i < (bytesRead >> 1); i++) {
    inference.buffers[inference.buf_select][inference.buf_count++] = sampleBuffer[i];
    if (inference.buf_count >= inference.n_samples) {
      inference.buf_select ^= 1;
      inference.buf_count = 0;
      inference.buf_ready = 1;
    }
  }
}

/** @brief Reserva los buffers de audio y arranca el micrófono a la frecuencia del modelo. */
static bool microphone_inference_start(uint32_t n_samples) {
  inference.buffers[0] = (signed short*)malloc(n_samples * sizeof(signed short));
  inference.buffers[1] = (signed short*)malloc(n_samples * sizeof(signed short));
  sampleBuffer = (signed short*)malloc((n_samples >> 1) * sizeof(signed short));
  if (!inference.buffers[0] || !inference.buffers[1] || !sampleBuffer) return false;

  inference.buf_select = 0;
  inference.buf_count = 0;
  inference.n_samples = n_samples;
  inference.buf_ready = 0;

  PDM.onReceive(&pdm_data_ready_inference_callback);
  PDM.setBufferSize((n_samples >> 1) * sizeof(int16_t));
  if (!PDM.begin(1, EI_CLASSIFIER_FREQUENCY)) { Serial.println("ERR: no arrancó el micrófono PDM"); return false; }
  PDM.setGain(127);
  record_ready = true;
  return true;
}

/** @brief Entrega al clasificador las muestras del buffer completo convertidas a float. */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float* out_ptr) {
  numpy::int16_to_float(&inference.buffers[inference.buf_select ^ 1][offset], out_ptr, length);
  return 0;
}

/** @brief Indica si la etiqueta es ruido o desconocido y no debe generar eventos. */
bool etiquetaIgnorada(const char* e) {
  for (auto ig : ETIQUETAS_IGNORADAS) if (!strcasecmp(e, ig)) return true;
  return false;
}

/** @brief Clasifica el último trozo de audio y envía la palabra si supera UMBRAL_VOZ. */
void procesarAudio() {
  signal_t signal;
  signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
  signal.get_data = &microphone_audio_signal_get_data;
  ei_impulse_result_t result = { 0 };

  if (run_classifier_continuous(&signal, &result, false) != EI_IMPULSE_OK) return;

  size_t mejor = 0;
  for (size_t i = 1; i < EI_CLASSIFIER_LABEL_COUNT; i++)
    if (result.classification[i].value > result.classification[mejor].value) mejor = i;

  if (++print_results >= EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW) {
    Serial.print("[VOZ]");
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      Serial.print(' '); Serial.print(result.classification[i].label);
      Serial.print('='); Serial.print(result.classification[i].value, 2);
    }
    Serial.println();
    print_results = 0;
  }

  const char* etiqueta = result.classification[mejor].label;
  float conf = result.classification[mejor].value;
  if (conf >= UMBRAL_VOZ && !etiquetaIgnorada(etiqueta) && millis() - ultimoEventoMs > ENFRIAMIENTO_MS) {
    enviarEvento('V', etiqueta, conf);
  }
}

/* -----------------------------------------------------------------------------------------
 * 5. GESTOS (TFLite Micro + IMU, sin bloquear el audio)
 * ----------------------------------------------------------------------------------------- */
constexpr int TAM_ARENA_GESTOS = 8 * 1024;
alignas(16) static uint8_t arenaGestos[TAM_ARENA_GESTOS];
static tflite::MicroMutableOpResolver<4> resolverGestos;
static tflite::MicroInterpreter* interpreteGestos = nullptr;
static TfLiteTensor* entradaGestos = nullptr;
static TfLiteTensor* salidaGestos = nullptr;
bool gestosListos = false;
int muestrasLeidas = MUESTRAS_GESTO;   // == MUESTRAS_GESTO -> esperando movimiento

/** @brief Carga el modelo de gestos en TFLite Micro y valida su entrada (119 x 6 floats). */
bool iniciarGestos() {
  const tflite::Model* modelo = tflite::GetModel(modelo_gestos);
  if (modelo->version() != 3) { Serial.println("ERR: versión de esquema del modelo de gestos"); return false; }

  resolverGestos.AddFullyConnected();
  resolverGestos.AddSoftmax();
  resolverGestos.AddRelu();
  resolverGestos.AddReshape();

  static tflite::MicroInterpreter interprete(modelo, resolverGestos, arenaGestos, TAM_ARENA_GESTOS);
  interpreteGestos = &interprete;
  if (interpreteGestos->AllocateTensors() != kTfLiteOk) {
    Serial.println("ERR: AllocateTensors (sube TAM_ARENA_GESTOS a 16 o 32 KB)");
    return false;
  }
  entradaGestos = interpreteGestos->input(0);
  salidaGestos = interpreteGestos->output(0);
  if (entradaGestos->type != kTfLiteFloat32 || entradaGestos->bytes != MUESTRAS_GESTO * 6 * sizeof(float)) {
    Serial.println("ERR: el modelo de gestos no espera 119x6 floats");
    return false;
  }
  return true;
}

/** @brief Ejecuta el modelo de gestos y envía el gesto si supera UMBRAL_GESTO. */
void clasificarGesto() {
  if (interpreteGestos->Invoke() != kTfLiteOk) { Serial.println("ERR: Invoke gestos"); return; }
  size_t mejor = 0;
  Serial.print("[GESTO]");
  for (size_t i = 0; i < NUM_GESTOS; i++) {
    float v = salidaGestos->data.f[i];
    Serial.print(' '); Serial.print(GESTOS[i]); Serial.print('='); Serial.print(v, 2);
    if (v > salidaGestos->data.f[mejor]) mejor = i;
  }
  Serial.println();
  float conf = salidaGestos->data.f[mejor];
  if (conf >= UMBRAL_GESTO && millis() - ultimoEventoMs > ENFRIAMIENTO_MS) enviarEvento('G', GESTOS[mejor], conf);
}

/** @brief Espera un movimiento fuerte, toma 119 muestras normalizadas y clasifica, sin bloquear el audio. */
void atenderIMU() {
  if (!gestosListos) return;
  float aX, aY, aZ, gX, gY, gZ;

  if (muestrasLeidas == MUESTRAS_GESTO) {               // esperando un movimiento fuerte
    if (IMU.accelerationAvailable()) {
      IMU.readAcceleration(aX, aY, aZ);
      if (fabs(aX) + fabs(aY) + fabs(aZ) >= UMBRAL_MOVIMIENTO_G && millis() - ultimoEventoMs > ENFRIAMIENTO_MS)
        muestrasLeidas = 0;
    }
    return;
  }

  while (muestrasLeidas < MUESTRAS_GESTO && IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    IMU.readAcceleration(aX, aY, aZ);
    IMU.readGyroscope(gX, gY, gZ);
    float* d = &entradaGestos->data.f[muestrasLeidas * 6];   // misma normalización del entrenamiento
    d[0] = (aX + 4.0) / 8.0;    d[1] = (aY + 4.0) / 8.0;    d[2] = (aZ + 4.0) / 8.0;
    d[3] = (gX + 2000.0) / 4000.0; d[4] = (gY + 2000.0) / 4000.0; d[5] = (gZ + 2000.0) / 4000.0;
    muestrasLeidas++;
  }
  if (muestrasLeidas == MUESTRAS_GESTO) clasificarGesto();
}

/* -----------------------------------------------------------------------------------------
 * 6. SETUP Y LOOP
 * ----------------------------------------------------------------------------------------- */
/** @brief Inicializa LED, puertos serie, IMU, modelo de gestos, clasificador de voz y micrófono. */
void setup() {
  pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
  ponerColor(BLANCO);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000);   // no se queda esperando si no hay USB
  Serial1.begin(UART_BAUD);

  Serial.println("\n=== HELMO HMI voz + gestos v" HMI_VERSION " ===");
  Serial.print("Voz: "); Serial.print(EI_CLASSIFIER_LABEL_COUNT); Serial.println(" clases (Edge Impulse)");

  if (!IMU.begin()) Serial.println("ERR: IMU no inicializó (¿IMU_REV2 correcto?). Gestos desactivados.");
  else gestosListos = iniciarGestos();
  Serial.println(gestosListos ? "Gestos: listos" : "Gestos: DESACTIVADOS");

  run_classifier_init();
  if (!microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE))
    Serial.println("ERR: sin memoria para el buffer de audio");

  Serial1.print("HOLA," HMI_VERSION "\n");
  ponerColor(APAGADO);
}

/** @brief Atiende el UART, procesa audio cuando hay un buffer listo, lee el IMU y actualiza el LED. */
void loop() {
  atenderUART();
  if (inference.buf_ready) {      // llegó un nuevo trozo de audio (cada 250 ms)
    inference.buf_ready = 0;
    procesarAudio();
  }
  atenderIMU();
  actualizarLED();
}
