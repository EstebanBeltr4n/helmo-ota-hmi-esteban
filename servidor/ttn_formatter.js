/**
 * @file ttn_formatter.js
 * @brief Formateador de payload para TTN (Applications > Payload formatters > Custom JavaScript).
 * @details Nodo de pH v2 + OTA híbrida + HMI de voz y gestos. Compatible con el payload de 6 bytes anterior.
 */

var ESTADOS_PH = ["normal", "aviso", "alarma", "desconocido"];
var VOZ = { 1: "Rojo", 2: "Amarillo", 3: "Verde" };
var GESTO = { 1: "punch", 2: "flex" };
var ACCION = { 1: "incidente", 2: "medir", 3: "reconocer_alarma", 4: "mostrar_estado" };
var RESULTADO_OTA = { 1: "EXITO", 2: "ERROR", 3: "ROLLBACK", 4: "YA_AL_DIA" };
var ERROR_OTA = {
  0: "ninguno", 1: "wifi", 2: "manifiesto_http", 3: "manifiesto_json", 4: "producto",
  5: "version", 6: "firma_invalida", 7: "descarga_http", 8: "tamano", 9: "interrumpida",
  10: "flash", 11: "sha256", 12: "imagen_rechazada", 13: "ocupado_en_prueba"
};

/** @brief Lee un entero de 16 bits big-endian. */
function u16(b, i) { return (b[i] << 8) | b[i + 1]; }

/**
 * @brief Decodifica los uplinks de los puertos 3 (pH), 4 (HMI), 5 (OTA) y 21 (FUOTA de laboratorio).
 * @param input objeto de TTN con fPort y bytes
 * @return {data} con los campos decodificados, o {errors}
 */
function decodeUplink(input) {
  var b = input.bytes;
  switch (input.fPort) {
    case 3: {
      if (b.length < 6) return { errors: ["pH: payload corto"] };
      var d = { tipo: "lectura_ph", node_id: b[0], ph: u16(b, 1) / 100, voltaje: u16(b, 3) / 1000 };
      if (b.length >= 9) {
        d.fw_version = b[5] + "." + b[6] + "." + b[7];
        d.estado_ph = ESTADOS_PH[b[8] & 0x03];
        d.hmi_conectado = (b[8] & 0x04) !== 0;
        d.fw_en_prueba = (b[8] & 0x08) !== 0;
      } else {
        d.fw_version = b[5] + ".0.0";   // firmware anterior (6 bytes)
      }
      return { data: d };
    }
    case 4: {
      if (b.length < 5) return { errors: ["HMI: payload corto"] };
      var esVoz = b[1] === 1;
      return { data: {
        tipo: "evento_hmi", node_id: b[0], fuente: esVoz ? "voz" : "gesto",
        etiqueta: (esVoz ? VOZ : GESTO)[b[2]] || ("codigo_" + b[2]),
        confianza: b[3] / 100, accion: ACCION[b[4]] || ("accion_" + b[4])
      } };
    }
    case 5: {
      if (b.length < 7) return { errors: ["OTA: payload corto"] };
      return { data: {
        tipo: "reporte_ota", node_id: b[0], trabajo: b[1],
        resultado: RESULTADO_OTA[b[2]] || ("codigo_" + b[2]),
        fw_version: b[3] + "." + b[4] + "." + b[5],
        detalle: ERROR_OTA[b[6]] || ("error_" + b[6])
      } };
    }
    case 21: // reportes del protocolo FUOTA de laboratorio (ChirpStack), solo para compatibilidad
      return { data: { tipo: "fuota_lab", bytes: b } };
    default:
      return { warnings: ["puerto sin decodificador: " + input.fPort], data: { bytes: b } };
  }
}

// Downlink desde la consola de TTN, puerto 10. Ejemplos de JSON:
//   {"comando":"ota","trabajo":7,"version":"2.1.0"}
//   {"comando":"intervalo","minutos":10}
//   {"comando":"medir"}     {"comando":"reiniciar"}
/** @brief Convierte un comando JSON (ota, intervalo, medir, reiniciar) en bytes del puerto 10. */
function encodeDownlink(input) {
  var d = input.data || {};
  if (d.comando === "ota") {
    var v = String(d.version || "").split(".");
    if (v.length !== 3) return { errors: ["version debe ser M.m.p"] };
    return { fPort: 10, bytes: [0x01, (d.trabajo || 1) & 0xFF, +v[0] & 0xFF, +v[1] & 0xFF, +v[2] & 0xFF] };
  }
  if (d.comando === "intervalo") {
    var m = d.minutos | 0;
    if (m < 1 || m > 1440) return { errors: ["minutos entre 1 y 1440"] };
    return { fPort: 10, bytes: [0x02, (m >> 8) & 0xFF, m & 0xFF] };
  }
  if (d.comando === "medir") return { fPort: 10, bytes: [0x03] };
  if (d.comando === "reiniciar") return { fPort: 10, bytes: [0x04] };
  return { errors: ["comando desconocido"] };
}

/** @brief Muestra en la consola de TTN el comando de un downlink del puerto 10. */
function decodeDownlink(input) {
  var b = input.bytes;
  switch (b[0]) {
    case 0x01: return { data: { comando: "ota", trabajo: b[1], version: b[2] + "." + b[3] + "." + b[4] } };
    case 0x02: return { data: { comando: "intervalo", minutos: u16(b, 1) } };
    case 0x03: return { data: { comando: "medir" } };
    case 0x04: return { data: { comando: "reiniciar" } };
    default:   return { errors: ["comando desconocido"] };
  }
}
