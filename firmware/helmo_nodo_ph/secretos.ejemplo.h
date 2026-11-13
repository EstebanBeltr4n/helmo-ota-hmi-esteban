/**
 * @file secretos.ejemplo.h
 * @brief Plantilla de credenciales del nodo de pH (sin valores reales).
 *
 * @details Copia este archivo como `secretos.h` en la misma carpeta y llénalo con tus valores.
 * `secretos.h` está en `.gitignore` y excluido del Doxyfile: **nunca** se publica ni se documenta.
 *
 * @warning No escribas llaves reales en esta plantilla.
 */
#pragma once
#include <stdint.h>

/** @name LoRaWAN OTAA
 *  Los mismos valores del dispositivo en la consola de TTN, en orden MSB. @{ */
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  ///< DevEUI
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  ///< JoinEUI (AppEUI)
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  ///< AppKey (secreta)
/** @} */

/** @name Wi-Fi de mantenimiento
 *  Solo se enciende cuando llega una orden OTA. Red 1: hotspot del gateway.
 *  Red 2: hotspot del técnico (deja "" si no la usas). @{ */
#define WIFI_MANT_SSID_1 "NOMBRE-RED-1"   ///< SSID principal
#define WIFI_MANT_PASS_1 "CLAVE-RED-1"    ///< clave principal
#define WIFI_MANT_SSID_2 ""               ///< SSID de respaldo
#define WIFI_MANT_PASS_2 ""               ///< clave de respaldo
/** @} */

/**
 * @brief URL base del servidor de distribución OTA, sin barra al final.
 * @details Campo: `https://TU-DOMINIO.ngrok-free.app/ota` · Laboratorio: `http://IP-DEL-DEBIAN:8088/ota`.
 * Las imágenes van firmadas: el nodo solo acepta firmware firmado con la llave privada de HELMO.
 */
#define OTA_BASE_URL "http://IP-DEL-DEBIAN:8088/ota"
