/**
 * @file clave_publica_ota.h
 * @brief Clave pública ECDSA P-256 con la que el nodo verifica las actualizaciones.
 * @details La reemplaza `firmar_firmware.py llaves --header ...`. La clave pública NO es secreta.
 *          Mientras tenga este marcador, el nodo rechaza toda actualización (falla segura).
 */
#pragma once
#warning "clave_publica_ota.h aún tiene el marcador: genera las llaves con firmar_firmware.py"

/** @brief Clave pública en formato PEM (SubjectPublicKeyInfo). */
static const char OTA_CLAVE_PUBLICA_PEM[] =
  "-----BEGIN PUBLIC KEY-----\n"
  "GENERA-LAS-LLAVES-CON-firmar_firmware.py\n"
  "-----END PUBLIC KEY-----\n";
