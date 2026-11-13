#!/usr/bin/env python3
"""!
@file firmar_firmware.py
@brief HELMO - Firma y publicación de firmware para la OTA híbrida.

  llaves     Genera el par de llaves ECDSA P-256 (una sola vez) y el clave_publica_ota.h del nodo.
  firmar     Firma un .bin, lo copia al servidor de distribución y escribe manifest.json.
  verificar  Comprueba un manifiesto y su imagen con la llave pública (lo mismo que hace el nodo).

Mensaje firmado (idéntico en el firmware):
    HELMO-OTA|<producto>|<version>|<tamano>|<sha256 hex en minúsculas>

Requisito: pip install cryptography
"""
import argparse
import base64
import hashlib
import json
import os
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import ec

PRODUCTO_DEFECTO = "helmo-ph-node"
CANAL_DEFECTO = "ph-node"


def mensaje_canonico(producto: str, version: str, tamano: int, sha256_hex: str) -> bytes:
    """! @brief Texto exacto que se firma; debe coincidir con el que arma el nodo.
    @return bytes "HELMO-OTA|producto|versión|tamaño|sha256"."""
    return f"HELMO-OTA|{producto}|{version}|{tamano}|{sha256_hex.lower()}".encode()


def version_tupla(v: str):
    """! @brief Convierte "M.m.p" en una tupla comparable.
    @exception ValueError si el formato no es válido."""
    partes = v.split(".")
    if len(partes) != 3 or not all(p.isdigit() and 0 <= int(p) <= 255 for p in partes):
        raise ValueError(f"versión inválida '{v}': usa M.m.p con valores 0-255")
    return tuple(int(p) for p in partes)


def header_c(pem: str) -> str:
    """! @brief Genera el contenido de clave_publica_ota.h a partir de la clave PEM."""
    lineas = "".join(f'  "{l}\\n"\n' for l in pem.strip().splitlines())
    return (
        "/**\n * @file clave_publica_ota.h\n"
        " * @brief Clave pública ECDSA P-256 con la que el nodo verifica las actualizaciones.\n"
        " * @details Generado por firmar_firmware.py. La clave pública NO es secreta.\n */\n"
        "#pragma once\n\n/** @brief Clave pública en formato PEM (SubjectPublicKeyInfo). */\n"
        "static const char OTA_CLAVE_PUBLICA_PEM[] =\n" + lineas.rstrip("\n") + ";\n"
    )


def cmd_llaves(a):
    """! @brief Crea el par de llaves ECDSA P-256 (una sola vez) y, si se pide, el header del nodo."""
    d = Path(a.dir)
    d.mkdir(parents=True, exist_ok=True)
    priv_path, pub_path = d / "ota_privada.pem", d / "ota_publica.pem"
    if priv_path.exists() and not a.forzar:
        sys.exit(f"Ya existe {priv_path}. Usa --forzar solo si quieres REEMPLAZARLA "
                 "(los nodos actuales dejarían de aceptar firmware nuevo).")
    llave = ec.generate_private_key(ec.SECP256R1())
    priv_path.write_bytes(llave.private_bytes(
        serialization.Encoding.PEM, serialization.PrivateFormat.PKCS8, serialization.NoEncryption()))
    os.chmod(priv_path, 0o600)
    pub_pem = llave.public_key().public_bytes(
        serialization.Encoding.PEM, serialization.PublicFormat.SubjectPublicKeyInfo).decode()
    pub_path.write_text(pub_pem)
    print(f"Llave privada: {priv_path} (permisos 600, NO la compartas ni la subas a git)")
    print(f"Llave pública: {pub_path}")
    if a.header:
        Path(a.header).write_text(header_c(pub_pem))
        print(f"Header para el nodo: {a.header}")
    else:
        print("\n--- Pega esto en firmware/helmo_nodo_ph/clave_publica_ota.h ---\n")
        print(header_c(pub_pem))


def cmd_firmar(a):
    """! @brief Firma un .bin, lo publica en el servidor de distribución y escribe manifest.json.
    @details Rechaza imágenes que no son de aplicación ESP32 y versiones no mayores a la publicada."""
    binario = Path(a.bin).read_bytes()
    if not binario or binario[0] != 0xE9:
        sys.exit("El archivo no parece una imagen de aplicación ESP32 (byte mágico 0xE9). "
                 "Usa el .ino.bin de 'Sketch > Export Compiled Binary', no el .merged.bin.")
    version_tupla(a.version)

    destino = Path(a.destino) / a.canal
    destino.mkdir(parents=True, exist_ok=True)
    manifiesto_path = destino / "manifest.json"

    if manifiesto_path.exists() and not a.forzar:
        anterior = json.loads(manifiesto_path.read_text())
        if version_tupla(a.version) <= version_tupla(anterior["version"]):
            sys.exit(f"La versión {a.version} no es mayor que la publicada ({anterior['version']}). "
                     "Los nodos la rechazarían (anti-downgrade).")

    llave = serialization.load_pem_private_key(Path(a.privada).read_bytes(), password=None)
    sha = hashlib.sha256(binario).hexdigest()
    firma = llave.sign(mensaje_canonico(a.producto, a.version, len(binario), sha),
                       ec.ECDSA(hashes.SHA256()))

    archivo = f"{a.producto}-{a.version}.bin"
    shutil.copyfile(a.bin, destino / archivo)
    manifiesto = {
        "producto": a.producto,
        "version": a.version,
        "archivo": archivo,
        "tamano": len(binario),
        "sha256": sha,
        "firma": base64.b64encode(firma).decode(),
        "publicado": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "notas": a.notas or "",
    }
    texto = json.dumps(manifiesto, indent=2, ensure_ascii=False)
    (destino / f"manifest-{a.version}.json").write_text(texto)   # histórico
    manifiesto_path.write_text(texto)
    print(f"Publicado {archivo} ({len(binario)} bytes)\nSHA-256 {sha}\nManifiesto: {manifiesto_path}")


def cmd_verificar(a):
    """! @brief Comprueba tamaño, SHA-256 y firma de un manifiesto, igual que lo hace el nodo."""
    m = json.loads(Path(a.manifiesto).read_text())
    imagen = Path(a.manifiesto).parent / m["archivo"]
    datos = imagen.read_bytes()
    pub = serialization.load_pem_public_key(Path(a.publica).read_bytes())
    sha = hashlib.sha256(datos).hexdigest()
    ok = True
    if len(datos) != m["tamano"]:
        print("✗ El tamaño no coincide"); ok = False
    if sha != m["sha256"].lower():
        print("✗ El SHA-256 de la imagen no coincide con el manifiesto"); ok = False
    try:
        pub.verify(base64.b64decode(m["firma"]),
                   mensaje_canonico(m["producto"], m["version"], m["tamano"], m["sha256"]),
                   ec.ECDSA(hashes.SHA256()))
        print("✓ Firma válida")
    except InvalidSignature:
        print("✗ FIRMA INVÁLIDA"); ok = False
    print(f"{'✓' if ok else '✗'} {m['producto']} v{m['version']} ({m['tamano']} bytes)")
    sys.exit(0 if ok else 1)


def main():
    """! @brief Punto de entrada de la línea de comandos."""
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("llaves", help="generar el par de llaves")
    s.add_argument("--dir", default="llaves")
    s.add_argument("--header", help="ruta de clave_publica_ota.h a escribir")
    s.add_argument("--forzar", action="store_true")
    s.set_defaults(fn=cmd_llaves)

    s = sub.add_parser("firmar", help="firmar y publicar un .bin")
    s.add_argument("--bin", required=True)
    s.add_argument("--version", required=True, help="M.m.p, igual a FW_MAJOR.FW_MINOR.FW_PATCH del .ino")
    s.add_argument("--privada", default="llaves/ota_privada.pem")
    s.add_argument("--destino", default="/opt/helmo/ota")
    s.add_argument("--producto", default=PRODUCTO_DEFECTO)
    s.add_argument("--canal", default=CANAL_DEFECTO)
    s.add_argument("--notas")
    s.add_argument("--forzar", action="store_true", help="permite publicar una versión no mayor")
    s.set_defaults(fn=cmd_firmar)

    s = sub.add_parser("verificar", help="verificar un manifiesto publicado")
    s.add_argument("--manifiesto", required=True)
    s.add_argument("--publica", default="llaves/ota_publica.pem")
    s.set_defaults(fn=cmd_verificar)

    a = p.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
