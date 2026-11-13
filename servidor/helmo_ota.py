#!/usr/bin/env python3
"""!
@file helmo_ota.py
@brief HELMO - Orquestador OTA y eventos HMI (servidor de aplicación Debian 12).

Corre al lado de helmo-bridge con su propia conexión MQTT SALIENTE a TTN (TTN acepta varios
clientes por aplicación). No abre puertos en el servidor.

  escuchar                      Servicio: registra versión de firmware por nodo, reportes OTA y
                                eventos de voz/gestos en MongoDB local.
  ordenar  --dev ID --version   Ordena a un nodo actualizarse (1 downlink por TTN, puerto 10).
  campana  [--max N]            Ordena la versión publicada a los nodos desactualizados, de a N.
  estado                        Lista nodos, versiones y trabajos OTA.
  medir / intervalo / reiniciar Comandos de mantenimiento por downlink.

Configuración por variables de entorno o archivo .env en esta carpeta:
  TTN_APP_ID, TTN_API_KEY, TTN_MQTT_HOST (nam1.cloud.thethings.network), MONGO_URI,
  MONGO_DB (helmo_database), OTA_DIR (/opt/helmo/ota), OTA_CANAL (ph-node)

Requisitos: pip install paho-mqtt pymongo
"""
import argparse
import base64
import json
import os
import ssl
import sys
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path

import paho.mqtt.client as mqtt
from pymongo import MongoClient, ReturnDocument

# ---------------------------------------------------------------------------------------------
# Configuración
# ---------------------------------------------------------------------------------------------
_env = Path(__file__).with_name(".env")
if _env.exists():
    for linea in _env.read_text().splitlines():
        linea = linea.strip()
        if linea and not linea.startswith("#") and "=" in linea:
            k, v = linea.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip().strip('"'))

CFG = {
    "app_id": os.environ.get("TTN_APP_ID", ""),
    "api_key": os.environ.get("TTN_API_KEY", ""),
    "host": os.environ.get("TTN_MQTT_HOST", "nam1.cloud.thethings.network"),
    "mongo_uri": os.environ.get("MONGO_URI", "mongodb://localhost:27017"),
    "mongo_db": os.environ.get("MONGO_DB", "helmo_database"),
    "ota_dir": os.environ.get("OTA_DIR", "/opt/helmo/ota"),
    "canal": os.environ.get("OTA_CANAL", "ph-node"),
}

PUERTO_CMD = 10
TRABAJO_VENCE_H = 24   # un trabajo sin respuesta en 24 h se marca VENCIDO


def ahora():
    """! @brief Fecha y hora actual en UTC."""
    return datetime.now(timezone.utc)


def db():
    """! @brief Conexión a la base de datos MongoDB local de HELMO."""
    return MongoClient(CFG["mongo_uri"])[CFG["mongo_db"]]


def version_tupla(v):
    """! @brief Convierte "M.m.p" en una tupla comparable."""
    return tuple(int(x) for x in v.split("."))


# ---------------------------------------------------------------------------------------------
# MQTT (compatible con paho-mqtt 1.x y 2.x)
# ---------------------------------------------------------------------------------------------
def cliente_mqtt(nombre):
    """! @brief Crea un cliente MQTT con TLS hacia TTN (compatible con paho-mqtt 1.x y 2.x)."""
    if not CFG["app_id"] or not CFG["api_key"]:
        sys.exit("Faltan TTN_APP_ID / TTN_API_KEY (archivo .env)")
    cid = f"{nombre}-{os.getpid()}"
    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)
    except AttributeError:
        c = mqtt.Client(client_id=cid)
    c.username_pw_set(f"{CFG['app_id']}@ttn", CFG["api_key"])
    c.tls_set(cert_reqs=ssl.CERT_REQUIRED)
    return c


def enviar_downlink(dev_id, payload: bytes, prioridad="NORMAL"):
    """! @brief Encola un downlink en TTN por el puerto 10.
    @param prioridad prioridad del downlink en TTN (NORMAL por defecto)
    @param dev_id End device ID en TTN
    @param payload bytes del comando
    @details En Clase A el nodo lo recibe después de su próximo uplink."""
    topic = f"v3/{CFG['app_id']}@ttn/devices/{dev_id}/down/push"
    cuerpo = json.dumps({"downlinks": [{
        "f_port": PUERTO_CMD,
        "frm_payload": base64.b64encode(payload).decode(),
        "priority": prioridad,
    }]})
    c = cliente_mqtt("helmo-ota-cmd")
    c.connect(CFG["host"], 8883)
    c.loop_start()
    info = c.publish(topic, cuerpo, qos=1)
    info.wait_for_publish(timeout=15)
    c.loop_stop()
    c.disconnect()
    if not info.is_published():
        sys.exit("No se pudo publicar el downlink en TTN")
    print(f"Downlink encolado para {dev_id}: {payload.hex()} (llega tras su próximo uplink, Clase A)")


# ---------------------------------------------------------------------------------------------
# Orquestación OTA
# ---------------------------------------------------------------------------------------------
def manifiesto_publicado():
    """! @brief Lee el manifest.json publicado del canal configurado."""
    p = Path(CFG["ota_dir"]) / CFG["canal"] / "manifest.json"
    if not p.exists():
        sys.exit(f"No hay manifiesto publicado en {p}. Primero: firmar_firmware.py firmar ...")
    return json.loads(p.read_text())


def siguiente_trabajo(d):
    """! @brief Número de trabajo OTA consecutivo entre 1 y 255 (cabe en un byte)."""
    contador = d.contadores.find_one_and_update(
        {"_id": "ota_trabajo"}, {"$inc": {"valor": 1}}, upsert=True, return_document=ReturnDocument.AFTER)
    return (contador["valor"] - 1) % 255 + 1   # 1..255 (cabe en 1 byte)


def ordenar(dev_id, version, d=None):
    """! @brief Ordena a un nodo actualizarse a la versión publicada y registra el trabajo en MongoDB."""
    d = d or db()
    m = manifiesto_publicado()
    if version != m["version"]:
        sys.exit(f"La versión publicada es {m['version']}; el nodo rechazaría {version}.")
    nodo = d.nodos.find_one({"_id": dev_id})
    if nodo and nodo.get("fw_version") and version_tupla(nodo["fw_version"]) >= version_tupla(version):
        print(f"{dev_id} ya tiene {nodo['fw_version']}; no se ordena."); return
    if d.ota_trabajos.find_one({"dev_id": dev_id, "estado": "ORDENADO"}):
        sys.exit(f"{dev_id} ya tiene un trabajo OTA en curso. Espera su reporte o que venza.")

    trabajo = siguiente_trabajo(d)
    M, mi, p = version_tupla(version)
    enviar_downlink(dev_id, bytes([0x01, trabajo, M, mi, p]))
    d.ota_trabajos.insert_one({
        "dev_id": dev_id, "trabajo": trabajo, "version_objetivo": version,
        "version_inicial": (nodo or {}).get("fw_version"), "estado": "ORDENADO",
        "creado": ahora(), "actualizado": ahora(), "historial": [],
    })
    print(f"Trabajo OTA {trabajo}: {dev_id} -> v{version}")


def campana(max_simultaneos):
    """! @brief Ordena la versión publicada a los nodos desactualizados, sin superar max_simultaneos."""
    d = db()
    m = manifiesto_publicado()
    en_curso = d.ota_trabajos.count_documents({"estado": "ORDENADO"})
    cupo = max(0, max_simultaneos - en_curso)
    pendientes = [n for n in d.nodos.find({"fw_version": {"$exists": True}})
                  if version_tupla(n["fw_version"]) < version_tupla(m["version"])
                  and not d.ota_trabajos.find_one({"dev_id": n["_id"], "estado": "ORDENADO"})]
    print(f"Versión publicada {m['version']} | desactualizados {len(pendientes)} | cupo {cupo}")
    for n in pendientes[:cupo]:
        ordenar(n["_id"], m["version"], d)


# ---------------------------------------------------------------------------------------------
# Servicio: procesar uplinks
# ---------------------------------------------------------------------------------------------
def procesar_uplink(d, msg):
    """! @brief Registra en MongoDB la versión del nodo, los eventos HMI y los reportes OTA de un uplink."""
    dev_id = msg.get("end_device_ids", {}).get("device_id")
    up = msg.get("uplink_message", {})
    datos = up.get("decoded_payload") or {}
    tipo = datos.get("tipo")
    if not dev_id or not tipo:
        return
    recibido = msg.get("received_at") or ahora().isoformat()

    cambios = {"ultima_vista": ahora(), "ultimo_puerto": up.get("f_port")}
    if "fw_version" in datos and tipo == "lectura_ph":
        cambios.update({k: datos[k] for k in ("fw_version", "estado_ph", "hmi_conectado", "fw_en_prueba", "ph")
                        if k in datos})
    d.nodos.update_one({"_id": dev_id}, {"$set": cambios, "$setOnInsert": {"creado": ahora()}}, upsert=True)

    if tipo == "evento_hmi":
        d.hmi_eventos.insert_one({"dev_id": dev_id, "recibido": recibido, **datos})
        print(f"[HMI] {dev_id}: {datos.get('fuente')} '{datos.get('etiqueta')}' -> {datos.get('accion')}")

    elif tipo == "reporte_ota":
        resultado = datos.get("resultado")
        d.nodos.update_one({"_id": dev_id}, {"$set": {"fw_version": datos.get("fw_version")}})
        t = d.ota_trabajos.find_one_and_update(
            {"dev_id": dev_id, "trabajo": datos.get("trabajo")},
            {"$set": {"estado": resultado, "actualizado": ahora(), "detalle": datos.get("detalle"),
                      "fw_reportada": datos.get("fw_version")},
             "$push": {"historial": {"en": recibido, **datos}}},
            sort=[("creado", -1)])
        print(f"[OTA] {dev_id} trabajo {datos.get('trabajo')}: {resultado} "
              f"(v{datos.get('fw_version')}, {datos.get('detalle')}){'' if t else ' [sin trabajo registrado]'}")


def vencer_trabajos(d):
    """! @brief Marca como VENCIDO todo trabajo sin respuesta en TRABAJO_VENCE_H horas."""
    limite = ahora() - timedelta(hours=TRABAJO_VENCE_H)
    r = d.ota_trabajos.update_many({"estado": "ORDENADO", "creado": {"$lt": limite}},
                                   {"$set": {"estado": "VENCIDO", "actualizado": ahora()}})
    if r.modified_count:
        print(f"[OTA] {r.modified_count} trabajo(s) vencido(s)")


def escuchar():
    """! @brief Servicio principal: se suscribe a los uplinks de la aplicación en TTN."""
    d = db()
    d.ota_trabajos.create_index([("dev_id", 1), ("trabajo", 1)])
    d.hmi_eventos.create_index([("dev_id", 1), ("recibido", -1)])
    c = cliente_mqtt("helmo-ota")

    def al_conectar(cliente, *args):
        cliente.subscribe(f"v3/{CFG['app_id']}@ttn/devices/+/up", qos=1)
        print(f"[MQTT] Conectado a {CFG['host']}, escuchando uplinks de {CFG['app_id']}")

    def al_mensaje(cliente, userdata, m):
        try:
            procesar_uplink(d, json.loads(m.payload))
        except Exception as e:   # un mensaje malo no debe tumbar el servicio
            print(f"[ERROR] {e}")

    c.on_connect = al_conectar
    c.on_message = al_mensaje
    c.reconnect_delay_set(min_delay=2, max_delay=120)
    c.connect(CFG["host"], 8883)
    c.loop_start()
    try:
        while True:
            vencer_trabajos(d)
            time.sleep(600)
    except KeyboardInterrupt:
        c.loop_stop()


def estado():
    """! @brief Imprime nodos, versiones, estado de pH y últimos trabajos OTA."""
    d = db()
    print(f"{'NODO':<20}{'FW':<10}{'pH':<7}{'ESTADO':<10}{'HMI':<5}{'ÚLTIMA VEZ'}")
    for n in d.nodos.find().sort("_id"):
        print(f"{n['_id']:<20}{n.get('fw_version', '?'):<10}{str(n.get('ph', '-')):<7}"
              f"{n.get('estado_ph', '-'):<10}{'sí' if n.get('hmi_conectado') else 'no':<5}"
              f"{n.get('ultima_vista', '-')}")
    print("\nÚltimos trabajos OTA:")
    for t in d.ota_trabajos.find().sort("creado", -1).limit(10):
        print(f"  #{t['trabajo']:<4}{t['dev_id']:<20}v{t['version_objetivo']:<8}{t['estado']:<11}{t.get('detalle', '')}")


# ---------------------------------------------------------------------------------------------
def main():
    """! @brief Punto de entrada de la línea de comandos."""
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("escuchar")
    sub.add_parser("estado")
    s = sub.add_parser("ordenar"); s.add_argument("--dev", required=True); s.add_argument("--version", required=True)
    s = sub.add_parser("campana"); s.add_argument("--max", type=int, default=1)
    s = sub.add_parser("medir"); s.add_argument("--dev", required=True)
    s = sub.add_parser("reiniciar"); s.add_argument("--dev", required=True)
    s = sub.add_parser("intervalo"); s.add_argument("--dev", required=True); s.add_argument("--min", type=int, required=True)
    a = p.parse_args()

    if a.cmd == "escuchar":
        escuchar()
    elif a.cmd == "estado":
        estado()
    elif a.cmd == "ordenar":
        ordenar(a.dev, a.version)
    elif a.cmd == "campana":
        campana(a.max)
    elif a.cmd == "medir":
        enviar_downlink(a.dev, bytes([0x03]))
    elif a.cmd == "reiniciar":
        enviar_downlink(a.dev, bytes([0x04]))
    elif a.cmd == "intervalo":
        if not 1 <= a.min <= 1440:
            sys.exit("--min entre 1 y 1440")
        enviar_downlink(a.dev, bytes([0x02, a.min >> 8, a.min & 0xFF]))


if __name__ == "__main__":
    main()
