# HELMO · Nodo de pH con OTA híbrida e interfaz de voz y gestos

Integra en la arquitectura de HELMO (nodos Heltec V3 → gateway Dragino LPS8N → TTN Sandbox → MQTT → servidor Debian con helmo-bridge, MongoDB, Grafana y ngrok) tres trabajos previos:

| Trabajo previo | Qué se conserva | Qué cambia |
|---|---|---|
| FUOTA por LoRaWAN (ChirpStack, Clase C) | Puertos de comando, códigos de resultado, idea de sesión/trabajo | Ya no transfiere la imagen por LoRa: TTN Sandbox solo permite ≈10 downlinks/día. Se vuelve a Clase A |
| OTA segura por Wi-Fi (Jetson Orin Nano) | SHA-256 progresivo, `Update.abort()`, watchdog de transferencia | Se agrega **firma ECDSA**, anti-downgrade, rollback y el Wi-Fi solo se enciende al recibir la orden |
| Voz y gestos (Nano 33 BLE Sense) | Modelo de Edge Impulse y modelo de gestos | Se unen en un solo firmware y se comunican con el nodo por UART |

## ¿Por qué dos firmwares?

La Heltec V3 no tiene micrófono ni IMU, y el Nano 33 BLE Sense no tiene radio LoRa. Cada placa hace lo que sabe hacer:

- **Firmware 1 · `firmware/helmo_nodo_ph`** (Heltec V3): pH, LoRaWAN/TTN, OTA, decisiones.
- **Firmware 2 · `firmware/helmo_hmi_nano`** (Nano 33 BLE Sense): escucha y detecta gestos, muestra el semáforo de pH.

El nodo funciona igual sin el módulo HMI (`HMI_HABILITADO 0`).

## Cómo se orquesta una actualización

```
 Servidor Debian                    TTN Sandbox            Nodo Heltec (Clase A)
 ───────────────                    ───────────            ─────────────────────
 1. firmar_firmware.py firmar  ──►  /opt/helmo/ota/ph-node/{manifest.json, .bin}
 2. helmo_ota.py ordenar ──MQTT──►  cola downlink  ──────► (tras su próximo uplink)
                                    puerto 10: 01 07 02 01 00  = "trabajo 7, ve a la v2.1.0"
                                                           3. Enciende Wi-Fi de mantenimiento
                                                           4. GET manifest.json  (vía ngrok/Caddy)
                                                           5. Verifica firma ECDSA + versión > actual
                                                           6. Descarga con SHA-256 progresivo
                                                           7. Update.end() → reinicia
                                                           8. Periodo de prueba: JOIN a TTN
                                                              ✓ se une  → confirma la versión
                                                              ✗ 15 min / 3 reinicios → ROLLBACK
 9. helmo_ota.py escuchar  ◄──MQTT──  uplink puerto 5: EXITO | ERROR(detalle) | ROLLBACK
    → MongoDB: nodos, ota_trabajos  → Grafana
```

**Por qué es seguro aunque `/ota` sea público:** el nodo solo acepta un manifiesto firmado con la llave privada que vive en el servidor Debian. Quien controle la red o el túnel puede impedir la actualización, pero no instalar firmware falso ni una versión vieja.

## 1. Conexiones

| Nano 33 BLE Sense | Heltec V3 |
|---|---|
| D1 (TX) | GPIO 47 (RX) |
| D0 (RX) | GPIO 48 (TX) |
| GND | GND |

Ambas placas usan lógica de 3,3 V. Alimenta el Nano por su USB o por **VIN desde el pin 5V** de la Heltec. Sensor de pH y MOSFET quedan igual (GPIO 1 y GPIO 6).

## 2. Firmware 1 · Nodo de pH (Heltec V3)

1. Placa **Heltec WiFi LoRa 32(V3)**, región **US915**, y en *Tools → Partition Scheme* elige **8M with spiffs (3MB APP/1.5MB SPIFFS)**: deja dos particiones de aplicación con espacio de sobra.
2. Librería **ArduinoJson** (Library Manager).
3. Copia `secretos.ejemplo.h` como **`secretos.h`** (misma carpeta) y llénalo con tus DevEUI/JoinEUI/AppKey de TTN, el Wi-Fi de mantenimiento y `OTA_BASE_URL`. `secretos.h` nunca se publica ni se documenta.
4. Genera las llaves (paso 4) para reemplazar `clave_publica_ota.h`. Mientras tenga el marcador, el nodo rechaza toda actualización.
5. Graba la **v2.0.0 por USB** una sola vez. Desde ahí, las siguientes van por OTA.

### Puertos LoRaWAN

| Puerto | Dirección | Contenido |
|---|---|---|
| 3 | ↑ | pH, mV, versión de firmware, estado (normal/aviso/alarma), HMI conectado, en prueba |
| 4 | ↑ | Evento de voz o gesto (solo los que importan al servidor) |
| 5 | ↑ | Resultado OTA: `EXITO`, `ERROR` + detalle, `ROLLBACK`, `YA_AL_DIA` |
| 10 | ↓ | `01` OTA · `02` intervalo · `03` medir ya · `04` reiniciar |

Todos caben en DR0 (11 bytes). El intervalo por defecto es **5 min** para respetar el uso justo de TTN (~30 s de aire al día); en laboratorio puedes bajarlo con `helmo_ota.py intervalo --min 1` durante una prueba corta.

## 3. Firmware 2 · Módulo HMI (Nano 33 BLE Sense)

1. Instala el `.zip` de Edge Impulse de `speech_recognition_Esteban` (*Sketch → Include Library → Add .ZIP Library*).
2. Copia tu `model.h` de gestos en la carpeta del sketch y cambia su primera línea por:
   ```cpp
   alignas(16) const unsigned char modelo_gestos[] = {
   ```
3. En el `.ino` pon `IMU_REV2 1` si tu placa dice *REV2*; deja `0` si es la Rev1 (LSM9DS1).
4. **No necesitas `Arduino_TensorFlowLite`**: el modelo de gestos corre con el TensorFlow Lite que trae la librería de Edge Impulse. Así se evita tener dos copias de TensorFlow chocando.

### Qué significa cada palabra y gesto

Se cambia en la tabla `REGLAS_HMI` del firmware 1.

| Detección | Acción en el nodo | ¿Viaja a TTN? |
|---|---|---|
| Voz **Rojo** | Reporte de incidente del operario | Sí (puerto 4) |
| Voz **Amarillo** | Medición de pH inmediata | El pH sí (puerto 3) |
| Voz **Verde** | Reconoce la alarma activa (el LED deja de parpadear) | Sí (puerto 4) |
| Gesto **punch** | Medición inmediata (útil en planta ruidosa) | El pH sí |
| Gesto **flex** | Muestra el último pH en el LED | No |

Semáforo del LED RGB: verde 6,5–9,0 · amarillo a menos de 0,3 del límite · rojo parpadeando fuera de rango sin reconocer · rojo fijo reconocida · azul rápido durante una OTA. Cada detección se ve 1,5 s con su color.

## 4. Servidor Debian 12

```bash
sudo mkdir -p /opt/helmo/ota /opt/helmo/servidor && sudo chown -R $USER /opt/helmo
cp servidor/* /opt/helmo/servidor/ && cd /opt/helmo/servidor
python3 -m venv /opt/helmo/venv && /opt/helmo/venv/bin/pip install -r requirements.txt
cp env.ejemplo .env    # pon tu TTN_APP_ID y una API key con permiso de leer y escribir downlinks

# Llaves (una sola vez). La privada queda con permisos 600: haz una copia de seguridad offline.
/opt/helmo/venv/bin/python firmar_firmware.py llaves \
    --header ~/helmo-ota-hmi/firmware/helmo_nodo_ph/clave_publica_ota.h
```

**TTN:** en *Applications → Payload formatters → Uplink* y *Downlink* pega `ttn_formatter.js`.

**Distribución de imágenes:** instala Caddy (`sudo apt install caddy`), copia `Caddyfile` a `/etc/caddy/Caddyfile` y apunta el agente ngrok a `http://localhost:8088`. Grafana sigue en `/` y las imágenes firmadas en `/ota/`, con un solo túnel. En `secretos.h` usa `OTA_BASE_URL "https://TU-DOMINIO.ngrok-free.app/ota"`. En laboratorio usa `http://IP-DEL-DEBIAN:8088/ota`.

**Servicio de orquestación:**
```bash
sudo cp helmo-ota.service /etc/systemd/system/ && sudo systemctl enable --now helmo-ota
journalctl -u helmo-ota -f
```
Usa su propia conexión MQTT saliente, así que no hay que tocar helmo-bridge. Si helmo-bridge ya decodificaba el puerto 3 con 6 bytes, el formateador nuevo sigue entregando `node_id`, `ph` y `voltaje` con los mismos nombres.

Colecciones nuevas en `helmo_database`: `nodos` (versión y estado por nodo), `ota_trabajos` (cada orden y su resultado) y `hmi_eventos` (voz y gestos). Puedes graficarlas en Grafana.

## 5. Publicar y ordenar una actualización

```bash
# En el .ino: FW_MINOR 1  (2.0.0 → 2.1.0; el LED pasa de lento a triple destello)
# Arduino IDE: Sketch → Export Compiled Binary → usa helmo_nodo_ph.ino.bin (NO el .merged.bin)

/opt/helmo/venv/bin/python firmar_firmware.py firmar --bin helmo_nodo_ph.ino.bin --version 2.1.0
/opt/helmo/venv/bin/python firmar_firmware.py verificar --manifiesto /opt/helmo/ota/ph-node/manifest.json
/opt/helmo/venv/bin/python helmo_ota.py ordenar --dev ph-lab-01 --version 2.1.0
/opt/helmo/venv/bin/python helmo_ota.py estado          # o: campana --max 1 para todos los nodos
```
`--dev` es el *End device ID* de TTN. El nodo recibe la orden después de su siguiente uplink (máximo un intervalo).

## 6. Pruebas para la rúbrica

| Prueba | Cómo provocarla | Resultado esperado |
|---|---|---|
| Actualización normal | Ordenar 2.1.0 | LED cambia de patrón; puerto 5 `EXITO` v2.1.0 |
| Corte de red | Apagar el hotspot a mitad de la descarga | `ERROR / interrumpida`; sigue en 2.0.0 |
| Imagen alterada | Cambiar un byte del `.bin` publicado | `ERROR / sha256`; sigue en 2.0.0 |
| Firma falsa | Firmar con otra llave (`llaves --dir otra`) | `ERROR / firma_invalida`, antes de tocar la flash |
| Downgrade | Ordenar una versión menor (con `--forzar` al firmar) | `ERROR / version` o `YA_AL_DIA` |
| Rollback | Publicar una versión con `devEui` incorrecto (no puede unirse) | Tras 15 min vuelve a la anterior y reporta `ROLLBACK` |
| Voz / gesto | Decir "Rojo" cerca del Nano | LED rojo 1,5 s; en MongoDB `hmi_eventos` aparece el incidente |

## 7. Problemas frecuentes

- **`'Radio' was not declared`** al compilar el nodo: cambia `Radio.IrqProcess();` por `LoRaWAN.sleep(loraWanClass);` (algunas versiones de la librería Heltec no exponen `Radio`). Si con eso se pierden eventos del HMI, avísame.
- **`undefined reference ... MicroInterpreter`** en el Nano: exporta otra vez la librería en Edge Impulse con el **EON Compiler desactivado** (así incluye el intérprete completo).
- **`AllocateTensors` falla**: sube `TAM_ARENA_GESTOS` a `16 * 1024`.
- **`Update.begin` falla / `tamano`**: revisa el esquema de particiones del paso 2.
- **`ERROR / wifi`**: el nodo no alcanza el hotspot; acerca el celular del técnico o usa la red 2 de `secretos.h`.
- **`ERROR / manifiesto_http`**: prueba la URL de `OTA_BASE_URL/ph-node/manifest.json` desde el navegador del celular conectado al mismo hotspot.

## Archivos

```
firmware/helmo_nodo_ph/      helmo_nodo_ph.ino, secretos.ejemplo.h, clave_publica_ota.h
                             (+ tu secretos.h, que no se comparte)
firmware/helmo_hmi_nano/     helmo_hmi_nano.ino  (+ tu model.h)
servidor/                    ttn_formatter.js, firmar_firmware.py, helmo_ota.py,
                             Caddyfile, helmo-ota.service, env.ejemplo, requirements.txt
Doxyfile                     configuración de la documentación
```

## 8. Documentación con Doxygen

```bash
doxygen Doxyfile        # desde la carpeta helmo-ota-hmi
```
La documentación queda en `docs/html/index.html`. El `Doxyfile` excluye `secretos.h`, `servidor/.env`, la carpeta `servidor/llaves`, los `.pem` y el `model.h` de gestos, así que la documentación no contiene llaves ni contraseñas.

Para comentar funciones nuevas, escribe `/**` y pulsa Enter encima de la función (extensión *Doxygen Documentation Generator* de VS Code). En Python, empieza el docstring con `"""!`.
