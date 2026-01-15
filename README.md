# MeshLoRaLite_CLI

Firmware para nodos LoRa en malla (mesh) basado en **STM32F1** + **SX1262** (RadioLib) con interfaz de comandos por puerto serie (CLI), soporte para mensajes unicast/broadcast, reenvío inteligente y configuración persistente en Flash.

Probado en **DX-SMART DX-PJ26-V1.1** + **DX-LR30**.

Proyecto orientado a redes LoRa de bajo costo, larga distancia y bajo consumo para aplicaciones como monitoreo distribuido, comunicación off-grid o experimentación con mesh descentralizado.

## Características principales

- **Topología mesh flooding** con TTL (Time To Live) configurable
- Mensajes **unicast** y **broadcast** (0xFFFF)
- Detección y eliminación de duplicados (cache de últimos mensajes vistos)
- **Beacon periódico** para descubrimiento de nodos vecinos
- Interfaz CLI completa por puerto serie (115200 por defecto, configurable)
- Comandos: `send`, `broadcast`, `set`, `get`, `save`, `load`, `status`, `reboot`, etc.
- Configuración persistente en última página de Flash (frecuencia, SF, BW, CR, potencia, TTL, beacon, debug)
- Control de nivel de depuración persistente (`set debug on/off`)
- Uso de **RadioLib** para manejo del SX1262
- Generación de **NodeID único** basado en UID del chip STM32 + CRC16
- Jitter aleatorio en envíos y reenvíos para reducir colisiones

## Requisitos de hardware

- MCU: **STM32F103** (Blue Pill o similar)
- Módulo LoRa: **SX1262** (DX-LR30, RA-02, E22, Heltec, etc. con pines estándar)
- Conexiones recomendadas:

| Pin STM32   | Pin SX1262     | Función          |
|-------------|----------------|------------------|
| PA4         | NSS            | Chip Select      |
| PA5         | SCK            | SPI Clock        |
| PA7         | MOSI           | SPI Data In      |
| PA6         | MISO           | SPI Data Out     |
| PA3         | RESET          | Reset            |
| PA2         | BUSY           | Busy/IRQ         |
| PC15        | DIO1           | IRQ RX/TX        |
| PB11        | LED (opcional) | Indicador estado |

- Opcional: pines TXEN/RXEN si tu módulo los requiere (actualmente no usados, Dio2 como RF switch)

## Instalación y uso rápido

1. Clona el repositorio
   ```bash
   git clone https://github.com/aayes89/MeshLoRaLite.git
2. Abre el proyecto en PlatformIO / Arduino IDE (recomendado PlatformIO)
3. Conecta el nodo por USB y sube el firmware
4. Abre el monitor serie a 115200 baud (o el que hayas configurado)

## Comandos básicos:

- status                  → muestra info del nodo
- send 4CA9 hola mundo    → envía mensaje unicast al nodo 0x4CA9
- broadcast Alerta!       → envía a todos los nodos
- set freq 868000000      → cambia frecuencia (Hz)
- set sf 9                → spreading factor 7-12
- set bw 250              → bandwidth en kHz
- set power 14            → potencia dBm
- set debug on            → activa logs detallados
- save                    → guarda configuración en Flash
- reboot                  → reinicia el nodo

## Comandos disponibles:

- help
- get [all|radio|mesh]
- set <key> <value>     (freq, sf, bw, cr, ttl, beacon, power, debug)
- save
- load
- reboot
- send <NODEID> <mensaje>
- broadcast <mensaje>
- status

* NodeID se ingresa en formato hexadecimal sin 0x (ej: 6019, 4ca9, etc.) — coincide con el valor mostrado al iniciar.
* Configuración persistente. Todos los parámetros se guardan en la última página de Flash (1 KB reservado). Al iniciar, se carga automáticamente. Si falla, usa valores por defecto.

## Desarrollo y depuración

Activa/desactiva logs detallados:
- set debug on
- set debug off
- save

Logs condicionados a la variable debug (volatile bool sincronizada con cfg.debug)

##  Limitaciones conocidas / Futuro

- Solo flooding (no routing inteligente aún)
- No ACKs ni confirmaciones de entrega
- Tamaño máximo payload ~200 bytes
- Pendiente: soporte dinámico para cambio de baudrate serie sin reinicio completo
- Pendiente: lista de vecinos detectados vía beacons
- Pendiente: integración con apps/web (WebSerial, Bluetooth, etc.)

## Licencia
MIT License

## Contribuciones
**¡Bienvenidas!**<br>
Abre un issue o pull request si quieres agregar ACKs, encriptación simple, mejor manejo de colisiones, visualización de topología, etc.<br>
¡Gracias por probar MeshLoRaMesh!
