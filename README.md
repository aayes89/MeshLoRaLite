# MeshLoRaLite_CLI

Firmware para nodos LoRa en malla (mesh) basado en **STM32F1** + **SX1262** (RadioLib) con interfaz de comandos por puerto serie (CLI), soporte para mensajes unicast/broadcast, reenvío inteligente y configuración persistente en Flash. <br>
Escrito 100% en Arduino IDE y probado en **DX-SMART DX-PJ26-V1.1** + **DX-LR30**.<br>
Para subir el binario al STM32 ver el siguiente repositorio: <a href="https://github.com/aayes89/STM32F1-serie">STM32F1-serie</a>

Proyecto orientado a redes LoRa de bajo costo, larga distancia y bajo consumo para aplicaciones como monitoreo distribuido, comunicación off-grid o experimentación con mesh descentralizado.

***<h3>Nuevo: Aplicación cliente para Android añadida</h3>***

## Características principales

- **Topología mesh** basada en flooding controlado con TTL y supresión de duplicados
- Mensajes **unicast** (nodoID) y **broadcast** (0xFFFF)
- Configuración de canal lógico (`chan`) persistida en Flash (Permite segmentar redes mesh en la misma frecuencia física.)
- Detección y eliminación de duplicados (cache de últimos mensajes vistos)
- **Beacon periódico (30s)** para descubrimiento de nodos vecinos (ajustar)
- Interfaz CLI completa por puerto serie (115200 por defecto, configurable)
- Comandos: `send`, `broadcast`, `set`, `get`, `save`, `load`, `status`, `reboot`, etc.
- Configuración persistente en última página de Flash (frecuencia, SF, BW, CR, potencia, TTL, beacon, debug)
- Control de nivel de depuración persistente (`set debug on/off`)
- Uso de **RadioLib** para manejo del DX-LR30 (SX1262)
- Generación de **NodeID único** basado en UID del chip STM32 + CRC16
- Jitter aleatorio en envíos y reenvíos para reducir colisiones
- ACKs y confirmaciones de entrega
- Lista de vecinos detectados vía beacons

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
2. Abre el proyecto en Arduino IDE 
3. Conecta el nodo por USB y sube el firmware
4. Abre el monitor serie a 115200 baud (o el que hayas configurado)

## Comandos disponibles:

- help                    → muestra el menu de comandos
- get [all|radio|mesh]    → obtiene información específica del módulo
- status                  → muestra info del nodo
- send <NODEID> <mensaje> → envía mensaje unicast al nodo 0xXXXX
- broadcast <mensaje>     → envía a todos los nodos
- set freq 915000000      → cambia frecuencia (Hz)
- set sf 9                → spreading factor 7-12
- set bw 250              → ancho de banda en kHz
- set cr <4 a 8>          → tasa de codificación
- set ttl <0 a 10 (máx)>  → tiempo de vida
- set beacon <ms>         → intervalo de envío de beacon. Ej: 30000
- set power <-9 a 22>     → potencia dBm
- set debug <on|off|1|0>  → logs detallados
- set chan <0 a 255>      → modifica el canal de transmisión
- set baud <115200>       → baudios
- nodes                   → devuelve la información sobre los nodos cercanos enlistados
- nodes clear             → limpia la tabla de nodos cercanos
- save                    → guarda configuración en Flash
- load                    → carga la configuración del Flash
- reboot                  → reinicia el nodo

* NodeID se ingresa en formato hexadecimal sin 0x (ej: 6019, 4CA9, etc.) — coincide con el valor mostrado al iniciar.
* Configuración persistente. Todos los parámetros se guardan en la última página de Flash (1 KB reservado). Al iniciar, se carga automáticamente. Si falla, usa valores por defecto.

## Desarrollo

### Canal lógico: 
* Cada paquete lleva un campo `chan` en su header mesh.
* El nodo solo procesa mensajes cuyo `chan` coincida con el canal actual configurado.
* Esto permite coexistir varias redes mesh en la misma frecuencia física evitando colisiones indeseadas.
* El canal lógico no cifra ni aísla tráfico, solo filtra.

### Reconocimiento y reitentos
* Los paquetes unicast esperan ACK de destino.
* Si no se recibe dentro de ~3 s, se reintenta hasta 2 veces.
* El contenido original se guarda para reenviar correctamente (no solo header).

### Tabla de vecinos
* Nodos detectados vía beacons se registran con RSSI/SNR y tiempo.
* Se limpia automáticamente si no llegan beacons por ~90 s.

## Depuración
Activa/desactiva logs detallados:
- set debug on
- set debug off
- save

* RSSI y SNR se muestran automáticamente en logs RX si debug está activado.
* Logs condicionados a la variable debug (volatile bool sincronizada con cfg.debug)

## Persistencia en Flash

Se guarda:
- Parámetros de radio
- TTL, beacon, debug, canal lógico

NO se guarda:
- Tabla de vecinos
- Cache de paquetes vistos
- Cola de reintentos

## Arquitectura interna (resumen)

- Radio en RX continuo con interrupciones (DIO1)
- TX no bloqueante + retorno explícito a RX
- Cache de paquetes vistos (src + id)
- Cola de paquetes pendientes para reintentos unicast
- Reenvío condicionado por TTL, destino y canal lógico

## Consideraciones de memoria y limitaciones

STM32F103C8T6 (~20 KB RAM):
- Tamaño de tablas y buffers es fijo y limitado ( payload ~64 bytes )
- Optimizado para redes pequeñas / medianas
- No apto para cientos de nodos simultáneos
- Solo flooding (no routing inteligente aún)
- Pendiente: soporte dinámico para cambio de baudrate serie sin reinicio completo
- Pendiente: integración con apps/web (WebSerial, Bluetooth, etc.)

## Diagrama lógico
```
TX → Radio → Aire → RX ISR → handlePkt()
                  ↓
             Filtro (ver/chan/seen)
                  ↓
           Procesar / Forward / ACK
```

## Licencia
MIT License

## Contribuciones
**¡Bienvenidas!**<br>
Abre un issue o pull request si quieres agregar ACKs (backoff adaptativo, métricas, etc.), encriptación simple, mejor manejo de colisiones, visualización de topología, etc.<br>
¡Gracias por probar MeshLoRaLite!
