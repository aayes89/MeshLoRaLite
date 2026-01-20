// ========================== MeshLoRaLite by Slam 2026 ==========================

#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "stm32f1xx_hal.h"

/* ================= PINOUT ================= */
#define LORA_NSS PA4
#define LORA_SCK PA5
#define LORA_MOSI PA7
#define LORA_MISO PA6
#define LORA_NRST PA3
#define LORA_BUSY PA2
#define LORA_DIO1 PC15
#define LED_PIN PB11
//#define LORA_TXEN PA0  // no se usan directamente
//#define LORA_RXEN PA1  // no se usan directamente

/* ================= FLASH ================= */
#define FLASH_BASE_ADDR 0x08000000
#define FLASH_SIZE_KB (*((uint16_t*)0x1FFFF7E0))
#define FLASH_CFG_ADDR (FLASH_BASE_ADDR + FLASH_SIZE_KB * 1024 - 1024)

/* ================= RADIO ================= */
SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_NRST, LORA_BUSY);

/* ================= CONFIG ================= */
#define MAX_PKT 220
#define NODE_BCAST 0xFFFF
#define SEEN_CACHE 32
#define MESH_VER 1
#define UART_SPEED 115200
#define NODE_TABLE_SIZE 16     // cantidad máxima de nodos
#define NODE_TIMEOUT_MS 90000  // 90s antes de actualizar tabla de nodos
#define MAX_PENDING 4

enum { PKT_DATA = 1,
       PKT_BEACON = 2,
       PKT_ACK = 3 };

/* ================= STRUCTS ================= */
typedef struct {
  uint16_t id;
  int8_t rssi;
  int8_t snr;
  uint32_t lastSeen;
} node_t;
typedef struct {
  uint16_t id;
  uint16_t dst;
  uint32_t ts;
  uint8_t retries;
  uint8_t len;
  uint8_t payload[64];  // tamaño razonable y fijo
} pending_t;
typedef struct __attribute__((packed)) {
  uint32_t freq;
  uint32_t baud;
  uint32_t beacon_ms;
  uint8_t sf;
  uint8_t bw;
  uint8_t cr;
  uint8_t ttl;
  uint8_t chan;
  int8_t power;
  uint16_t debug;  // 0-false, 1-true
  uint16_t crc;
} mesh_cfg_t;
typedef struct __attribute__((packed)) {
  uint8_t  ver;
  uint8_t  type;
  uint16_t src;
  uint16_t dst;
  uint16_t id;
  uint8_t  ttl;
  uint16_t len;
  uint8_t  chan;
} mesh_hdr_t;
typedef struct {
  uint16_t src;
  uint16_t id;
} seen_t;
/* ================= GLOBALS ================= */
mesh_cfg_t cfg;
uint8_t txBuf[MAX_PKT], rxBuf[MAX_PKT];
seen_t seen[SEEN_CACHE];
uint8_t seenIdx = 0;
uint16_t nodeID;
uint16_t msgID = 0;
volatile bool rxFlag = false, txFlag = false;
volatile bool debug = false;
uint32_t lastBeacon = 0;
uint8_t nodeCount = 0;
node_t nodes[NODE_TABLE_SIZE];
pending_t pend[MAX_PENDING];
uint32_t lastPrune = 0;
uint8_t meshChan = 0;

/* ================= IRQ ================= */
void onRx() {
  rxFlag = true;
}
void onTx() {
  txFlag = true;
}
/* ================= CRC ================= */
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}
/* ================= NODE ID ================= */
uint16_t genNodeID() {
  uint32_t uid[3] = {
    *(uint32_t*)0x1FFFF7E8,
    *(uint32_t*)0x1FFFF7EC,
    *(uint32_t*)0x1FFFF7F0
  };
  return crc16_ccitt_false((uint8_t*)uid, 12);
}
/* ================= NODOS ================= */
void updateNode(uint16_t id, int8_t rssi, int8_t snr) {
  uint32_t now = millis();

  for (uint8_t i = 0; i < nodeCount; i++) {
    if (nodes[i].id == id) {
      nodes[i].rssi = rssi;
      nodes[i].snr = snr;
      nodes[i].lastSeen = now;
      return;
    }
  }

  if (nodeCount < NODE_TABLE_SIZE) {
    nodes[nodeCount++] = { id, rssi, snr, now };
  }
}
void pruneNodes() {
  uint32_t now = millis();
  for (int i = nodeCount - 1; i >= 0; i--) {
    if (now - nodes[i].lastSeen > NODE_TIMEOUT_MS) {
      nodes[i] = nodes[--nodeCount];
    }
  }
}
/* ================= BW y CR =================*/
uint8_t bwToRadio(uint16_t bw) {
  switch (bw) {
    case 125: return RADIOLIB_SX126X_LORA_BW_125_0;
    case 250: return RADIOLIB_SX126X_LORA_BW_250_0;
    case 500: return RADIOLIB_SX126X_LORA_BW_500_0;
    case 62: return RADIOLIB_SX126X_LORA_BW_62_5;
    case 41: return RADIOLIB_SX126X_LORA_BW_41_7;
    case 31: return RADIOLIB_SX126X_LORA_BW_31_25;
    case 20: return RADIOLIB_SX126X_LORA_BW_20_8;
    case 15: return RADIOLIB_SX126X_LORA_BW_15_6;
    case 10: return RADIOLIB_SX126X_LORA_BW_10_4;
    case 7: return RADIOLIB_SX126X_LORA_BW_7_8;
    default: return RADIOLIB_SX126X_LORA_BW_125_0;
  }
}
uint8_t crToRadio(uint8_t cr) {
  switch (cr) {
    case 5: return RADIOLIB_SX126X_LORA_CR_4_5;
    case 6: return RADIOLIB_SX126X_LORA_CR_4_6;
    case 7: return RADIOLIB_SX126X_LORA_CR_4_7;
    case 8: return RADIOLIB_SX126X_LORA_CR_4_8;
    default: return RADIOLIB_SX126X_LORA_CR_4_5;
  }
}
/* ================= FLASH ================= */
bool loadConfig() {
  mesh_cfg_t* f = (mesh_cfg_t*)FLASH_CFG_ADDR;
  if (crc16_ccitt_false((uint8_t*)f, sizeof(mesh_cfg_t) - 2) != f->crc) return false;
  memcpy(&cfg, f, sizeof(cfg));
  return true;
}
void saveConfig() {
  cfg.crc = crc16_ccitt_false((uint8_t*)&cfg, sizeof(cfg) - 2);
  HAL_FLASH_Unlock();

  FLASH_EraseInitTypeDef EraseInit = { 0 };
  uint32_t PageError = 0;
  EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
  EraseInit.PageAddress = FLASH_CFG_ADDR;
  EraseInit.NbPages = 1;

  if (HAL_FLASHEx_Erase(&EraseInit, &PageError) != HAL_OK) {
    Serial.println("Flash erase failed");
    HAL_FLASH_Lock();
    return;
  }

  // Programar de a 16 bits (halfword) – STM32F1 no soporta byte
  for (size_t i = 0; i < sizeof(cfg); i += 2) {
    uint16_t value = *(uint16_t*)((uint8_t*)&cfg + i);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_CFG_ADDR + i, value) != HAL_OK) {
      Serial.println("Flash program failed");
      HAL_FLASH_Lock();
      return;
    }
  }

  HAL_FLASH_Lock();
  Serial.println("Config saved");
}
/* ================= RADIO ================= */
void radioRX() {
  radio.startReceive();
}
void radioTX(uint8_t* d, size_t l) {
  txFlag = false;
  int err = radio.startTransmit(d, l);
  if (err != RADIOLIB_ERR_NONE) {
    if (debug) Serial.printf("[TX START FAILED] error=%d\n", err);
    radio.startReceive();
  }
}
bool shouldForward(int8_t rssi, uint8_t ttl) {
  if (ttl == 0) return false;
  if (rssi < -115) return false;
  if (rssi > -90) return true;
  return random(0, 100) < 40;  // 40% chance
}
/* ================= SEEN ================= */
bool seenBefore(uint16_t s, uint16_t i) {
  for (uint8_t k = 0; k < SEEN_CACHE; k++)
    if (seen[k].src == s && seen[k].id == i) return true;
  return false;
}
void markSeen(uint16_t s, uint16_t i) {
  seen[seenIdx++] = { s, i };
  seenIdx %= SEEN_CACHE;
}
/* ================= SEND ================= */
void sendPktEx(uint8_t type, uint16_t dst, const uint8_t* payload, uint8_t len, uint8_t ttl) {
  if (len > MAX_PKT - sizeof(mesh_hdr_t) - 2) return;

  mesh_hdr_t h = { MESH_VER, type, nodeID, dst, msgID++, ttl, len, meshChan };

  markSeen(h.src, h.id);
  if (type == PKT_DATA && dst != NODE_BCAST) {
    for (uint8_t i = 0; i < MAX_PENDING; i++) {
      if (!pend[i].id) {
        pend[i].id = h.id;
        pend[i].dst = dst;
        pend[i].ts = millis();
        pend[i].retries = 0;
        pend[i].len = len;
        memcpy(pend[i].payload, payload, len);
        break;
      }
    }
  }

  uint16_t pos = 0;
  memcpy(txBuf + pos, &h, sizeof(h));
  pos += sizeof(h);

  if (len && payload) {
    memcpy(txBuf + pos, payload, len);
    pos += len;
  }

  uint16_t crc = crc16_ccitt_false(txBuf, pos);
  memcpy(txBuf + pos, &crc, 2);
  pos += 2;
  if (debug) {
    Serial.printf("[SEND] type=%u dst=%04X len=%u bytes=%u\n",
                  type, dst, len, pos);
  }
  radio.startTransmit(txBuf, pos);
}
void sendPkt(uint8_t type, uint16_t dst, const uint8_t* payload, uint8_t len) {
  sendPktEx(type, dst, payload, len, (type == PKT_ACK || type == PKT_BEACON) ? 1 : cfg.ttl);
}
/* ================= RESEND ================= */
void resend(pending_t& p) {
  mesh_hdr_t h = { MESH_VER, PKT_DATA, nodeID, p.dst, p.id, cfg.ttl, p.len, meshChan };

  uint16_t pos = 0;
  memcpy(txBuf + pos, &h, sizeof(h));
  pos += sizeof(h);

  memcpy(txBuf + pos, p.payload, p.len);
  pos += p.len;

  uint16_t crc = crc16_ccitt_false(txBuf, pos);
  memcpy(txBuf + pos, &crc, 2);
  pos += 2;

  radio.startTransmit(txBuf, pos);
}
/* ================= RX ================= */
void handlePkt(uint8_t* b, size_t l) {
  if (l < sizeof(mesh_hdr_t) + 2) return;

  // CRC RX
  uint16_t rxCrc;
  memcpy(&rxCrc, b + l - 2, 2);
  if (rxCrc != crc16_ccitt_false(b, l - 2)) {
    if (debug) Serial.println("[DROP] CRC mismatch");
    return;
  }
  mesh_hdr_t h;
  memcpy(&h, b, sizeof(h));
  if (h.ver != MESH_VER) return;
  if (h.chan != meshChan) return;  // filtro de canal lógico
  if (seenBefore(h.src, h.id)) return;
  markSeen(h.src, h.id);

  int8_t rssi = radio.getRSSI();
  int8_t snr = radio.getSNR();
  updateNode(h.src, rssi, snr);

  if (h.type == PKT_BEACON) {  // BEACON
    Serial.printf("[BEACON] %04X\n", h.src);
    return;
  }
  if (h.type == PKT_ACK) {  // ACK (nunca forward)
    uint16_t acked;
    memcpy(&acked, b + sizeof(mesh_hdr_t), sizeof(uint16_t));
    if (debug) Serial.printf("[ACK RX] from %04X for msg %u\n", h.src, acked);
    for (uint8_t i = 0; i < MAX_PENDING; i++) {
      if (pend[i].id == acked && pend[i].dst == h.src) {
        pend[i].id = 0;
        break;
      }
    }
    return;
  }
  if (h.type == PKT_DATA) {  // DATA
    if (h.type != PKT_DATA) return;
    bool isForMe = (h.dst == nodeID);
    bool isBroadcast = (h.dst == NODE_BCAST);

    // 1. Mostrar el mensaje si es para mí o broadcast
    if (isForMe || isBroadcast) {
      Serial.print("[MSG] ");
      Serial.write(b + sizeof(mesh_hdr_t), h.len);
      Serial.printf("  ← from %04X  dst=%04X  id=%u\n", h.src, h.dst, h.id);
      if (isForMe) sendPkt(PKT_ACK, h.src, (uint8_t*)&h.id, sizeof(h.id));
    }

    // 2. Reenvío SOLO si: tiene TTL restante, NO lo generé yo o
    // - es broadcast (flooding) O todavía no llegó al destino (unicast en ruta)
    if (h.ttl > 0 && h.src != nodeID && (isBroadcast || !isForMe) && shouldForward(rssi, h.ttl)) {
      h.ttl--;

      uint16_t pos = 0;
      memcpy(txBuf + pos, &h, sizeof(h));
      pos += sizeof(h);

      memcpy(txBuf + pos, b + sizeof(mesh_hdr_t), h.len);
      pos += h.len;

      uint16_t crc = crc16_ccitt_false(txBuf, pos);
      memcpy(txBuf + pos, &crc, 2);
      pos += 2;

      delay(random(10, 30) * h.ttl);

      if (debug) Serial.printf("[FWD] src=%04X → dst=%04X ttl=%u len=%u\n",
                               h.src, h.dst, h.ttl, h.len);

      radioTX(txBuf, pos);
    }
  }
}
uint16_t parseNodeID(const char* s) {
  // Parsea como Hex
  // Esto permite: 6019 → 0x6019, 4ca9 → 0x4ca9, 4CA9 → 0x4CA9
  char* endptr;
  uint32_t val = strtoul(s, &endptr, 16);  // strtoul para valores sin signo

  // Validación básica para evitar basura
  if (*endptr != '\0' || val > 0xFFFF) {
    Serial.printf("[ERROR] NodeID inválido: '%s' (esperado 1-4 dígitos hex)\n", s);
    return 0xFFFF;  // o 0, o el valor que prefieras para indicar error
  }

  return (uint16_t)val;
}
/* ================= CLI ================= */
// Gestión de comandos de cliente
void cli(char* l) {
  if (debug) Serial.printf("CLI input: %s\n", l);  // ← Log para depurar si entra
  char* c = strtok(l, " ");
  if (!c) return;
  if (!strcmp(c, "help")) {
    Serial.println("Comandos: \n\tget [all/radio/mesh]\n\tset <key> <val>\n\tsave\n\tload\n\treboot\n\tsend <dst> <msg>\n\tbroadcast <msg>\n\tstatus\n\tnodes\n\tnodes clear");
    return;
  }
  if (!strcmp(c, "get")) {
    char* p = strtok(NULL, " ");
    if (!p || !strcmp(p, "all")) {
      Serial.printf("Node: %04X bauds=%lu freq=%lu sf=%u bw=%u cr=%u ttl=%u beacon=%lu power=%d\n",
                    nodeID, cfg.baud, cfg.freq, cfg.sf, cfg.bw, cfg.cr, cfg.ttl, cfg.beacon_ms, cfg.power);
    } else if (!strcmp(p, "radio")) {
      Serial.printf("freq=%lu sf=%u bw=%u cr=%u power=%d\n",
                    cfg.freq, cfg.sf, cfg.bw, cfg.cr, cfg.power);
    } else if (!strcmp(p, "mesh")) {
      Serial.printf("ttl=%u beacon=%lu chan=%u\n", cfg.ttl, cfg.beacon_ms, meshChan);
    }
  }
  if (!strcmp(c, "set")) {
    char* k = strtok(NULL, " ");
    char* v = strtok(NULL, " ");
    if (!k || !v) return;
    if (!strcmp(k, "freq")) cfg.freq = atol(v);
    else if (!strcmp(k, "sf")) {
      int val = atoi(v);
      if (val < 7 || val > 12) {
        Serial.println("SF 7-12");
        return;
      }
      cfg.sf = val;
    } else if (!strcmp(k, "bw")) {
      int val = atoi(v);
      if (val != 7 && val != 10 && val != 15 && val != 20 && val != 31 && val != 41 && val != 62 && val != 125 && val != 250 && val != 500) {
        Serial.println("BW: 7.8,10.4,15.6,20.8,31.25,41.7,62.5,125,250,500");
        return;
      }
      cfg.bw = val;
    } else if (!strcmp(k, "cr")) {
      int val = atoi(v);
      if (val < 5 || val > 8) {
        Serial.println("CR 5-8");
        return;
      }
      cfg.cr = val;
    } else if (!strcmp(k, "ttl")) {
      int val = atoi(v);
      if (val > 10) {
        Serial.println("TTL max 10");
        return;
      }  // Ejemplo límite
      cfg.ttl = val;
    } else if (!strcmp(k, "beacon")) cfg.beacon_ms = atol(v);
    else if (!strcmp(k, "power")) {
      int val = atoi(v);
      if (val < -9 || val > 22) {
        Serial.println("Power -9 to 22");
        return;
      }
      cfg.power = val;
    } else if (!strcmp(k, "baud")) {
      uint32_t newBaud = atol(v);
      if (newBaud >= 1200 && newBaud <= 2000000) {
        cfg.baud = newBaud;
        Serial.println("Baud set. Reiniciando puerto serie en nuevo baudrate...");
        delay(200);
        Serial.end();
        Serial.begin(cfg.baud);
      } else {
        Serial.println("Baud inválido (1200-2000000)");
      }
    } else if (!strcmp(k, "chan")) {
      int val = atoi(v);
      if (val < 0 || val > 255) {
        Serial.println("chan 0-255");
        return;
      }
      cfg.chan = (uint8_t)val;
      meshChan = cfg.chan;
      Serial.printf("Channel set to %u\n", meshChan);
    } else if (!strcmp(k, "debug")) {
      if (!strcmp(v, "on") || !strcmp(v, "true") || !strcmp(v, "1")) {
        cfg.debug = 1;
        debug = true;
        Serial.println("Debug: ON");
      } else if (!strcmp(v, "off") || !strcmp(v, "false") || !strcmp(v, "0")) {
        cfg.debug = 0;
        debug = false;
        Serial.println("Debug: OFF");
      } else {
        Serial.println("Uso: set debug on|off | true|false | 1|0");
      }
    }
    Serial.println("Param set, apply changes after reboot");
  }
  if (!strcmp(c, "save")) saveConfig();
  if (!strcmp(c, "load")) {
    if (loadConfig()) Serial.println("Config loaded");
    else Serial.println("Failed load");
  }
  if (!strcmp(c, "reboot")) NVIC_SystemReset();
  if (!strcmp(c, "status")) {
    Serial.printf("Node: %04X | Freq: %lu MHz | SF: %u | BW: %u kHz | Power: %d dBm | Debug: %s\n",
                  nodeID, cfg.freq / 1000000, cfg.sf, cfg.bw, cfg.power,
                  debug ? "ON" : "OFF");
  }
  if (!strcmp(c, "nodes")) {
    char* sub = strtok(NULL, " ");
    if (sub && !strcmp(sub, "clear")) {
      nodeCount = 0;
      Serial.println("Node table cleared");
      return;
    } else {
      Serial.printf("Neighbors (%u):\n", nodeCount);
      for (uint8_t i = 0; i < nodeCount; i++) {
        Serial.printf("  %04X  RSSI=%d  SNR=%d  age=%lus\n",
                      nodes[i].id,
                      nodes[i].rssi,
                      nodes[i].snr,
                      (millis() - nodes[i].lastSeen) / 1000);
      }
    }
  }
  if (!strcmp(c, "send")) {
    char* dstStr = strtok(NULL, " ");
    char* msg = strtok(NULL, "");
    if (!dstStr || !msg) {
      Serial.println("Usage: send <NODEID> <message>");
      return;
    }
    uint16_t dst = parseNodeID(dstStr);
    delay(random(1, 10));  // jitter
    sendPkt(PKT_DATA, dst, (uint8_t*)msg, strlen(msg));
  }
  if (!strcmp(c, "broadcast")) {
    char* msg = strtok(NULL, "");
    if (!msg) {
      Serial.println("Usage: broadcast <message>");
      return;
    }
    delay(random(1, 10));  // jitter
    sendPkt(PKT_DATA, NODE_BCAST, (uint8_t*)msg, strlen(msg));
  }
}
/* ================= UTILS ================= */
void processPending() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < MAX_PENDING; i++) {
    if (pend[i].id && now - pend[i].ts > 3000) {
      if (pend[i].retries < 2) {
        resend(pend[i]);
        pend[i].retries++;
        pend[i].ts = now;
      } else {
        pend[i].id = 0;
      }
    }
  }
}
/* ================= SETUP ================= */
void setup() {
  nodeID = genNodeID();

  // Cargar config o defaults
  if (!loadConfig()) {
    cfg.freq = 915000000;
    cfg.baud = UART_SPEED;
    cfg.beacon_ms = 30000;
    cfg.sf = 9;
    cfg.bw = 250;
    cfg.cr = 5;
    cfg.ttl = 4;
    cfg.power = 5;  // Bajo para prueba
    cfg.debug = 0;
    cfg.chan = 0;
    meshChan = 0;
  }
  debug = (cfg.debug != 0);
  meshChan = cfg.chan;

  pinMode(LED_PIN, OUTPUT);
  pinMode(LORA_NRST, OUTPUT);  // Estado RF seguro al arranque: RX

  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(cfg.baud);
  while (!Serial)
    ;  // quitar en producción
  SPI.setMOSI(LORA_MOSI);
  SPI.setMISO(LORA_MISO);
  SPI.setSCLK(LORA_SCK);
  SPI.begin();
  // Reset SX1262
  digitalWrite(LORA_NRST, LOW);
  delay(10);
  digitalWrite(LORA_NRST, HIGH);
  delay(20);

  int state = radio.begin(
    cfg.freq / 1000000.0,
    (float)cfg.bw,
    cfg.sf,
    cfg.cr,
    0x12,
    cfg.power,
    8,
    0.0);

  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("begin failed: ");
    Serial.println(state);
    while (true) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  // Config post-init
  radio.setDio2AsRfSwitch(true);
  radio.setSyncWord(0x34);
  radio.setCurrentLimit(80);
  radio.setRxBoostedGainMode(true);
  radio.setPacketSentAction(onTx);
  radio.setPacketReceivedAction(onRx);

  lastBeacon = millis() + random(500, 5000);
  radio.startReceive();
  Serial.printf("MeshLoRa node: %04X\n", nodeID);
}
/* ================= LOOP ================= */
void loop() {
  static char cliBuf[128];
  static uint8_t cliLen = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      cliBuf[cliLen] = 0;
      cli(cliBuf);
      cliLen = 0;
    } else if (cliLen < 127) cliBuf[cliLen++] = c;
  }

  if (millis() - lastBeacon > cfg.beacon_ms) {
    lastBeacon = millis();
    uint8_t d = 0;
    sendPkt(PKT_BEACON, NODE_BCAST, &d, 1);
  }

  if (rxFlag) {
    rxFlag = false;
    // Obtener longitud ANTES de leer
    size_t len = radio.getPacketLength(true);  // true = descartar inválidos automáticamente
    if (debug) Serial.printf("[RX IRQ] len detectada = %d\n", len);

    if (len > 0 && len < MAX_PKT) {
      int err = radio.readData(rxBuf, len);
      if (debug) Serial.printf("[RX READ] err = %d, len = %d\n", err, len);

      if (err == RADIOLIB_ERR_NONE) {
        if (debug) {
          Serial.println("[RX OK] Paquete válido!");
          Serial.printf("[RX] RSSI: %.1f dBm  SNR: %.1f dB\n", radio.getRSSI(), radio.getSNR());
        }
        // Procesamos el paquete
        handlePkt(rxBuf, len);
      } else if (debug) {
        Serial.printf("[RX ERR] %d\n", err);
      }
    } else if (debug) {
      Serial.println("[RX] len = 0 - paquete detectado pero inválido/droppeado");
    }
    radio.startReceive();  // vuelve a RX continuo
  }

  if (txFlag) {
    txFlag = false;
    // Esperar que el chip realmente termine (BUSY baja)
    uint32_t timeout = millis() + 100;
    while (digitalRead(LORA_BUSY) && millis() < timeout) {
      delay(1);
    }
    int state = radio.finishTransmit();
    if (state != RADIOLIB_ERR_NONE && debug) {
      Serial.printf("[TX finish failed] %d\n", state);
    }
    radio.standby();
    radio.setRxBoostedGainMode(true);
    radio.startReceive();
  }

  if (millis() - lastPrune > 5000) {
    pruneNodes();
    lastPrune = millis();
  }
}
