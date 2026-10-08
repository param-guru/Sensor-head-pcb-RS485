#include <Arduino.h>
#include "driver/pcnt.h"

// ---------- Pins ----------
#define PIN_FREQ      4    // from SN74HC14 output
#define PIN_DE_RE     6    // MAX485 DE + RE (tied together)
#define PIN_RS485_TX  17   // -> MAX485 DI
#define PIN_RS485_RX  18   // <- MAX485 RO

// ---------- Settings ----------
#define RS485_BAUD     9600
#define GATE_MS        500
#define PCNT_UNIT_USED PCNT_UNIT_0
#define PCNT_LIMIT     20000
#define PCNT_FILTER    100   // 0 = off; 100 = ~1.25 us. Tune after the sweep test

volatile uint32_t overflowCount = 0;
static uint32_t seq        = 0;
static uint64_t lastTotal  = 0;
static int64_t  lastTimeUs = 0;
static uint32_t lastGateMs = 0;

static void IRAM_ATTR pcntOverflowISR(void *arg) { overflowCount++; }

static void pcntSetup() {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = PIN_FREQ;
  cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  cfg.channel        = PCNT_CHANNEL_0;
  cfg.unit           = PCNT_UNIT_USED;
  cfg.pos_mode       = PCNT_COUNT_INC;
  cfg.neg_mode       = PCNT_COUNT_DIS;
  cfg.lctrl_mode     = PCNT_MODE_KEEP;
  cfg.hctrl_mode     = PCNT_MODE_KEEP;
  cfg.counter_h_lim  = PCNT_LIMIT;
  cfg.counter_l_lim  = 0;
  pcnt_unit_config(&cfg);

#if PCNT_FILTER > 0
  pcnt_set_filter_value(PCNT_UNIT_USED, PCNT_FILTER);
  pcnt_filter_enable(PCNT_UNIT_USED);
#else
  pcnt_filter_disable(PCNT_UNIT_USED);
#endif

  pcnt_event_enable(PCNT_UNIT_USED, PCNT_EVT_H_LIM);
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT_USED, pcntOverflowISR, NULL);
  pcnt_counter_pause(PCNT_UNIT_USED);
  pcnt_counter_clear(PCNT_UNIT_USED);
  pcnt_counter_resume(PCNT_UNIT_USED);
}

static uint64_t getTotalCount() {
  uint32_t o1, o2; int16_t c;
  do {
    o1 = overflowCount;
    pcnt_get_counter_value(PCNT_UNIT_USED, &c);
    o2 = overflowCount;
  } while (o1 != o2);
  return (uint64_t)o1 * PCNT_LIMIT + (uint16_t)c;
}

static uint8_t xorChecksum(const char *s) {
  uint8_t cs = 0;
  while (*s) cs ^= (uint8_t)*s++;
  return cs;
}

static void rs485Send(const char *line) {
  digitalWrite(PIN_DE_RE, HIGH);   // driver ON
  delayMicroseconds(200);
  Serial1.print(line);
  Serial1.flush();
  delayMicroseconds(500);          // let the last stop bit leave the line
  digitalWrite(PIN_DE_RE, LOW);    // driver OFF
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(PIN_DE_RE, OUTPUT);
  digitalWrite(PIN_DE_RE, LOW);
  Serial1.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
  pcntSetup();
  lastTotal  = getTotalCount();
  lastTimeUs = esp_timer_get_time();
  lastGateMs = millis();
  Serial.printf("Sensor head started, PCNT filter=%d\n", PCNT_FILTER);
}

void loop() {
  if (millis() - lastGateMs < GATE_MS) return;
  lastGateMs += GATE_MS;

  uint64_t total  = getTotalCount();
  int64_t  nowUs  = esp_timer_get_time();
  uint32_t pulses = (uint32_t)(total - lastTotal);
  uint32_t gateUs = (uint32_t)(nowUs - lastTimeUs);
  double   freqHz = (double)pulses * 1e6 / (double)gateUs;
  lastTotal  = total;
  lastTimeUs = nowUs;

  char body[48], frame[64];
  uint32_t s = seq++;
  snprintf(body, sizeof(body), "FUEL,%lu,%.1f", (unsigned long)s, freqHz);
  snprintf(frame, sizeof(frame), "$%s*%02X\n", body, xorChecksum(body));

  Serial.printf("#%lu pulses=%lu gate=%lu us freq=%.1f Hz | %s",
                (unsigned long)s, (unsigned long)pulses,
                (unsigned long)gateUs, freqHz, frame);
  rs485Send(frame);
}
