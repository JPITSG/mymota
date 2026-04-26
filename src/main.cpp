#include <Arduino.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <Updater.h>
#include <WiFiUdp.h>
#include <math.h>
#include <stddef.h>

extern "C" {
#include <user_interface.h>
}

extern "C" uint32_t _EEPROM_start;

#ifndef MYMOTA_VERSION
#define MYMOTA_VERSION "dev"
#endif

#ifndef MYMOTA_TARGET
#define MYMOTA_TARGET "esp8266"
#endif

namespace {

constexpr uint32_t kConfigMagic = 0x4d594d4f;  // MYMO
constexpr uint32_t kBootRecoveryMagic = 0x4d595246;  // MYRF
constexpr uint32_t kEnergyJournalMagic = 0x4d59454a;  // MYEJ
constexpr uint16_t kConfigVersionV1 = 1;
constexpr uint16_t kConfigVersionV2 = 2;
constexpr uint16_t kConfigVersionV3 = 3;
constexpr uint16_t kConfigVersionV4 = 4;
constexpr uint16_t kConfigVersionV5 = 5;
constexpr uint16_t kConfigVersionV6 = 6;
constexpr uint16_t kConfigVersionV7 = 7;
constexpr uint16_t kConfigVersionV8 = 8;
constexpr uint16_t kConfigVersionV9 = 9;
constexpr uint16_t kConfigVersion = 10;
constexpr size_t kEepromSize = 4096;
constexpr size_t kFlashSectorSize = 4096;
constexpr uint8_t kEnergyJournalSectorCount = 2;
constexpr size_t kBootRecoveryOffset = 3072;
constexpr uint32_t kConnectTimeoutMs = 20000;
constexpr uint32_t kReconnectStartApMs = 90000;
constexpr uint32_t kApRetryMs = 10000;
constexpr uint32_t kBootRecoveryStableMs = 30000;
constexpr uint32_t kBootRecoveryBootMarker = 0x4d594254;  // MYBT
constexpr uint32_t kBootRecoveryStableMarker = kBootRecoveryMagic;
constexpr uint32_t kBootRecoveryEmptyMarker = 0xffffffffUL;
constexpr uint8_t kPhyModeAuto = 0;
constexpr uint8_t kPhyModeFailsafe = WIFI_PHY_MODE_11G;
constexpr uint8_t kBootRecoveryLimit = 5;
constexpr size_t kTemplateSlotCount = 14;
constexpr size_t kTemplateJsonMaxLen = 640;
constexpr size_t kTemplateJsonDocCapacity = 1024;
constexpr size_t kMqttHostMaxLen = 64;
constexpr size_t kMqttTopicMaxLen = 150;
constexpr uint16_t kMqttDefaultPort = 1883;
constexpr uint16_t kMqttKeepaliveMax = 65535U;
constexpr uint16_t kMqttProtocolKeepaliveSec = 30;
constexpr uint32_t kMqttReconnectMs = 5000;
constexpr uint32_t kMqttConnectTimeoutMs = 650;
constexpr uint32_t kMqttConnackTimeoutMs = 250;
constexpr uint32_t kMqttIoTimeoutMs = 250;
constexpr uint32_t kMqttInboundReadTimeoutMs = 20;
constexpr uint32_t kMqttBrokerSilenceTimeoutMs = static_cast<uint32_t>(kMqttProtocolKeepaliveSec) * 2000UL;
constexpr uint32_t kMqttConnackMaxRemainingLength = 2;
constexpr uint8_t kMqttInboundPacketLimit = 4;
constexpr uint16_t kMqttEnergyIntervalMax = 65535U;
constexpr float kMqttEnergyChangeMaxPercent = 1000.0f;
constexpr uint8_t kMqttPacketConnack = 0x20;
constexpr uint8_t kMqttPacketPingreq = 0xc0;
constexpr uint8_t kMqttPacketPingresp = 0xd0;
constexpr uint8_t kInvalidPin = 0xff;
constexpr uint8_t kAdc0Pin = 17;
constexpr uint8_t kMaxRelays = 8;
constexpr uint8_t kMaxButtons = 4;
constexpr uint8_t kMaxLeds = 4;
constexpr uint8_t kMaxLedOutputs = kMaxLeds + 1;
constexpr uint16_t kButtonDebounceDefaultMs = 50;
constexpr uint16_t kButtonDebounceMinMs = 5;
constexpr uint16_t kButtonDebounceMaxMs = 200;
constexpr uint16_t kButtonHoldDefaultMs = 1000;
constexpr uint16_t kButtonHoldMinMs = 1;
constexpr uint16_t kButtonHoldMaxMs = 60000;
constexpr uint32_t kLedUpdateMs = 250;
constexpr uint32_t kAdcUpdateMs = 2000;
constexpr uint32_t kEnergyUpdateMs = 200;
constexpr uint32_t kEnergyIntegrateMs = 1000;
constexpr uint32_t kEnergyReportBootSettleMs = 5000;
constexpr uint16_t kEnergyJournalVersion = 1;
constexpr uint32_t kEnergyPersistMinMs = 600000;
constexpr uint64_t kEnergyPersistDeltaUkwh = 10000;
constexpr uint64_t kEnergyTotalMaxUkwh = 1000000000000ULL;
constexpr float kEnergyTotalOffsetMinKwh = 0.0f;
constexpr float kEnergyTotalOffsetMaxKwh = 1000000.0f;
constexpr uint8_t kLedAttachNone = 0;
constexpr uint8_t kLedAttachRelayBase = 1;
constexpr uint8_t kLedAttachButtonBase = 33;
constexpr uint8_t kButtonActionNone = 0;
constexpr uint8_t kButtonActionRelayToggle = 1;
constexpr uint8_t kButtonActionMqtt = 2;
constexpr uint8_t kButtonActionWebhook = 3;
constexpr size_t kButtonActionTargetMaxLen = 128;
constexpr size_t kButtonActionPayloadMaxLen = 128;
constexpr uint8_t kMqttButtonQueueDepth = 4;
constexpr size_t kMqttButtonTopicMaxLen = kButtonActionTargetMaxLen + kMqttTopicMaxLen + 16;
constexpr size_t kMqttButtonPayloadMaxLen = kButtonActionPayloadMaxLen + kMqttTopicMaxLen + 24;
constexpr uint32_t kMqttButtonQueueMaxAgeMs = 5000;
constexpr uint32_t kWebhookConnectTimeoutMs = 500;
constexpr uint32_t kWebhookFlushTimeoutMs = 50;
constexpr uint32_t kWebhookStopTimeoutMs = 25;
constexpr size_t kHtmlStreamChunkReserve = 5200;
constexpr size_t kJsonStreamChunkReserve = 900;
constexpr const char *kDefaultButtonMqttTopic = "stat/{TOPIC}/RESULT";
constexpr const char *kDefaultButtonMqttPressPayload = "{\"Switch{BUTTONID}\":{\"Action\":\"{TYPE}\"}}";
constexpr const char *kDefaultButtonMqttHoldPayload = "{\"Switch{BUTTONID}\":{\"Action\":\"{TYPE}\"}}";
constexpr uint8_t kMqttConnectIdle = 0;
constexpr uint8_t kMqttConnectOk = 1;
constexpr uint8_t kMqttConnectTcpFailed = 2;
constexpr uint8_t kMqttConnectWriteFailed = 3;
constexpr uint8_t kMqttConnectConnackTimeout = 4;
constexpr uint8_t kMqttConnectConnackRejected = 5;

constexpr uint16_t kTplNone = 0;
constexpr uint16_t kTplUser = 1;
constexpr uint16_t kTplKey1 = 32;
constexpr uint16_t kTplKey1Np = 64;
constexpr uint16_t kTplKey1Inv = 96;
constexpr uint16_t kTplKey1InvNp = 128;
constexpr uint16_t kTplRel1 = 224;
constexpr uint16_t kTplRel1Inv = 256;
constexpr uint16_t kTplLed1 = 288;
constexpr uint16_t kTplLed1Inv = 320;
constexpr uint16_t kTplLedLnk = 544;
constexpr uint16_t kTplLedLnkInv = 576;
constexpr uint16_t kTplNrgSel = 2592;
constexpr uint16_t kTplNrgSelInv = 2624;
constexpr uint16_t kTplNrgCf1 = 2656;
constexpr uint16_t kTplHlwCf = 2688;
constexpr uint16_t kTplHjlCf = 2720;
constexpr uint16_t kTplAdcTemp = 4736;

const char kTemplateShellyPlugSJson[] PROGMEM =
  "{\"NAME\":\"Shelly Plug S\",\"GPIO\":[320,1,576,1,1,2720,0,0,2624,32,2656,224,1,4736],\"FLAG\":0,\"BASE\":45}";
const char kTemplateNousA1TJson[] PROGMEM =
  "{\"NAME\":\"NOUS A1T\",\"GPIO\":[32,0,0,0,2720,2656,0,0,2624,320,224,0,0,0],\"FLAG\":0,\"BASE\":18}";

constexpr uint32_t kHlwPowerCal = 12530;
constexpr uint32_t kHlwVoltageCal = 1950;
constexpr uint32_t kHlwCurrentCal = 3500;
constexpr uint32_t kHlwPowerRatio = 10000;
constexpr uint32_t kHlwVoltageRatio = 2200;
constexpr uint32_t kHlwCurrentRatio = 4545;
constexpr uint32_t kHjlPowerRatio = 1362;
constexpr uint32_t kHjlVoltageRatio = 822;
constexpr uint32_t kHjlCurrentRatio = 3300;
constexpr uint32_t kEnergyNoPulseTimeoutUs = 10000000UL;
constexpr uint8_t kEnergyCf1SampleCount = 10;
constexpr float kAnalogT0Kelvin = 298.15f;
constexpr float kAnalogNtcBridgeResistance = 32000.0f;
constexpr float kAnalogNtcResistance = 10000.0f;
constexpr float kAnalogNtcBeta = 3350.0f;

const uint8_t kTemplateSlotToPin[kTemplateSlotCount] = {
  0, 1, 2, 3, 4, 5, 9, 10, 12, 13, 14, 15, 16, kAdc0Pin
};

struct ConfigHeader {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
};

struct StoredConfigV1 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char ssid[33];
  char password[65];
  char hostname[33];
  uint32_t crc;
};

struct StoredConfigV2 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char ssid[33];
  char password[65];
  char hostname[33];
  uint8_t phy_mode;
  uint8_t reserved[3];
  uint32_t crc;
};

struct StoredConfigV3 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char ssid[33];
  char password[65];
  char hostname[33];
  uint8_t phy_mode;
  uint8_t template_enabled;
  uint16_t template_base;
  uint32_t template_flag;
  char template_name[33];
  uint16_t template_gpio[kTemplateSlotCount];
  uint8_t reserved[3];
  uint32_t crc;
};

struct StoredConfigV4 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char ssid[33];
  char password[65];
  char hostname[33];
  uint8_t phy_mode;
  uint8_t template_enabled;
  uint16_t template_base;
  uint32_t template_flag;
  char template_name[33];
  uint16_t template_gpio[kTemplateSlotCount];
  uint16_t mqtt_port;
  uint16_t mqtt_keepalive;
  char mqtt_host[kMqttHostMaxLen + 1];
  char mqtt_topic[kMqttTopicMaxLen + 1];
  uint8_t reserved[18];
  uint32_t crc;
};

struct StoredConfigV5 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char ssid[33];
  char password[65];
  char hostname[33];
  uint8_t phy_mode;
  uint8_t template_enabled;
  uint16_t template_base;
  uint32_t template_flag;
  char template_name[33];
  uint16_t template_gpio[kTemplateSlotCount];
  uint16_t mqtt_port;
  uint16_t mqtt_keepalive;
  char mqtt_host[kMqttHostMaxLen + 1];
  char mqtt_topic[kMqttTopicMaxLen + 1];
  float energy_total_offset_kwh;
  uint8_t reserved[14];
  uint32_t crc;
};

struct StoredConfigV6 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char ssid[33];
  char password[65];
  char hostname[33];
  uint8_t phy_mode;
  uint8_t template_enabled;
  uint16_t template_base;
  uint32_t template_flag;
  char template_name[33];
  uint16_t template_gpio[kTemplateSlotCount];
  uint16_t mqtt_port;
  uint16_t mqtt_keepalive;
  char mqtt_host[kMqttHostMaxLen + 1];
  char mqtt_topic[kMqttTopicMaxLen + 1];
  float energy_total_offset_kwh;
  uint8_t led_attach[kMaxLedOutputs];
  uint8_t reserved[9];
  uint32_t crc;
};

struct StoredConfigV7 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char ssid[33];
  char password[65];
  char hostname[33];
  uint8_t phy_mode;
  uint8_t template_enabled;
  uint16_t template_base;
  uint32_t template_flag;
  char template_name[33];
  uint16_t template_gpio[kTemplateSlotCount];
  uint16_t mqtt_port;
  uint16_t mqtt_keepalive;
  char mqtt_host[kMqttHostMaxLen + 1];
  char mqtt_topic[kMqttTopicMaxLen + 1];
  float energy_total_offset_kwh;
  uint8_t led_attach[kMaxLedOutputs];
  uint16_t button_hold_ms;
  uint8_t button_press_action[kMaxButtons];
  uint8_t button_hold_action[kMaxButtons];
  uint32_t crc;
};

struct StoredConfigV8 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char ssid[33];
  char password[65];
  char hostname[33];
  uint8_t phy_mode;
  uint8_t template_enabled;
  uint16_t template_base;
  uint32_t template_flag;
  char template_name[33];
  uint16_t template_gpio[kTemplateSlotCount];
  uint16_t mqtt_port;
  uint16_t mqtt_keepalive;
  char mqtt_host[kMqttHostMaxLen + 1];
  char mqtt_topic[kMqttTopicMaxLen + 1];
  float energy_total_offset_kwh;
  uint8_t led_attach[kMaxLedOutputs];
  uint16_t button_hold_ms;
  uint8_t button_press_action[kMaxButtons];
  uint8_t button_hold_action[kMaxButtons];
  uint16_t energy_mqtt_interval;
  uint16_t energy_mqtt_change_percent_x10;
  uint32_t crc;
};

struct StoredConfigV9 {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char ssid[33];
  char password[65];
  char hostname[33];
  uint8_t phy_mode;
  uint8_t template_enabled;
  uint16_t template_base;
  uint32_t template_flag;
  char template_name[33];
  uint16_t template_gpio[kTemplateSlotCount];
  uint16_t mqtt_port;
  uint16_t mqtt_keepalive;
  char mqtt_host[kMqttHostMaxLen + 1];
  char mqtt_topic[kMqttTopicMaxLen + 1];
  float energy_total_offset_kwh;
  uint8_t led_attach[kMaxLedOutputs];
  uint16_t button_hold_ms;
  uint8_t button_press_action[kMaxButtons];
  uint8_t button_hold_action[kMaxButtons];
  uint16_t energy_mqtt_interval;
  uint16_t energy_mqtt_change_percent_x10;
  char button_press_target[kMaxButtons][kButtonActionTargetMaxLen + 1];
  char button_press_payload[kMaxButtons][kButtonActionPayloadMaxLen + 1];
  char button_hold_target[kMaxButtons][kButtonActionTargetMaxLen + 1];
  char button_hold_payload[kMaxButtons][kButtonActionPayloadMaxLen + 1];
  uint32_t crc;
};

struct StoredConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char ssid[33];
  char password[65];
  char hostname[33];
  uint8_t phy_mode;
  uint8_t template_enabled;
  uint16_t template_base;
  uint32_t template_flag;
  char template_name[33];
  uint16_t template_gpio[kTemplateSlotCount];
  uint16_t mqtt_port;
  uint16_t mqtt_keepalive;
  char mqtt_host[kMqttHostMaxLen + 1];
  char mqtt_topic[kMqttTopicMaxLen + 1];
  float energy_total_offset_kwh;
  uint8_t led_attach[kMaxLedOutputs];
  uint16_t button_hold_ms;
  uint8_t button_press_action[kMaxButtons];
  uint8_t button_hold_action[kMaxButtons];
  uint16_t energy_mqtt_interval;
  uint16_t energy_mqtt_change_percent_x10;
  char button_press_target[kMaxButtons][kButtonActionTargetMaxLen + 1];
  char button_press_payload[kMaxButtons][kButtonActionPayloadMaxLen + 1];
  char button_hold_target[kMaxButtons][kButtonActionTargetMaxLen + 1];
  char button_hold_payload[kMaxButtons][kButtonActionPayloadMaxLen + 1];
  uint16_t button_debounce_ms;
  uint32_t crc;
};

constexpr size_t kBootRecoveryLogWords = (kEepromSize - kBootRecoveryOffset) / sizeof(uint32_t);

static_assert(sizeof(StoredConfig) <= kEepromSize, "StoredConfig exceeds EEPROM size");
static_assert(sizeof(StoredConfig) <= kBootRecoveryOffset, "StoredConfig overlaps boot recovery state");
static_assert(kBootRecoveryOffset % sizeof(uint32_t) == 0, "Boot recovery log must be word aligned");
static_assert(kBootRecoveryLogWords >= kBootRecoveryLimit * 2, "Boot recovery log is too small");
static_assert(sizeof(kTemplateSlotToPin) == kTemplateSlotCount, "Template pin map size mismatch");

struct PinAssignment {
  uint8_t pin;
  bool inverted;
  bool no_pullup;
};

struct ButtonState {
  bool raw_pressed;
  bool stable_pressed;
  bool hold_emitted;
  uint32_t changed_at;
  uint32_t pressed_at;
};

struct RuntimeTemplate {
  bool enabled;
  char name[33];
  uint16_t base;
  uint32_t flag;
  PinAssignment relays[kMaxRelays];
  PinAssignment buttons[kMaxButtons];
  PinAssignment leds[kMaxLeds];
  PinAssignment link_led;
  uint8_t relay_count;
  uint8_t button_count;
  uint8_t led_count;
  uint8_t energy_cf_pin;
  uint8_t energy_cf1_pin;
  uint8_t energy_sel_pin;
  bool energy_sel_inverted;
  bool energy_hjl;
  bool adc_temp;
  uint8_t unsupported_count;
  uint8_t unsupported_pin[8];
  uint16_t unsupported_code[8];
};

struct EnergyState {
  bool present;
  bool hjl;
  uint8_t cf_pin;
  uint8_t cf1_pin;
  uint8_t sel_pin;
  bool select_ui_flag;
  bool ui_flag;
  volatile bool load_off;
  volatile uint32_t cf_pulse_length;
  volatile uint32_t cf_pulse_last_time;
  volatile uint32_t cf_summed_pulse_length;
  volatile uint32_t cf_pulse_counter;
  uint32_t cf_power_pulse_length;
  volatile uint32_t cf1_pulse_length;
  volatile uint32_t cf1_pulse_last_time;
  volatile uint32_t cf1_summed_pulse_length;
  volatile uint32_t cf1_pulse_counter;
  uint32_t cf1_voltage_pulse_length;
  uint32_t cf1_current_pulse_length;
  volatile uint8_t cf1_timer;
  uint32_t power_ratio;
  uint32_t voltage_ratio;
  uint32_t current_ratio;
  uint32_t last_update_ms;
  uint32_t last_integrate_ms;
  uint8_t power_retry;
  float voltage;
  float current;
  float power;
  float total_kwh;
};

struct EnergyJournalRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t sequence;
  uint64_t total_ukwh;
  uint32_t crc;
  uint32_t reserved;
};

static_assert(sizeof(EnergyJournalRecord) % sizeof(uint32_t) == 0, "Energy journal records must be word aligned");

ESP8266WebServer server(80);
WiFiClient mqtt_client;
StoredConfig config{};
RuntimeTemplate runtime_template{};
ButtonState button_state[kMaxButtons]{};
EnergyState energy{};

bool config_ok = false;
bool ap_started = false;
bool update_started = false;
bool update_ok = false;
bool update_mqtt_paused = false;
uint8_t update_error = UPDATE_ERROR_OK;
uint32_t update_max_size = 0;
uint32_t restart_at = 0;
bool restart_pending = false;
uint32_t disconnected_since = 0;
bool disconnected_timer_active = false;
uint32_t last_ap_attempt = 0;
uint32_t last_led_update = 0;
uint32_t last_adc_update = 0;
uint32_t next_mqtt_reconnect = 0;
uint32_t last_mqtt_io = 0;
uint32_t last_mqtt_rx = 0;
uint32_t last_mqtt_ping = 0;
uint32_t last_mqtt_state_publish = 0;
uint32_t last_mqtt_energy_publish = 0;
uint32_t last_mqtt_connect_attempt = 0;
uint32_t last_mqtt_connect_duration = 0;
float last_mqtt_energy_power = NAN;
uint16_t mqtt_pending_relay_mask = 0;
struct MqttButtonPending {
  uint32_t queued_at;
  char topic[kMqttButtonTopicMaxLen + 1];
  char payload[kMqttButtonPayloadMaxLen + 1];
};
MqttButtonPending mqtt_button_queue[kMqttButtonQueueDepth]{};
uint8_t mqtt_button_queue_head = 0;
uint8_t mqtt_button_queue_count = 0;
uint8_t last_mqtt_connect_result = kMqttConnectIdle;
bool mqtt_ping_pending = false;
uint32_t boot_id = 0;
uint8_t boot_recovery_count = 0;
bool boot_recovery_armed = false;
bool boot_recovery_factory_reset = false;
bool relay_state[kMaxRelays]{};
bool energy_report_boot_settled = false;
bool energy_journal_loaded = false;
uint8_t energy_journal_sector = 0;
uint8_t energy_journal_next_slot = 0;
uint32_t energy_journal_sequence = 0;
uint32_t last_energy_persist_ms = 0;
uint64_t energy_journal_saved_ukwh = 0;
bool energy_persist_requested = false;
float adc_temperature_c = NAN;
uint16_t adc_raw = 0;
uint32_t perf_window_start_ms = 0;
bool perf_window_started = false;
uint32_t perf_loop_count = 0;
uint32_t perf_busy_us = 0;
uint32_t perf_max_loop_us = 0;
uint32_t perf_last_loop_hz = 0;
uint8_t perf_last_loop_load = 0;
uint32_t perf_last_loop_max_us = 0;

bool hasPin(const PinAssignment &assignment);
bool mqttEnergyReportingEnabled();

void recordLoopPerf(uint32_t started_us, uint32_t ended_us) {
  const uint32_t now_ms = millis();
  const uint32_t elapsed_us = ended_us - started_us;
  if (!perf_window_started) {
    perf_window_start_ms = now_ms;
    perf_window_started = true;
  }

  perf_loop_count++;
  perf_busy_us += elapsed_us;
  if (elapsed_us > perf_max_loop_us) perf_max_loop_us = elapsed_us;

  const uint32_t window_ms = now_ms - perf_window_start_ms;
  if (window_ms < 1000) return;

  perf_last_loop_hz = (perf_loop_count * 1000UL) / window_ms;
  const uint32_t window_us = window_ms * 1000UL;
  uint32_t load = 0;
  if (window_us > 0) {
    load = static_cast<uint32_t>((static_cast<uint64_t>(perf_busy_us) * 100ULL) / window_us);
    if (load > 100) load = 100;
  }
  perf_last_loop_load = static_cast<uint8_t>(load);
  perf_last_loop_max_us = perf_max_loop_us;

  perf_window_start_ms = now_ms;
  perf_loop_count = 0;
  perf_busy_us = 0;
  perf_max_loop_us = 0;
}

uint32_t fnv1a(const uint8_t *data, size_t len) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < len; i++) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

uint32_t fnv1aUpdate(uint32_t hash, uint8_t value) {
  hash ^= value;
  hash *= 16777619UL;
  return hash;
}

template <typename ConfigT>
uint32_t configCrc(const ConfigT &cfg) {
  const uint8_t *data = reinterpret_cast<const uint8_t *>(&cfg);
  const size_t crc_offset = offsetof(ConfigT, crc);
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < sizeof(ConfigT); i++) {
    const bool crc_byte = i >= crc_offset && i < crc_offset + sizeof(cfg.crc);
    hash = fnv1aUpdate(hash, crc_byte ? 0 : data[i]);
  }
  return hash;
}

uint8_t sanitizePhyMode(uint8_t mode) {
  if (mode > WIFI_PHY_MODE_11N) return kPhyModeAuto;
  return mode;
}

const __FlashStringHelper *phyModeName(uint8_t mode) {
  switch (mode) {
    case WIFI_PHY_MODE_11B: return F("11b");
    case WIFI_PHY_MODE_11G: return F("11g");
    case WIFI_PHY_MODE_11N: return F("11n");
    default: return F("auto");
  }
}

String chipIdHex() {
  char buf[9];
  snprintf(buf, sizeof(buf), "%06X", ESP.getChipId());
  return String(buf);
}

String defaultHostname() {
  return "mymota-" + chipIdHex();
}

String defaultMqttTopic() {
  return "mymota_" + chipIdHex();
}

void setDefaultMqttConfig() {
  memset(config.mqtt_host, 0, sizeof(config.mqtt_host));
  config.mqtt_port = kMqttDefaultPort;
  config.mqtt_keepalive = 0;
  strlcpy(config.mqtt_topic, defaultMqttTopic().c_str(), sizeof(config.mqtt_topic));
}

void setDefaultEnergyMqttConfig() {
  config.energy_mqtt_interval = 0;
  config.energy_mqtt_change_percent_x10 = 0;
}

uint8_t defaultLedAttachment(uint8_t led) {
  if (led < kMaxLeds && led < kMaxRelays) {
    return kLedAttachRelayBase + led;
  }
  return kLedAttachNone;
}

void setDefaultLedConfig() {
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    config.led_attach[i] = defaultLedAttachment(i);
  }
}

void setDefaultButtonActionText(uint8_t button) {
  if (button >= kMaxButtons) return;
  strlcpy(config.button_press_target[button], kDefaultButtonMqttTopic, sizeof(config.button_press_target[button]));
  strlcpy(config.button_press_payload[button], kDefaultButtonMqttPressPayload, sizeof(config.button_press_payload[button]));
  strlcpy(config.button_hold_target[button], kDefaultButtonMqttTopic, sizeof(config.button_hold_target[button]));
  strlcpy(config.button_hold_payload[button], kDefaultButtonMqttHoldPayload, sizeof(config.button_hold_payload[button]));
}

void setDefaultButtonActionTexts() {
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    setDefaultButtonActionText(i);
  }
}

void setDefaultButtonConfig() {
  config.button_hold_ms = kButtonHoldDefaultMs;
  config.button_debounce_ms = kButtonDebounceDefaultMs;
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    config.button_press_action[i] = kButtonActionRelayToggle;
    config.button_hold_action[i] = kButtonActionNone;
  }
  setDefaultButtonActionTexts();
}

bool isLedAttachmentEncoding(uint8_t value) {
  if (value == kLedAttachNone) return true;
  if (value >= kLedAttachRelayBase && value < kLedAttachRelayBase + kMaxRelays) return true;
  if (value >= kLedAttachButtonBase && value < kLedAttachButtonBase + kMaxButtons) return true;
  return false;
}

bool ledAttachmentRelayIndex(uint8_t value, uint8_t &index) {
  if (value < kLedAttachRelayBase || value >= kLedAttachRelayBase + kMaxRelays) return false;
  index = value - kLedAttachRelayBase;
  return true;
}

bool ledAttachmentButtonIndex(uint8_t value, uint8_t &index) {
  if (value < kLedAttachButtonBase || value >= kLedAttachButtonBase + kMaxButtons) return false;
  index = value - kLedAttachButtonBase;
  return true;
}

bool isButtonActionEncoding(uint8_t value) {
  return value == kButtonActionNone ||
         value == kButtonActionRelayToggle ||
         value == kButtonActionMqtt ||
         value == kButtonActionWebhook;
}

void setDefaultConfig() {
  memset(&config, 0, sizeof(config));
  config.magic = kConfigMagic;
  config.version = kConfigVersion;
  config.size = sizeof(StoredConfig);
  config.phy_mode = kPhyModeAuto;
  strlcpy(config.hostname, defaultHostname().c_str(), sizeof(config.hostname));
  setDefaultMqttConfig();
  setDefaultEnergyMqttConfig();
  setDefaultLedConfig();
  setDefaultButtonConfig();
  config.crc = configCrc(config);
  config_ok = false;
}

bool commitConfig(bool force_commit = false);

void scheduleRestart(uint32_t delay_ms) {
  restart_at = millis() + delay_ms;
  restart_pending = true;
}

bool restartDue() {
  return restart_pending && static_cast<int32_t>(millis() - restart_at) >= 0;
}

uint32_t makeBootId() {
  uint32_t id = ESP.random();
  id ^= ESP.getCycleCount();
  id ^= micros();
  id ^= (millis() << 16) | (millis() >> 16);
  id ^= ESP.getChipId() * 2654435761UL;
  id ^= ESP.getFreeHeap();
  id ^= system_get_rtc_time();
  return id ? id : (ESP.getChipId() ^ 0xa5a5a5a5UL);
}

uint32_t eepromFlashBase() {
  return ((reinterpret_cast<uint32_t>(&_EEPROM_start) - 0x40200000UL) / kFlashSectorSize) * kFlashSectorSize;
}

uint32_t energyJournalFlashBase() {
  // Use the last two filesystem-reserved sectors immediately before EEPROM.
  // myMota does not mount a filesystem, and OTA sketch writes do not touch this area.
  return eepromFlashBase() - (static_cast<uint32_t>(kEnergyJournalSectorCount) * kFlashSectorSize);
}

uint32_t energyJournalSectorAddress(uint8_t sector) {
  return energyJournalFlashBase() + (static_cast<uint32_t>(sector) * kFlashSectorSize);
}

uint32_t energyJournalSlotAddress(uint8_t sector, uint8_t slot) {
  return energyJournalSectorAddress(sector) + (static_cast<uint32_t>(slot) * sizeof(EnergyJournalRecord));
}

uint8_t energyJournalRecordCount() {
  return kFlashSectorSize / sizeof(EnergyJournalRecord);
}

float ukwhToKwh(uint64_t value) {
  return static_cast<float>(value) / 1000000.0f;
}

uint64_t energyTotalUkwh() {
  if (energy.total_kwh <= 0.0f || isnan(energy.total_kwh)) return 0;
  const float value = energy.total_kwh * 1000000.0f;
  if (value >= static_cast<float>(kEnergyTotalMaxUkwh)) return kEnergyTotalMaxUkwh;
  return static_cast<uint64_t>(value + 0.5f);
}

bool sequenceNewer(uint32_t candidate, uint32_t current) {
  return static_cast<int32_t>(candidate - current) > 0;
}

bool readEnergyJournalRecord(uint8_t sector, uint8_t slot, EnergyJournalRecord &record) {
  if (sector >= kEnergyJournalSectorCount || slot >= energyJournalRecordCount()) return false;
  return ESP.flashRead(energyJournalSlotAddress(sector, slot),
                       reinterpret_cast<uint32_t *>(&record),
                       sizeof(record));
}

bool energyJournalRecordEmpty(const EnergyJournalRecord &record) {
  const uint32_t *words = reinterpret_cast<const uint32_t *>(&record);
  for (size_t i = 0; i < sizeof(record) / sizeof(uint32_t); i++) {
    if (words[i] != 0xffffffffUL) return false;
  }
  return true;
}

bool energyJournalSlotEmpty(uint8_t sector, uint8_t slot) {
  EnergyJournalRecord record{};
  return readEnergyJournalRecord(sector, slot, record) && energyJournalRecordEmpty(record);
}

bool energyJournalRecordValid(const EnergyJournalRecord &record) {
  if (record.magic != kEnergyJournalMagic) return false;
  if (record.version != kEnergyJournalVersion) return false;
  if (record.size != sizeof(EnergyJournalRecord)) return false;
  if (record.reserved != 0) return false;
  if (record.total_ukwh > kEnergyTotalMaxUkwh) return false;
  return record.crc == configCrc(record);
}

bool eraseEnergyJournalSector(uint8_t sector) {
  if (sector >= kEnergyJournalSectorCount) return false;
  return ESP.flashEraseSector(energyJournalSectorAddress(sector) / kFlashSectorSize);
}

bool writeEnergyJournalRecord(uint8_t sector, uint8_t slot, uint64_t total_ukwh, uint32_t sequence) {
  if (!energyJournalSlotEmpty(sector, slot)) return false;

  EnergyJournalRecord record{};
  record.magic = kEnergyJournalMagic;
  record.version = kEnergyJournalVersion;
  record.size = sizeof(EnergyJournalRecord);
  record.sequence = sequence;
  record.total_ukwh = total_ukwh;
  record.crc = configCrc(record);

  const uint32_t address = energyJournalSlotAddress(sector, slot);
  if (!ESP.flashWrite(address, reinterpret_cast<uint32_t *>(&record), sizeof(record))) {
    return false;
  }

  EnergyJournalRecord verify{};
  return readEnergyJournalRecord(sector, slot, verify) &&
         memcmp(&verify, &record, sizeof(record)) == 0 &&
         energyJournalRecordValid(verify);
}

bool eraseEnergyJournal() {
  bool ok = true;
  for (uint8_t sector = 0; sector < kEnergyJournalSectorCount; sector++) {
    ok = eraseEnergyJournalSector(sector) && ok;
  }
  energy_journal_loaded = true;
  energy_journal_sector = 0;
  energy_journal_next_slot = 0;
  energy_journal_sequence = 0;
  energy_journal_saved_ukwh = 0;
  last_energy_persist_ms = millis();
  energy_persist_requested = false;
  energy.total_kwh = 0.0f;
  return ok;
}

void loadEnergyJournal() {
  bool found = false;
  uint8_t found_sector = 0;
  uint8_t found_slot = 0;
  uint32_t found_sequence = 0;
  uint64_t found_total = 0;

  for (uint8_t sector = 0; sector < kEnergyJournalSectorCount; sector++) {
    for (uint8_t slot = 0; slot < energyJournalRecordCount(); slot++) {
      EnergyJournalRecord record{};
      if (!readEnergyJournalRecord(sector, slot, record)) continue;
      if (!energyJournalRecordValid(record)) continue;
      if (!found || sequenceNewer(record.sequence, found_sequence)) {
        found = true;
        found_sector = sector;
        found_slot = slot;
        found_sequence = record.sequence;
        found_total = record.total_ukwh;
      }
    }
  }

  energy_journal_loaded = true;
  energy_journal_sector = found ? found_sector : 0;
  energy_journal_next_slot = found ? static_cast<uint8_t>(found_slot + 1) : 0;
  energy_journal_sequence = found ? found_sequence : 0;
  energy_journal_saved_ukwh = found ? found_total : 0;
  last_energy_persist_ms = millis();
  energy_persist_requested = false;
}

bool persistEnergyTotal(bool force) {
  if (!energy.present) return true;
  if (!energy_journal_loaded) loadEnergyJournal();

  const uint32_t now = millis();
  const uint64_t total = energyTotalUkwh();
  const uint64_t saved = energy_journal_saved_ukwh;
  const uint64_t delta = total > saved ? total - saved : 0;
  if (delta == 0) {
    energy_persist_requested = false;
    return true;
  }
  if (!force) {
    if (now - last_energy_persist_ms < kEnergyPersistMinMs) return true;
    if (!energy_persist_requested && delta < kEnergyPersistDeltaUkwh) return true;
  }

  uint8_t target_sector = energy_journal_sector;
  uint8_t target_slot = energy_journal_next_slot;
  const uint8_t record_count = energyJournalRecordCount();
  if (target_slot >= record_count || !energyJournalSlotEmpty(target_sector, target_slot)) {
    const uint8_t old_sector = target_sector;
    target_sector = (target_sector + 1) % kEnergyJournalSectorCount;
    target_slot = 0;
    if (!eraseEnergyJournalSector(target_sector)) return false;
    if (!writeEnergyJournalRecord(target_sector, target_slot, total, energy_journal_sequence + 1)) return false;
    eraseEnergyJournalSector(old_sector);
  } else {
    if (target_slot == 0 && !energyJournalSlotEmpty(target_sector, target_slot)) {
      if (!eraseEnergyJournalSector(target_sector)) return false;
    }
    if (!writeEnergyJournalRecord(target_sector, target_slot, total, energy_journal_sequence + 1)) return false;
  }

  energy_journal_sector = target_sector;
  energy_journal_next_slot = target_slot + 1;
  energy_journal_sequence++;
  energy_journal_saved_ukwh = total;
  last_energy_persist_ms = now;
  energy_persist_requested = false;
  return true;
}

// Boot recovery uses append-only marker writes so normal boots do not erase the
// EEPROM flash sector. Compaction falls back to EEPROM.commit() only when full.
uint32_t bootRecoveryMarkerAt(size_t index) {
  uint32_t marker = kBootRecoveryEmptyMarker;
  if (index < kBootRecoveryLogWords) {
    EEPROM.get(kBootRecoveryOffset + index * sizeof(marker), marker);
  }
  return marker;
}

size_t bootRecoveryWriteIndex() {
  for (size_t i = 0; i < kBootRecoveryLogWords; i++) {
    if (bootRecoveryMarkerAt(i) == kBootRecoveryEmptyMarker) {
      return i;
    }
  }
  return kBootRecoveryLogWords;
}

uint8_t bootRecoveryFastBootCount() {
  uint16_t count = 0;
  for (size_t i = 0; i < kBootRecoveryLogWords; i++) {
    const uint32_t marker = bootRecoveryMarkerAt(i);
    if (marker == kBootRecoveryEmptyMarker) break;
    if (marker == kBootRecoveryStableMarker) {
      count = 0;
    } else if (marker == kBootRecoveryBootMarker && count < 255) {
      count++;
    }
  }
  return static_cast<uint8_t>(count > 255 ? 255 : count);
}

void writeBootRecoveryLogShadowWord(size_t index, uint32_t marker) {
  if (index >= kBootRecoveryLogWords) return;
  const size_t offset = kBootRecoveryOffset + index * sizeof(marker);
  uint8_t *shadow = const_cast<uint8_t *>(EEPROM.getConstDataPtr());
  if (!shadow || offset + sizeof(marker) > EEPROM.length()) return;
  memcpy(shadow + offset, &marker, sizeof(marker));
}

void clearBootRecoveryLogShadow() {
  for (size_t i = 0; i < kBootRecoveryLogWords * sizeof(uint32_t); i++) {
    EEPROM.write(kBootRecoveryOffset + i, 0xff);
  }
}

bool compactBootRecoveryLog(uint8_t fast_boot_count) {
  if (fast_boot_count >= kBootRecoveryLimit) {
    fast_boot_count = kBootRecoveryLimit - 1;
  }
  clearBootRecoveryLogShadow();
  for (uint8_t i = 0; i < fast_boot_count; i++) {
    const uint32_t marker = kBootRecoveryBootMarker;
    EEPROM.put(kBootRecoveryOffset + i * sizeof(marker), marker);
  }
  return EEPROM.commit();
}

bool rawWriteBootRecoveryMarker(size_t index, uint32_t marker) {
  if (index >= kBootRecoveryLogWords) return false;
  if (bootRecoveryMarkerAt(index) != kBootRecoveryEmptyMarker) return false;

  const uint32_t offset = eepromFlashBase() + kBootRecoveryOffset + index * sizeof(marker);
  uint32_t word = marker;
  if (!ESP.flashWrite(offset, &word, sizeof(word))) return false;

  uint32_t verify = kBootRecoveryEmptyMarker;
  if (!ESP.flashRead(offset, &verify, sizeof(verify)) || verify != marker) return false;

  writeBootRecoveryLogShadowWord(index, marker);
  return true;
}

bool appendBootRecoveryMarker(uint32_t marker) {
  size_t index = bootRecoveryWriteIndex();
  if (index >= kBootRecoveryLogWords) {
    if (marker == kBootRecoveryStableMarker) {
      return compactBootRecoveryLog(0);
    }
    if (!compactBootRecoveryLog(bootRecoveryFastBootCount())) {
      return false;
    }
    index = bootRecoveryWriteIndex();
  }
  if (rawWriteBootRecoveryMarker(index, marker)) {
    return true;
  }
  EEPROM.put(kBootRecoveryOffset + index * sizeof(marker), marker);
  return EEPROM.commit();
}

bool clearBootRecoveryState() {
  if (bootRecoveryFastBootCount() == 0) {
    boot_recovery_count = 0;
    boot_recovery_armed = false;
    return true;
  }
  const bool saved = appendBootRecoveryMarker(kBootRecoveryStableMarker);
  if (saved) {
    boot_recovery_count = 0;
    boot_recovery_armed = false;
  }
  return saved;
}

bool recordBootRecoveryStart() {
  uint8_t boot_count = bootRecoveryFastBootCount();
  if (boot_count < 255) {
    boot_count++;
  }

  if (boot_count >= kBootRecoveryLimit) {
    boot_recovery_count = boot_count;
    boot_recovery_armed = false;
    boot_recovery_factory_reset = true;
    return true;
  }

  if (!appendBootRecoveryMarker(kBootRecoveryBootMarker)) {
    Serial.println(F("Could not persist fast power-cycle recovery marker"));
  }
  boot_recovery_count = boot_count;
  boot_recovery_armed = boot_count > 0;
  return false;
}

bool factoryResetConfig() {
  const bool energy_erased = eraseEnergyJournal();
  setDefaultConfig();
  clearBootRecoveryLogShadow();
  return energy_erased && commitConfig(true);
}

void maintainBootRecovery() {
  if (!boot_recovery_armed || millis() < kBootRecoveryStableMs) return;
  if (clearBootRecoveryState()) {
    Serial.println(F("Fast power-cycle recovery counter cleared"));
  }
}

void clearTemplateConfig() {
  config.template_enabled = 0;
  config.template_base = 0;
  config.template_flag = 0;
  memset(config.template_name, 0, sizeof(config.template_name));
  memset(config.template_gpio, 0, sizeof(config.template_gpio));
  setDefaultLedConfig();
  setDefaultButtonConfig();
}

void normalizeConfigStrings() {
  config.ssid[sizeof(config.ssid) - 1] = '\0';
  config.password[sizeof(config.password) - 1] = '\0';
  config.hostname[sizeof(config.hostname) - 1] = '\0';
  config.template_name[sizeof(config.template_name) - 1] = '\0';
  config.mqtt_host[sizeof(config.mqtt_host) - 1] = '\0';
  config.mqtt_topic[sizeof(config.mqtt_topic) - 1] = '\0';
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    config.button_press_target[i][sizeof(config.button_press_target[i]) - 1] = '\0';
    config.button_press_payload[i][sizeof(config.button_press_payload[i]) - 1] = '\0';
    config.button_hold_target[i][sizeof(config.button_hold_target[i]) - 1] = '\0';
    config.button_hold_payload[i][sizeof(config.button_hold_payload[i]) - 1] = '\0';
  }
  if (config.hostname[0] == '\0') {
    strlcpy(config.hostname, defaultHostname().c_str(), sizeof(config.hostname));
  }
  config.phy_mode = sanitizePhyMode(config.phy_mode);
  if (config.mqtt_port == 0) {
    config.mqtt_port = kMqttDefaultPort;
  }
  if (config.mqtt_topic[0] == '\0') {
    strlcpy(config.mqtt_topic, defaultMqttTopic().c_str(), sizeof(config.mqtt_topic));
  }
  if (isnan(config.energy_total_offset_kwh) ||
      config.energy_total_offset_kwh < kEnergyTotalOffsetMinKwh ||
      config.energy_total_offset_kwh > kEnergyTotalOffsetMaxKwh) {
    config.energy_total_offset_kwh = 0.0f;
  }
  if (config.energy_mqtt_change_percent_x10 > static_cast<uint16_t>(kMqttEnergyChangeMaxPercent * 10.0f)) {
    config.energy_mqtt_change_percent_x10 = 0;
  }
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    if (!isLedAttachmentEncoding(config.led_attach[i])) {
      config.led_attach[i] = defaultLedAttachment(i);
    }
  }
  if (config.button_hold_ms < kButtonHoldMinMs || config.button_hold_ms > kButtonHoldMaxMs) {
    config.button_hold_ms = kButtonHoldDefaultMs;
  }
  if (config.button_debounce_ms < kButtonDebounceMinMs || config.button_debounce_ms > kButtonDebounceMaxMs) {
    config.button_debounce_ms = kButtonDebounceDefaultMs;
  }
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    if (!isButtonActionEncoding(config.button_press_action[i])) {
      config.button_press_action[i] = kButtonActionRelayToggle;
    }
    if (!isButtonActionEncoding(config.button_hold_action[i])) {
      config.button_hold_action[i] = kButtonActionNone;
    }
    if (config.button_press_target[i][0] == '\0') {
      strlcpy(config.button_press_target[i], kDefaultButtonMqttTopic, sizeof(config.button_press_target[i]));
    }
    if (config.button_press_payload[i][0] == '\0') {
      strlcpy(config.button_press_payload[i], kDefaultButtonMqttPressPayload, sizeof(config.button_press_payload[i]));
    }
    if (config.button_hold_target[i][0] == '\0') {
      strlcpy(config.button_hold_target[i], kDefaultButtonMqttTopic, sizeof(config.button_hold_target[i]));
    }
    if (config.button_hold_payload[i][0] == '\0') {
      strlcpy(config.button_hold_payload[i], kDefaultButtonMqttHoldPayload, sizeof(config.button_hold_payload[i]));
    }
  }
  if (!config.template_enabled) {
    clearTemplateConfig();
  } else {
    if (config.template_name[0] == '\0') {
      strlcpy(config.template_name, "Template", sizeof(config.template_name));
    }
    if (config.template_base == 0) {
      config.template_base = 18;
    }
  }
}

bool storedConfigMatchesCurrentConfig() {
  const uint8_t *stored = EEPROM.getConstDataPtr();
  if (!stored) return false;
  return memcmp(stored, &config, sizeof(config)) == 0;
}

bool commitConfig(bool force_commit) {
  config.magic = kConfigMagic;
  config.version = kConfigVersion;
  config.size = sizeof(StoredConfig);
  normalizeConfigStrings();
  config.crc = configCrc(config);
  if (!force_commit && storedConfigMatchesCurrentConfig()) {
    config_ok = config.ssid[0] != '\0';
    return true;
  }
  EEPROM.put(0, config);
  const bool committed = EEPROM.commit();
  config_ok = committed && config.ssid[0] != '\0';
  return committed;
}

bool saveWifiConfig(const char *ssid, const char *password, const char *hostname, uint8_t phy_mode);
bool saveMqttConfig(const char *host, uint16_t port, const char *topic, uint16_t keepalive);
bool saveEnergyConfig(float total_offset_kwh, uint16_t mqtt_interval, uint16_t mqtt_change_percent_x10);
bool saveLedConfig(const uint8_t *attachments);
bool saveButtonConfig(uint16_t hold_ms, uint16_t debounce_ms,
                      const uint8_t *press_actions, const uint8_t *hold_actions,
                      const char press_targets[][kButtonActionTargetMaxLen + 1],
                      const char press_payloads[][kButtonActionPayloadMaxLen + 1],
                      const char hold_targets[][kButtonActionTargetMaxLen + 1],
                      const char hold_payloads[][kButtonActionPayloadMaxLen + 1]);

bool loadConfig() {
  EEPROM.begin(kEepromSize);
  if (recordBootRecoveryStart()) {
    Serial.println(F("Factory reset triggered by fast power cycling"));
    factoryResetConfig();
    return false;
  }

  ConfigHeader header{};
  EEPROM.get(0, header);
  if (header.magic != kConfigMagic) {
    setDefaultConfig();
    return false;
  }

  if (header.version == kConfigVersion && header.size == sizeof(StoredConfig)) {
    EEPROM.get(0, config);
    if (config.crc != configCrc(config)) {
      setDefaultConfig();
      return false;
    }
    normalizeConfigStrings();
    config_ok = config.ssid[0] != '\0';
    return config_ok;
  }

  if (header.version == kConfigVersionV9 && header.size == sizeof(StoredConfigV9)) {
    StoredConfigV9 *old_config = new StoredConfigV9;
    if (!old_config) {
      setDefaultConfig();
      return false;
    }
    EEPROM.get(0, *old_config);
    if (old_config->crc != configCrc(*old_config)) {
      delete old_config;
      setDefaultConfig();
      return false;
    }
    old_config->ssid[sizeof(old_config->ssid) - 1] = '\0';
    old_config->password[sizeof(old_config->password) - 1] = '\0';
    old_config->hostname[sizeof(old_config->hostname) - 1] = '\0';
    old_config->template_name[sizeof(old_config->template_name) - 1] = '\0';
    old_config->mqtt_host[sizeof(old_config->mqtt_host) - 1] = '\0';
    old_config->mqtt_topic[sizeof(old_config->mqtt_topic) - 1] = '\0';
    memset(&config, 0, sizeof(config));
    strlcpy(config.ssid, old_config->ssid, sizeof(config.ssid));
    strlcpy(config.password, old_config->password, sizeof(config.password));
    strlcpy(config.hostname, old_config->hostname, sizeof(config.hostname));
    config.phy_mode = old_config->phy_mode;
    config.template_enabled = old_config->template_enabled;
    config.template_base = old_config->template_base;
    config.template_flag = old_config->template_flag;
    strlcpy(config.template_name, old_config->template_name, sizeof(config.template_name));
    memcpy(config.template_gpio, old_config->template_gpio, sizeof(config.template_gpio));
    config.mqtt_port = old_config->mqtt_port;
    config.mqtt_keepalive = old_config->mqtt_keepalive;
    strlcpy(config.mqtt_host, old_config->mqtt_host, sizeof(config.mqtt_host));
    strlcpy(config.mqtt_topic, old_config->mqtt_topic, sizeof(config.mqtt_topic));
    config.energy_total_offset_kwh = old_config->energy_total_offset_kwh;
    memcpy(config.led_attach, old_config->led_attach, sizeof(config.led_attach));
    config.button_hold_ms = old_config->button_hold_ms;
    memcpy(config.button_press_action, old_config->button_press_action, sizeof(config.button_press_action));
    memcpy(config.button_hold_action, old_config->button_hold_action, sizeof(config.button_hold_action));
    config.energy_mqtt_interval = old_config->energy_mqtt_interval;
    config.energy_mqtt_change_percent_x10 = old_config->energy_mqtt_change_percent_x10;
    memcpy(config.button_press_target, old_config->button_press_target, sizeof(config.button_press_target));
    memcpy(config.button_press_payload, old_config->button_press_payload, sizeof(config.button_press_payload));
    memcpy(config.button_hold_target, old_config->button_hold_target, sizeof(config.button_hold_target));
    memcpy(config.button_hold_payload, old_config->button_hold_payload, sizeof(config.button_hold_payload));
    config.button_debounce_ms = kButtonDebounceDefaultMs;
    delete old_config;
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV8 && header.size == sizeof(StoredConfigV8)) {
    StoredConfigV8 old_config{};
    EEPROM.get(0, old_config);
    if (old_config.crc != configCrc(old_config)) {
      setDefaultConfig();
      return false;
    }
    old_config.ssid[sizeof(old_config.ssid) - 1] = '\0';
    old_config.password[sizeof(old_config.password) - 1] = '\0';
    old_config.hostname[sizeof(old_config.hostname) - 1] = '\0';
    old_config.template_name[sizeof(old_config.template_name) - 1] = '\0';
    old_config.mqtt_host[sizeof(old_config.mqtt_host) - 1] = '\0';
    old_config.mqtt_topic[sizeof(old_config.mqtt_topic) - 1] = '\0';
    memset(&config, 0, sizeof(config));
    strlcpy(config.ssid, old_config.ssid, sizeof(config.ssid));
    strlcpy(config.password, old_config.password, sizeof(config.password));
    strlcpy(config.hostname, old_config.hostname, sizeof(config.hostname));
    config.phy_mode = old_config.phy_mode;
    config.template_enabled = old_config.template_enabled;
    config.template_base = old_config.template_base;
    config.template_flag = old_config.template_flag;
    strlcpy(config.template_name, old_config.template_name, sizeof(config.template_name));
    memcpy(config.template_gpio, old_config.template_gpio, sizeof(config.template_gpio));
    config.mqtt_port = old_config.mqtt_port;
    config.mqtt_keepalive = old_config.mqtt_keepalive;
    strlcpy(config.mqtt_host, old_config.mqtt_host, sizeof(config.mqtt_host));
    strlcpy(config.mqtt_topic, old_config.mqtt_topic, sizeof(config.mqtt_topic));
    config.energy_total_offset_kwh = old_config.energy_total_offset_kwh;
    memcpy(config.led_attach, old_config.led_attach, sizeof(config.led_attach));
    config.button_hold_ms = old_config.button_hold_ms;
    memcpy(config.button_press_action, old_config.button_press_action, sizeof(config.button_press_action));
    memcpy(config.button_hold_action, old_config.button_hold_action, sizeof(config.button_hold_action));
    config.energy_mqtt_interval = old_config.energy_mqtt_interval;
    config.energy_mqtt_change_percent_x10 = old_config.energy_mqtt_change_percent_x10;
    setDefaultButtonActionTexts();
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV7 && header.size == sizeof(StoredConfigV7)) {
    StoredConfigV7 old_config{};
    EEPROM.get(0, old_config);
    if (old_config.crc != configCrc(old_config)) {
      setDefaultConfig();
      return false;
    }
    old_config.ssid[sizeof(old_config.ssid) - 1] = '\0';
    old_config.password[sizeof(old_config.password) - 1] = '\0';
    old_config.hostname[sizeof(old_config.hostname) - 1] = '\0';
    old_config.template_name[sizeof(old_config.template_name) - 1] = '\0';
    old_config.mqtt_host[sizeof(old_config.mqtt_host) - 1] = '\0';
    old_config.mqtt_topic[sizeof(old_config.mqtt_topic) - 1] = '\0';
    memset(&config, 0, sizeof(config));
    strlcpy(config.ssid, old_config.ssid, sizeof(config.ssid));
    strlcpy(config.password, old_config.password, sizeof(config.password));
    strlcpy(config.hostname, old_config.hostname, sizeof(config.hostname));
    config.phy_mode = old_config.phy_mode;
    config.template_enabled = old_config.template_enabled;
    config.template_base = old_config.template_base;
    config.template_flag = old_config.template_flag;
    strlcpy(config.template_name, old_config.template_name, sizeof(config.template_name));
    memcpy(config.template_gpio, old_config.template_gpio, sizeof(config.template_gpio));
    config.mqtt_port = old_config.mqtt_port;
    config.mqtt_keepalive = old_config.mqtt_keepalive;
    strlcpy(config.mqtt_host, old_config.mqtt_host, sizeof(config.mqtt_host));
    strlcpy(config.mqtt_topic, old_config.mqtt_topic, sizeof(config.mqtt_topic));
    config.energy_total_offset_kwh = old_config.energy_total_offset_kwh;
    memcpy(config.led_attach, old_config.led_attach, sizeof(config.led_attach));
    config.button_hold_ms = old_config.button_hold_ms;
    memcpy(config.button_press_action, old_config.button_press_action, sizeof(config.button_press_action));
    memcpy(config.button_hold_action, old_config.button_hold_action, sizeof(config.button_hold_action));
    setDefaultEnergyMqttConfig();
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV6 && header.size == sizeof(StoredConfigV6)) {
    StoredConfigV6 old_config{};
    EEPROM.get(0, old_config);
    if (old_config.crc != configCrc(old_config)) {
      setDefaultConfig();
      return false;
    }
    old_config.ssid[sizeof(old_config.ssid) - 1] = '\0';
    old_config.password[sizeof(old_config.password) - 1] = '\0';
    old_config.hostname[sizeof(old_config.hostname) - 1] = '\0';
    old_config.template_name[sizeof(old_config.template_name) - 1] = '\0';
    old_config.mqtt_host[sizeof(old_config.mqtt_host) - 1] = '\0';
    old_config.mqtt_topic[sizeof(old_config.mqtt_topic) - 1] = '\0';
    memset(&config, 0, sizeof(config));
    strlcpy(config.ssid, old_config.ssid, sizeof(config.ssid));
    strlcpy(config.password, old_config.password, sizeof(config.password));
    strlcpy(config.hostname, old_config.hostname, sizeof(config.hostname));
    config.phy_mode = old_config.phy_mode;
    config.template_enabled = old_config.template_enabled;
    config.template_base = old_config.template_base;
    config.template_flag = old_config.template_flag;
    strlcpy(config.template_name, old_config.template_name, sizeof(config.template_name));
    memcpy(config.template_gpio, old_config.template_gpio, sizeof(config.template_gpio));
    config.mqtt_port = old_config.mqtt_port;
    config.mqtt_keepalive = old_config.mqtt_keepalive;
    strlcpy(config.mqtt_host, old_config.mqtt_host, sizeof(config.mqtt_host));
    strlcpy(config.mqtt_topic, old_config.mqtt_topic, sizeof(config.mqtt_topic));
    config.energy_total_offset_kwh = old_config.energy_total_offset_kwh;
    memcpy(config.led_attach, old_config.led_attach, sizeof(config.led_attach));
    setDefaultEnergyMqttConfig();
    setDefaultButtonConfig();
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV5 && header.size == sizeof(StoredConfigV5)) {
    StoredConfigV5 old_config{};
    EEPROM.get(0, old_config);
    if (old_config.crc != configCrc(old_config)) {
      setDefaultConfig();
      return false;
    }
    old_config.ssid[sizeof(old_config.ssid) - 1] = '\0';
    old_config.password[sizeof(old_config.password) - 1] = '\0';
    old_config.hostname[sizeof(old_config.hostname) - 1] = '\0';
    old_config.template_name[sizeof(old_config.template_name) - 1] = '\0';
    old_config.mqtt_host[sizeof(old_config.mqtt_host) - 1] = '\0';
    old_config.mqtt_topic[sizeof(old_config.mqtt_topic) - 1] = '\0';
    memset(&config, 0, sizeof(config));
    strlcpy(config.ssid, old_config.ssid, sizeof(config.ssid));
    strlcpy(config.password, old_config.password, sizeof(config.password));
    strlcpy(config.hostname, old_config.hostname, sizeof(config.hostname));
    config.phy_mode = old_config.phy_mode;
    config.template_enabled = old_config.template_enabled;
    config.template_base = old_config.template_base;
    config.template_flag = old_config.template_flag;
    strlcpy(config.template_name, old_config.template_name, sizeof(config.template_name));
    memcpy(config.template_gpio, old_config.template_gpio, sizeof(config.template_gpio));
    config.mqtt_port = old_config.mqtt_port;
    config.mqtt_keepalive = old_config.mqtt_keepalive;
    strlcpy(config.mqtt_host, old_config.mqtt_host, sizeof(config.mqtt_host));
    strlcpy(config.mqtt_topic, old_config.mqtt_topic, sizeof(config.mqtt_topic));
    config.energy_total_offset_kwh = old_config.energy_total_offset_kwh;
    setDefaultEnergyMqttConfig();
    setDefaultLedConfig();
    setDefaultButtonConfig();
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV4 && header.size == sizeof(StoredConfigV4)) {
    StoredConfigV4 old_config{};
    EEPROM.get(0, old_config);
    if (old_config.crc != configCrc(old_config)) {
      setDefaultConfig();
      return false;
    }
    old_config.ssid[sizeof(old_config.ssid) - 1] = '\0';
    old_config.password[sizeof(old_config.password) - 1] = '\0';
    old_config.hostname[sizeof(old_config.hostname) - 1] = '\0';
    old_config.template_name[sizeof(old_config.template_name) - 1] = '\0';
    old_config.mqtt_host[sizeof(old_config.mqtt_host) - 1] = '\0';
    old_config.mqtt_topic[sizeof(old_config.mqtt_topic) - 1] = '\0';
    memset(&config, 0, sizeof(config));
    strlcpy(config.ssid, old_config.ssid, sizeof(config.ssid));
    strlcpy(config.password, old_config.password, sizeof(config.password));
    strlcpy(config.hostname, old_config.hostname, sizeof(config.hostname));
    config.phy_mode = old_config.phy_mode;
    config.template_enabled = old_config.template_enabled;
    config.template_base = old_config.template_base;
    config.template_flag = old_config.template_flag;
    strlcpy(config.template_name, old_config.template_name, sizeof(config.template_name));
    memcpy(config.template_gpio, old_config.template_gpio, sizeof(config.template_gpio));
    config.mqtt_port = old_config.mqtt_port;
    config.mqtt_keepalive = old_config.mqtt_keepalive;
    strlcpy(config.mqtt_host, old_config.mqtt_host, sizeof(config.mqtt_host));
    strlcpy(config.mqtt_topic, old_config.mqtt_topic, sizeof(config.mqtt_topic));
    config.energy_total_offset_kwh = 0.0f;
    setDefaultEnergyMqttConfig();
    setDefaultLedConfig();
    setDefaultButtonConfig();
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV3 && header.size == sizeof(StoredConfigV3)) {
    StoredConfigV3 old_config{};
    EEPROM.get(0, old_config);
    if (old_config.crc != configCrc(old_config)) {
      setDefaultConfig();
      return false;
    }
    old_config.ssid[sizeof(old_config.ssid) - 1] = '\0';
    old_config.password[sizeof(old_config.password) - 1] = '\0';
    old_config.hostname[sizeof(old_config.hostname) - 1] = '\0';
    old_config.template_name[sizeof(old_config.template_name) - 1] = '\0';
    memset(&config, 0, sizeof(config));
    strlcpy(config.ssid, old_config.ssid, sizeof(config.ssid));
    strlcpy(config.password, old_config.password, sizeof(config.password));
    strlcpy(config.hostname, old_config.hostname, sizeof(config.hostname));
    config.phy_mode = old_config.phy_mode;
    config.template_enabled = old_config.template_enabled;
    config.template_base = old_config.template_base;
    config.template_flag = old_config.template_flag;
    strlcpy(config.template_name, old_config.template_name, sizeof(config.template_name));
    memcpy(config.template_gpio, old_config.template_gpio, sizeof(config.template_gpio));
    setDefaultMqttConfig();
    setDefaultEnergyMqttConfig();
    setDefaultLedConfig();
    setDefaultButtonConfig();
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV2 && header.size == sizeof(StoredConfigV2)) {
    StoredConfigV2 old_config{};
    EEPROM.get(0, old_config);
    if (old_config.crc != configCrc(old_config)) {
      setDefaultConfig();
      return false;
    }
    old_config.ssid[sizeof(old_config.ssid) - 1] = '\0';
    old_config.password[sizeof(old_config.password) - 1] = '\0';
    old_config.hostname[sizeof(old_config.hostname) - 1] = '\0';
    memset(&config, 0, sizeof(config));
    strlcpy(config.ssid, old_config.ssid, sizeof(config.ssid));
    strlcpy(config.password, old_config.password, sizeof(config.password));
    strlcpy(config.hostname, old_config.hostname, sizeof(config.hostname));
    config.phy_mode = old_config.phy_mode;
    clearTemplateConfig();
    setDefaultMqttConfig();
    setDefaultEnergyMqttConfig();
    setDefaultLedConfig();
    setDefaultButtonConfig();
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV1 && header.size == sizeof(StoredConfigV1)) {
    StoredConfigV1 old_config{};
    EEPROM.get(0, old_config);
    if (old_config.crc != configCrc(old_config)) {
      setDefaultConfig();
      return false;
    }
    old_config.ssid[sizeof(old_config.ssid) - 1] = '\0';
    old_config.password[sizeof(old_config.password) - 1] = '\0';
    old_config.hostname[sizeof(old_config.hostname) - 1] = '\0';
    memset(&config, 0, sizeof(config));
    config.magic = kConfigMagic;
    config.version = kConfigVersion;
    config.size = sizeof(StoredConfig);
    strlcpy(config.ssid, old_config.ssid, sizeof(config.ssid));
    strlcpy(config.password, old_config.password, sizeof(config.password));
    strlcpy(config.hostname, old_config.hostname, sizeof(config.hostname));
    config.phy_mode = kPhyModeAuto;
    clearTemplateConfig();
    setDefaultMqttConfig();
    setDefaultEnergyMqttConfig();
    setDefaultLedConfig();
    setDefaultButtonConfig();
    commitConfig();
    return config_ok;
  }

  setDefaultConfig();
  return false;
}

bool saveWifiConfig(const char *ssid, const char *password, const char *hostname, uint8_t phy_mode) {
  strlcpy(config.ssid, ssid, sizeof(config.ssid));
  strlcpy(config.password, password, sizeof(config.password));
  if (hostname && hostname[0]) {
    strlcpy(config.hostname, hostname, sizeof(config.hostname));
  } else {
    strlcpy(config.hostname, defaultHostname().c_str(), sizeof(config.hostname));
  }
  config.phy_mode = sanitizePhyMode(phy_mode);
  return commitConfig();
}

bool saveMqttConfig(const char *host, uint16_t port, const char *topic, uint16_t keepalive) {
  strlcpy(config.mqtt_host, host ? host : "", sizeof(config.mqtt_host));
  config.mqtt_port = port;
  strlcpy(config.mqtt_topic, topic ? topic : "", sizeof(config.mqtt_topic));
  config.mqtt_keepalive = keepalive;
  mqtt_client.stop();
  next_mqtt_reconnect = 0;
  last_mqtt_io = 0;
  last_mqtt_rx = 0;
  last_mqtt_ping = 0;
  last_mqtt_state_publish = 0;
  last_mqtt_energy_publish = 0;
  last_mqtt_connect_attempt = 0;
  last_mqtt_connect_duration = 0;
  last_mqtt_connect_result = kMqttConnectIdle;
  last_mqtt_energy_power = NAN;
  mqtt_pending_relay_mask = 0;
  mqtt_button_queue_head = 0;
  mqtt_button_queue_count = 0;
  mqtt_ping_pending = false;
  return commitConfig();
}

bool saveEnergyConfig(float total_offset_kwh, uint16_t mqtt_interval, uint16_t mqtt_change_percent_x10) {
  config.energy_total_offset_kwh = total_offset_kwh;
  config.energy_mqtt_interval = mqtt_interval;
  config.energy_mqtt_change_percent_x10 = mqtt_change_percent_x10;
  last_mqtt_energy_publish = 0;
  last_mqtt_energy_power = NAN;
  return commitConfig();
}

bool saveLedConfig(const uint8_t *attachments) {
  memcpy(config.led_attach, attachments, sizeof(config.led_attach));
  return commitConfig();
}

bool saveButtonConfig(uint16_t hold_ms, uint16_t debounce_ms,
                      const uint8_t *press_actions, const uint8_t *hold_actions,
                      const char press_targets[][kButtonActionTargetMaxLen + 1],
                      const char press_payloads[][kButtonActionPayloadMaxLen + 1],
                      const char hold_targets[][kButtonActionTargetMaxLen + 1],
                      const char hold_payloads[][kButtonActionPayloadMaxLen + 1]) {
  config.button_hold_ms = hold_ms;
  config.button_debounce_ms = debounce_ms;
  memcpy(config.button_press_action, press_actions, sizeof(config.button_press_action));
  memcpy(config.button_hold_action, hold_actions, sizeof(config.button_hold_action));
  memcpy(config.button_press_target, press_targets, sizeof(config.button_press_target));
  memcpy(config.button_press_payload, press_payloads, sizeof(config.button_press_payload));
  memcpy(config.button_hold_target, hold_targets, sizeof(config.button_hold_target));
  memcpy(config.button_hold_payload, hold_payloads, sizeof(config.button_hold_payload));
  return commitConfig();
}

float reportedEnergyTotalKwh() {
  return energy.total_kwh + config.energy_total_offset_kwh;
}

float energyMqttChangePercent() {
  return static_cast<float>(config.energy_mqtt_change_percent_x10) / 10.0f;
}

const char *buttonEventType(bool hold) {
  return hold ? "HOLD" : "TOGGLE";
}

const char *buttonActionTarget(uint8_t button, bool hold) {
  if (button >= kMaxButtons) return "";
  return hold ? config.button_hold_target[button] : config.button_press_target[button];
}

const char *buttonActionPayload(uint8_t button, bool hold) {
  if (button >= kMaxButtons) return "";
  return hold ? config.button_hold_payload[button] : config.button_press_payload[button];
}

bool parseRelayStateToken(const char *token, size_t len, uint8_t &relay) {
  if (len < 14 || strncmp(token, "{RELAY", 6) != 0) return false;
  uint16_t number = 0;
  size_t index = 6;
  while (index < len && token[index] >= '0' && token[index] <= '9') {
    number = (number * 10U) + static_cast<uint16_t>(token[index] - '0');
    index++;
  }
  if (number == 0 || number > kMaxRelays) return false;
  if (index + 7 != len || strncmp(token + index, "_STATE}", 7) != 0) return false;
  relay = static_cast<uint8_t>(number - 1);
  return true;
}

void appendRelayStateTokenValue(String &out, uint8_t relay) {
  const bool available = relay < runtime_template.relay_count && hasPin(runtime_template.relays[relay]);
  out += available ? (relay_state[relay] ? F("ON") : F("OFF")) : F("UNKNOWN");
}

String expandButtonActionText(const char *input, uint8_t button, bool hold) {
  String out;
  if (!input) return out;
  out.reserve(strlen(input) + strlen(config.mqtt_topic) + 16);

  const char *p = input;
  while (*p) {
    if (*p != '{') {
      out += *p++;
      continue;
    }

    const char *end = strchr(p, '}');
    if (!end) {
      out += *p++;
      continue;
    }

    const size_t len = static_cast<size_t>(end - p + 1);
    uint8_t relay = 0;
    if (len == 10 && strncmp(p, "{BUTTONID}", len) == 0) {
      out += String(button + 1);
    } else if (len == 6 && strncmp(p, "{TYPE}", len) == 0) {
      out += buttonEventType(hold);
    } else if (len == 7 && strncmp(p, "{TOPIC}", len) == 0) {
      out += config.mqtt_topic;
    } else if (parseRelayStateToken(p, len, relay)) {
      appendRelayStateTokenValue(out, relay);
    } else {
      for (const char *copy = p; copy <= end; copy++) {
        out += *copy;
      }
    }
    p = end + 1;
  }
  return out;
}

bool hasControlChar(const String &value, bool allow_multiline = false) {
  for (size_t i = 0; i < value.length(); i++) {
    const char c = value[i];
    if (allow_multiline && (c == '\n' || c == '\r' || c == '\t')) continue;
    if (static_cast<uint8_t>(c) < 0x20 || c == 0x7f) return true;
  }
  return false;
}

bool isValidButtonActionText(const String &value, size_t max_len, bool allow_empty, bool allow_multiline = false) {
  if (!allow_empty && value.length() == 0) return false;
  if (value.length() > max_len) return false;
  return !hasControlChar(value, allow_multiline);
}

bool isValidMqttPublishTopicTemplate(const String &topic) {
  if (!isValidButtonActionText(topic, kButtonActionTargetMaxLen, false)) return false;
  for (size_t i = 0; i < topic.length(); i++) {
    if (topic[i] == '#' || topic[i] == '+') return false;
  }
  return true;
}

bool isValidWebhookUrlTemplate(const String &url) {
  if (!isValidButtonActionText(url, kButtonActionTargetMaxLen, false)) return false;
  if (!url.startsWith(F("http://"))) return false;
  const int host_start = 7;
  const int path_start = url.indexOf('/', host_start);
  const String host_port = path_start < 0 ? url.substring(host_start) : url.substring(host_start, path_start);
  return host_port.length() > 0 && host_port.indexOf(' ') < 0;
}

String htmlEscape(const String &input) {
  String out;
  out.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); i++) {
    const char c = input[i];
    if (c == '&') out += F("&amp;");
    else if (c == '<') out += F("&lt;");
    else if (c == '>') out += F("&gt;");
    else if (c == '"') out += F("&quot;");
    else if (c == '\'') out += F("&#39;");
    else out += c;
  }
  return out;
}

String jsonEscape(const char *input) {
  String out;
  if (!input) return out;
  out.reserve(strlen(input) + 8);
  for (const char *p = input; *p; p++) {
    const char c = *p;
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<uint8_t>(c) < 0x20) {
      out += ' ';
    } else {
      out += c;
    }
  }
  return out;
}

String pinName(uint8_t pin) {
  if (pin == kAdc0Pin) return F("ADC0");
  if (pin == kInvalidPin) return F("-");
  return String(F("GPIO")) + String(pin);
}

void resetPinAssignment(PinAssignment &assignment) {
  assignment.pin = kInvalidPin;
  assignment.inverted = false;
  assignment.no_pullup = false;
}

bool hasPin(const PinAssignment &assignment) {
  return assignment.pin != kInvalidPin;
}

bool digitalPinSupported(uint8_t pin) {
  return pin != kInvalidPin && pin <= 16;
}

bool interruptPinSupported(uint8_t pin) {
  return pin != kInvalidPin && pin <= 15;
}

void writeAssignedPin(const PinAssignment &assignment, bool on) {
  if (!digitalPinSupported(assignment.pin)) return;
  digitalWrite(assignment.pin, (on ^ assignment.inverted) ? HIGH : LOW);
}

bool readAssignedPressed(const PinAssignment &assignment) {
  if (!digitalPinSupported(assignment.pin)) return false;
  const bool high = digitalRead(assignment.pin) == HIGH;
  return assignment.inverted ? high : !high;
}

const PinAssignment *ledOutputAssignment(uint8_t led) {
  if (led < kMaxLeds) return &runtime_template.leds[led];
  if (led == kMaxLeds) return &runtime_template.link_led;
  return nullptr;
}

bool hasLedOutput(uint8_t led) {
  const PinAssignment *assignment = ledOutputAssignment(led);
  if (!assignment || !hasPin(*assignment)) return false;
  return led >= kMaxLeds || led < runtime_template.led_count;
}

String ledOutputName(uint8_t led) {
  if (led < kMaxLeds) {
    return String(F("LED ")) + String(led + 1);
  }
  return F("Link LED");
}

String ledAttachmentName(uint8_t value) {
  uint8_t index = 0;
  if (ledAttachmentRelayIndex(value, index)) {
    return String(F("relay")) + String(index + 1);
  }
  if (ledAttachmentButtonIndex(value, index)) {
    return String(F("button")) + String(index + 1);
  }
  return F("none");
}

bool ledAttachmentAvailable(uint8_t value) {
  uint8_t index = 0;
  if (value == kLedAttachNone) return true;
  if (ledAttachmentRelayIndex(value, index)) {
    return index < runtime_template.relay_count && hasPin(runtime_template.relays[index]);
  }
  if (ledAttachmentButtonIndex(value, index)) {
    return index < runtime_template.button_count && hasPin(runtime_template.buttons[index]);
  }
  return false;
}

bool ledOutputOn(uint8_t led) {
  if (!hasLedOutput(led) || led >= kMaxLedOutputs) return false;
  const uint8_t attachment = config.led_attach[led];
  uint8_t index = 0;
  if (ledAttachmentRelayIndex(attachment, index)) {
    return index < runtime_template.relay_count && hasPin(runtime_template.relays[index]) && relay_state[index];
  }
  if (ledAttachmentButtonIndex(attachment, index)) {
    return index < runtime_template.button_count && hasPin(runtime_template.buttons[index]) && button_state[index].stable_pressed;
  }
  return false;
}

bool hasConfigurableLedOutputs() {
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    if (hasLedOutput(i)) return true;
  }
  return false;
}

String buttonActionName(uint8_t action) {
  if (action == kButtonActionRelayToggle) return F("relay toggle");
  if (action == kButtonActionMqtt) return F("mqtt broadcast");
  if (action == kButtonActionWebhook) return F("webhook exec");
  return F("nothing");
}

bool buttonRelayTarget(uint8_t button, uint8_t &relay) {
  if (button < runtime_template.relay_count && hasPin(runtime_template.relays[button])) {
    relay = button;
    return true;
  }
  if (hasPin(runtime_template.relays[0])) {
    relay = 0;
    return true;
  }
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (hasPin(runtime_template.relays[i])) {
      relay = i;
      return true;
    }
  }
  return false;
}

bool buttonActionAvailable(uint8_t button, uint8_t action) {
  if (action == kButtonActionNone) return true;
  if (action == kButtonActionRelayToggle) {
    uint8_t relay = 0;
    return buttonRelayTarget(button, relay);
  }
  if (action == kButtonActionMqtt || action == kButtonActionWebhook) return true;
  return false;
}

bool hasConfigurableButtons() {
  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (hasPin(runtime_template.buttons[i])) return true;
  }
  return false;
}

void addUnsupportedTemplatePin(uint8_t pin, uint16_t code) {
  if (runtime_template.unsupported_count >= sizeof(runtime_template.unsupported_code) / sizeof(runtime_template.unsupported_code[0])) {
    return;
  }
  const uint8_t index = runtime_template.unsupported_count++;
  runtime_template.unsupported_pin[index] = pin;
  runtime_template.unsupported_code[index] = code;
}

void parseTemplateFunction(uint8_t pin, uint16_t code) {
  if (code == kTplNone || code == kTplUser || code == 65504U) {
    return;
  }

  const uint16_t base = code & 0xffe0U;
  const uint8_t index = code & 0x1fU;
  if (pin == kAdc0Pin) {
    if (code == kTplAdcTemp) {
      runtime_template.adc_temp = true;
    } else {
      addUnsupportedTemplatePin(pin, code);
    }
    return;
  }

  if (!digitalPinSupported(pin)) {
    addUnsupportedTemplatePin(pin, code);
    return;
  }

  if (base == kTplKey1 || base == kTplKey1Np || base == kTplKey1Inv || base == kTplKey1InvNp) {
    if (index >= kMaxButtons) {
      addUnsupportedTemplatePin(pin, code);
      return;
    }
    runtime_template.buttons[index] = {
      pin,
      base == kTplKey1Inv || base == kTplKey1InvNp,
      base == kTplKey1Np || base == kTplKey1InvNp
    };
    if (runtime_template.button_count <= index) runtime_template.button_count = index + 1;
    return;
  }

  if (base == kTplRel1 || base == kTplRel1Inv) {
    if (index >= kMaxRelays) {
      addUnsupportedTemplatePin(pin, code);
      return;
    }
    runtime_template.relays[index] = {pin, base == kTplRel1Inv, false};
    if (runtime_template.relay_count <= index) runtime_template.relay_count = index + 1;
    return;
  }

  if (base == kTplLed1 || base == kTplLed1Inv) {
    if (index >= kMaxLeds) {
      addUnsupportedTemplatePin(pin, code);
      return;
    }
    runtime_template.leds[index] = {pin, base == kTplLed1Inv, false};
    if (runtime_template.led_count <= index) runtime_template.led_count = index + 1;
    return;
  }

  if (code == kTplLedLnk || code == kTplLedLnkInv) {
    runtime_template.link_led = {pin, code == kTplLedLnkInv, false};
    return;
  }

  if (code == kTplNrgSel || code == kTplNrgSelInv) {
    runtime_template.energy_sel_pin = pin;
    runtime_template.energy_sel_inverted = code == kTplNrgSelInv;
    return;
  }

  if (code == kTplNrgCf1) {
    if (!interruptPinSupported(pin)) {
      addUnsupportedTemplatePin(pin, code);
      return;
    }
    runtime_template.energy_cf1_pin = pin;
    return;
  }

  if (code == kTplHlwCf || code == kTplHjlCf) {
    if (!interruptPinSupported(pin)) {
      addUnsupportedTemplatePin(pin, code);
      return;
    }
    runtime_template.energy_cf_pin = pin;
    runtime_template.energy_hjl = code == kTplHjlCf;
    return;
  }

  addUnsupportedTemplatePin(pin, code);
}

void detachEnergyInterrupts() {
  if (!energy.present) return;
  if (interruptPinSupported(runtime_template.energy_cf_pin)) {
    detachInterrupt(digitalPinToInterrupt(runtime_template.energy_cf_pin));
  }
  if (runtime_template.energy_cf1_pin != runtime_template.energy_cf_pin &&
      interruptPinSupported(runtime_template.energy_cf1_pin)) {
    detachInterrupt(digitalPinToInterrupt(runtime_template.energy_cf1_pin));
  }
  energy.present = false;
}

void decodeTemplateConfig() {
  detachEnergyInterrupts();
  memset(&runtime_template, 0, sizeof(runtime_template));
  for (uint8_t i = 0; i < kMaxRelays; i++) resetPinAssignment(runtime_template.relays[i]);
  for (uint8_t i = 0; i < kMaxButtons; i++) resetPinAssignment(runtime_template.buttons[i]);
  for (uint8_t i = 0; i < kMaxLeds; i++) resetPinAssignment(runtime_template.leds[i]);
  resetPinAssignment(runtime_template.link_led);
  runtime_template.energy_cf_pin = kInvalidPin;
  runtime_template.energy_cf1_pin = kInvalidPin;
  runtime_template.energy_sel_pin = kInvalidPin;

  if (!config.template_enabled) return;

  runtime_template.enabled = true;
  strlcpy(runtime_template.name, config.template_name, sizeof(runtime_template.name));
  runtime_template.base = config.template_base;
  runtime_template.flag = config.template_flag;
  for (uint8_t i = 0; i < kTemplateSlotCount; i++) {
    parseTemplateFunction(kTemplateSlotToPin[i], config.template_gpio[i]);
  }
}

bool isJsonSpace(char c) {
  return c == ' ' || c == '\n' || c == '\r' || c == '\t';
}

bool templateJsonHasSingleRootObject(const String &json) {
  const char *p = json.c_str();
  while (isJsonSpace(*p)) p++;
  if (*p != '{') return false;

  uint16_t depth = 0;
  bool in_string = false;
  bool escaped = false;
  for (; *p; p++) {
    const char c = *p;
    if (in_string) {
      if (escaped) {
        escaped = false;
      } else if (c == '\\') {
        escaped = true;
      } else if (c == '"') {
        in_string = false;
      }
      continue;
    }

    if (c == '"') {
      in_string = true;
    } else if (c == '{' || c == '[') {
      depth++;
    } else if (c == '}' || c == ']') {
      if (depth == 0) return false;
      depth--;
      if (depth == 0) {
        p++;
        break;
      }
    }
  }

  if (depth != 0 || in_string || escaped) return false;
  while (isJsonSpace(*p)) p++;
  return *p == '\0';
}

bool parseTemplateJson(const String &json, StoredConfig &target, String &error) {
  if (json.length() < 9 || json.length() > kTemplateJsonMaxLen) {
    error = F("Template JSON length is invalid");
    return false;
  }
  if (!templateJsonHasSingleRootObject(json)) {
    error = F("Template must be one complete JSON object");
    return false;
  }

  DynamicJsonDocument doc(kTemplateJsonDocCapacity);
  const DeserializationError json_error = deserializeJson(doc, json);
  if (json_error) {
    error = F("Template JSON parse failed: ");
    error += json_error.c_str();
    return false;
  }
  if (!doc.is<JsonObject>()) {
    error = F("Template must be a JSON object");
    return false;
  }

  const char *arch = doc["ARCH"] | "";
  if (arch[0] && strcmp(arch, "ESP8266") && strcmp(arch, "ESP8285") && strcmp(arch, "ESP82XX")) {
    error = F("Template ARCH is not ESP8266/ESP8285");
    return false;
  }

  uint16_t gpio[kTemplateSlotCount]{};
  const char *name = doc["NAME"] | "";
  if (name[0] == '\0') {
    error = F("Template NAME is empty");
    return false;
  }
  if (strlen(name) >= sizeof(target.template_name)) {
    error = F("Template NAME is too long");
    return false;
  }

  JsonArray gpio_values = doc["GPIO"].as<JsonArray>();
  if (gpio_values.isNull() || gpio_values.size() != kTemplateSlotCount) {
    error = F("GPIO must contain exactly 14 ESP8266 entries");
    return false;
  }
  for (uint8_t i = 0; i < kTemplateSlotCount; i++) {
    JsonVariant value = gpio_values[i];
    if (!value.is<uint16_t>()) {
      error = F("Invalid GPIO value");
      return false;
    }
    gpio[i] = value.as<uint16_t>();
  }

  JsonVariant base_value = doc["BASE"];
  if (!base_value.is<uint8_t>() || base_value.as<uint8_t>() == 0) {
    error = F("Template BASE is invalid");
    return false;
  }
  JsonVariant flag_value = doc["FLAG"];
  if (!flag_value.isNull() && !flag_value.is<uint32_t>()) {
    error = F("Template FLAG is invalid");
    return false;
  }

  target.template_enabled = 1;
  target.template_base = base_value.as<uint8_t>();
  target.template_flag = flag_value.isNull() ? 0 : flag_value.as<uint32_t>();
  strlcpy(target.template_name, name, sizeof(target.template_name));
  memcpy(target.template_gpio, gpio, sizeof(target.template_gpio));
  return true;
}

String currentTemplateJson() {
  if (!config.template_enabled) return String();
  String out;
  out.reserve(220);
  out += F("{\"NAME\":\"");
  out += jsonEscape(config.template_name);
  out += F("\",\"GPIO\":[");
  for (uint8_t i = 0; i < kTemplateSlotCount; i++) {
    if (i) out += ',';
    out += String(config.template_gpio[i]);
  }
  out += F("],\"FLAG\":");
  out += String(config.template_flag);
  out += F(",\"BASE\":");
  out += String(config.template_base);
  out += F("}");
  return out;
}

bool parseUint16Input(const String &input, uint16_t min_value, uint16_t max_value, uint16_t &out) {
  if (input.length() == 0) return false;
  uint32_t value = 0;
  for (size_t i = 0; i < input.length(); i++) {
    const char c = input[i];
    if (c < '0' || c > '9') return false;
    value = (value * 10U) + static_cast<uint32_t>(c - '0');
    if (value > max_value) return false;
  }
  if (value < min_value) return false;
  out = static_cast<uint16_t>(value);
  return true;
}

bool parseFloatInput(const String &input, float min_value, float max_value, float &out) {
  String value = input;
  value.trim();
  if (value.length() == 0) return false;

  const char *p = value.c_str();
  bool negative = false;
  if (*p == '+' || *p == '-') {
    negative = *p == '-';
    p++;
  }

  bool has_digit = false;
  double parsed = 0.0;
  while (*p >= '0' && *p <= '9') {
    has_digit = true;
    parsed = (parsed * 10.0) + static_cast<double>(*p - '0');
    p++;
  }

  if (*p == '.') {
    p++;
    double divisor = 10.0;
    while (*p >= '0' && *p <= '9') {
      has_digit = true;
      parsed += static_cast<double>(*p - '0') / divisor;
      divisor *= 10.0;
      p++;
    }
  }

  if (!has_digit || *p != '\0') return false;
  if (negative) parsed = -parsed;
  if (parsed < min_value || parsed > max_value) return false;
  out = static_cast<float>(parsed);
  return true;
}

bool parsePowerCommand(const char *p, size_t len, uint8_t &relay, char *response_key, size_t key_size) {
  if (len < 5 || strncasecmp(p, "power", 5) != 0) return false;
  if (len == 5) {
    relay = 0;
    strlcpy(response_key, "POWER", key_size);
    return true;
  }
  uint16_t relay_number = 0;
  for (size_t i = 5; i < len; i++) {
    const char c = p[i];
    if (c < '0' || c > '9') return false;
    relay_number = (relay_number * 10U) + static_cast<uint16_t>(c - '0');
    if (relay_number > kMaxRelays) return false;
  }
  if (relay_number == 0) return false;
  relay = static_cast<uint8_t>(relay_number - 1);
  if (snprintf(response_key, key_size, "POWER%u", static_cast<unsigned>(relay_number)) >= static_cast<int>(key_size)) {
    return false;
  }
  return true;
}

bool parsePowerState(const char *p, size_t len, bool &on) {
  if (len == 2 && (p[0] | 0x20) == 'o' && (p[1] | 0x20) == 'n') {
    on = true;
    return true;
  }
  if (len == 3 && (p[0] | 0x20) == 'o' && (p[1] | 0x20) == 'f' && (p[2] | 0x20) == 'f') {
    on = false;
    return true;
  }
  return false;
}

bool isValidMqttHost(const String &host) {
  if (host.length() > kMqttHostMaxLen) return false;
  for (size_t i = 0; i < host.length(); i++) {
    const char c = host[i];
    if (c <= ' ' || c == '/' || c == '\\') return false;
  }
  return true;
}

bool isValidMqttTopic(const String &topic) {
  if (topic.length() == 0 || topic.length() > kMqttTopicMaxLen) return false;
  for (size_t i = 0; i < topic.length(); i++) {
    const uint8_t c = static_cast<uint8_t>(topic[i]);
    if (c < 0x20 || c == 0x7f) return false;
  }
  return true;
}

String ipToString(const IPAddress &ip) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return String(buf);
}

const __FlashStringHelper *updateErrorName(uint8_t error) {
  switch (error) {
    case UPDATE_ERROR_OK: return F("ok");
    case UPDATE_ERROR_WRITE: return F("write failed");
    case UPDATE_ERROR_ERASE: return F("erase failed");
    case UPDATE_ERROR_READ: return F("read failed");
    case UPDATE_ERROR_SPACE: return F("not enough space");
    case UPDATE_ERROR_SIZE: return F("invalid size");
    case UPDATE_ERROR_STREAM: return F("stream failed");
    case UPDATE_ERROR_MD5: return F("md5 mismatch");
    case UPDATE_ERROR_FLASH_CONFIG: return F("bad flash config");
    case UPDATE_ERROR_NEW_FLASH_CONFIG: return F("image flash config too large");
    case UPDATE_ERROR_MAGIC_BYTE: return F("invalid image magic");
    case UPDATE_ERROR_BOOTSTRAP: return F("bad boot mode");
    case UPDATE_ERROR_SIGN: return F("signature failed");
    default: return F("unknown");
  }
}

bool mqttConfigured() {
  return config.mqtt_host[0] != '\0' && config.mqtt_topic[0] != '\0' && config.mqtt_port != 0;
}

const __FlashStringHelper *mqttConnectResultName(uint8_t result) {
  switch (result) {
    case kMqttConnectOk: return F("ok");
    case kMqttConnectTcpFailed: return F("tcp_failed");
    case kMqttConnectWriteFailed: return F("connect_write_failed");
    case kMqttConnectConnackTimeout: return F("connack_timeout");
    case kMqttConnectConnackRejected: return F("connack_rejected");
    default: return F("idle");
  }
}

void clearMqttButtonQueue() {
  mqtt_button_queue_head = 0;
  mqtt_button_queue_count = 0;
}

uint8_t mqttButtonQueueIndex(uint8_t offset) {
  return (mqtt_button_queue_head + offset) % kMqttButtonQueueDepth;
}

void dropMqttButtonQueueHead() {
  if (mqtt_button_queue_count == 0) return;
  mqtt_button_queue_head = mqttButtonQueueIndex(1);
  mqtt_button_queue_count--;
}

bool mqttButtonQueueExpired(const MqttButtonPending &item, uint32_t now) {
  return static_cast<uint32_t>(now - item.queued_at) > kMqttButtonQueueMaxAgeMs;
}

void expireMqttButtonQueue(uint32_t now) {
  while (mqtt_button_queue_count > 0 && mqttButtonQueueExpired(mqtt_button_queue[mqtt_button_queue_head], now)) {
    dropMqttButtonQueueHead();
  }
}

bool pushMqttButtonQueue(const String &topic, const String &payload) {
  if (topic.length() == 0 || topic.length() > kMqttButtonTopicMaxLen) return false;
  if (payload.length() == 0 || payload.length() > kMqttButtonPayloadMaxLen) return false;

  if (mqtt_button_queue_count >= kMqttButtonQueueDepth) {
    dropMqttButtonQueueHead();
  }

  MqttButtonPending &slot = mqtt_button_queue[mqttButtonQueueIndex(mqtt_button_queue_count)];
  slot.queued_at = millis();
  strlcpy(slot.topic, topic.c_str(), sizeof(slot.topic));
  strlcpy(slot.payload, payload.c_str(), sizeof(slot.payload));
  mqtt_button_queue_count++;
  return true;
}

void recordMqttConnectResult(uint8_t result, uint32_t started) {
  last_mqtt_connect_result = result;
  last_mqtt_connect_duration = millis() - started;
}

bool mqttReadByteUntil(uint8_t &value, uint32_t deadline_ms) {
  while (!mqtt_client.available()) {
    if (!mqtt_client.connected() || static_cast<int32_t>(millis() - deadline_ms) >= 0) {
      return false;
    }
    delay(1);
  }
  const int read_value = mqtt_client.read();
  if (read_value < 0) return false;
  value = static_cast<uint8_t>(read_value);
  last_mqtt_rx = millis();
  return true;
}

bool mqttReadRemainingLengthUntil(uint32_t &length, uint32_t max_length, uint32_t deadline_ms) {
  length = 0;
  uint32_t multiplier = 1;
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t encoded = 0;
    if (!mqttReadByteUntil(encoded, deadline_ms)) return false;
    length += static_cast<uint32_t>(encoded & 0x7fU) * multiplier;
    if (length > max_length) return true;
    if ((encoded & 0x80U) == 0) return true;
    multiplier *= 128U;
  }
  return false;
}

bool mqttWriteByte(uint8_t value) {
  return mqtt_client.write(&value, 1) == 1;
}

bool mqttWriteRemainingLength(uint32_t length) {
  do {
    uint8_t encoded = length % 128U;
    length /= 128U;
    if (length) encoded |= 0x80U;
    if (!mqttWriteByte(encoded)) return false;
  } while (length);
  return true;
}

bool mqttWriteString(const char *value) {
  const uint16_t len = value ? strlen(value) : 0;
  uint8_t header[2] = {
    static_cast<uint8_t>(len >> 8),
    static_cast<uint8_t>(len & 0xffU)
  };
  if (mqtt_client.write(header, sizeof(header)) != sizeof(header)) return false;
  return len == 0 || mqtt_client.write(reinterpret_cast<const uint8_t *>(value), len) == len;
}

String mqttClientId() {
  return "mymota_" + chipIdHex();
}

void mqttStop() {
  mqtt_client.stop();
  last_mqtt_io = 0;
  last_mqtt_rx = 0;
  last_mqtt_ping = 0;
  mqtt_ping_pending = false;
}

void queueMqttConnectHeal() {
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (hasPin(runtime_template.relays[i])) {
      mqtt_pending_relay_mask |= (1U << i);
    }
  }

  if (mqttEnergyReportingEnabled()) {
    last_mqtt_energy_publish = 0;
    last_mqtt_energy_power = NAN;
  }
}

bool mqttConnect() {
  if (!mqttConfigured() || WiFi.status() != WL_CONNECTED) return false;

  const uint32_t started = millis();
  last_mqtt_connect_attempt = started;
  mqttStop();
  mqtt_client.setTimeout(kMqttConnectTimeoutMs);
  if (!mqtt_client.connect(config.mqtt_host, config.mqtt_port)) {
    recordMqttConnectResult(kMqttConnectTcpFailed, started);
    return false;
  }
  mqtt_client.setTimeout(kMqttIoTimeoutMs);

  const String client_id = mqttClientId();
  const uint32_t remaining_length = 10U + 2U + client_id.length();
  bool ok = mqttWriteByte(0x10) &&
            mqttWriteRemainingLength(remaining_length) &&
            mqttWriteString("MQTT") &&
            mqttWriteByte(0x04) &&
            mqttWriteByte(0x02) &&
            mqttWriteByte(static_cast<uint8_t>(kMqttProtocolKeepaliveSec >> 8)) &&
            mqttWriteByte(static_cast<uint8_t>(kMqttProtocolKeepaliveSec & 0xffU)) &&
            mqttWriteString(client_id.c_str());
  if (!ok) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectWriteFailed, started);
    return false;
  }
  last_mqtt_io = millis();

  uint8_t packet_type = 0;
  uint32_t remaining = 0;
  uint8_t flags = 0;
  uint8_t return_code = 0;
  const uint32_t connack_deadline = millis() + kMqttConnackTimeoutMs;
  ok = mqttReadByteUntil(packet_type, connack_deadline);
  if (!ok) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackTimeout, started);
    return false;
  }
  if (packet_type != kMqttPacketConnack) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackRejected, started);
    return false;
  }
  ok = mqttReadRemainingLengthUntil(remaining, kMqttConnackMaxRemainingLength, connack_deadline);
  if (!ok) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackTimeout, started);
    return false;
  }
  if (remaining != 0x02) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackRejected, started);
    return false;
  }
  ok = mqttReadByteUntil(flags, connack_deadline) &&
       mqttReadByteUntil(return_code, connack_deadline);
  if (!ok) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackTimeout, started);
    return false;
  }
  if (flags != 0x00 || return_code != 0x00) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectConnackRejected, started);
    return false;
  }
  last_mqtt_io = millis();
  last_mqtt_rx = last_mqtt_io;
  last_mqtt_ping = 0;
  mqtt_ping_pending = false;
  recordMqttConnectResult(kMqttConnectOk, started);
  queueMqttConnectHeal();
  return true;
}

bool mqttEnsureConnected() {
  if (mqtt_client.connected()) return true;
  const uint32_t now = millis();
  if (next_mqtt_reconnect && now - next_mqtt_reconnect < kMqttReconnectMs) {
    return false;
  }
  next_mqtt_reconnect = now;
  return mqttConnect();
}

bool mqttPublish(const char *topic, const char *payload) {
  if (!mqttEnsureConnected()) return false;

  const uint16_t topic_len = topic ? strlen(topic) : 0;
  const uint16_t payload_len = payload ? strlen(payload) : 0;
  if (topic_len == 0) return false;

  const uint32_t remaining_length = 2U + topic_len + payload_len;
  const bool ok = mqttWriteByte(0x30) &&
                  mqttWriteRemainingLength(remaining_length) &&
                  mqttWriteString(topic) &&
                  (payload_len == 0 || mqtt_client.write(reinterpret_cast<const uint8_t *>(payload), payload_len) == payload_len);
  if (!ok) {
    mqttStop();
    return false;
  }
  last_mqtt_io = millis();
  return true;
}

String mqttRelayTopic(uint8_t relay) {
  String topic;
  topic.reserve(kMqttTopicMaxLen + 16);
  topic += F("stat/");
  topic += config.mqtt_topic;
  topic += F("/");
  if (runtime_template.relay_count <= 1) {
    topic += F("POWER");
  } else {
    topic += F("POWER");
    topic += String(relay + 1);
  }
  return topic;
}

void scheduleMqttRelayPublish(uint8_t relay) {
  if (!mqttConfigured()) return;
  if (relay >= kMaxRelays) return;
  mqtt_pending_relay_mask |= (1U << relay);
}

bool mqttPublishRelayState(uint8_t relay) {
  if (relay >= runtime_template.relay_count || !hasPin(runtime_template.relays[relay])) return true;
  const String topic = mqttRelayTopic(relay);
  const bool ok = mqttPublish(topic.c_str(), relay_state[relay] ? "ON" : "OFF");
  if (ok) {
    last_mqtt_state_publish = millis();
  }
  return ok;
}

bool mqttPublishAllRelayStates() {
  bool ok = true;
  bool published = false;
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (!hasPin(runtime_template.relays[i])) continue;
    published = true;
    if (!mqttPublishRelayState(i)) {
      ok = false;
      break;
    }
  }
  if (ok && published) {
    last_mqtt_state_publish = millis();
  }
  return ok;
}

String mqttEnergyTopic() {
  String topic;
  topic.reserve(kMqttTopicMaxLen + 16);
  topic += F("stat/");
  topic += config.mqtt_topic;
  topic += F("/STATUS8");
  return topic;
}

bool mqttEnergyReportingEnabled() {
  return energy.present && (config.energy_mqtt_interval > 0 || config.energy_mqtt_change_percent_x10 > 0);
}

bool mqttEnergyReportReady() {
  if (!interruptPinSupported(energy.cf1_pin)) return true;
  if (!energy_report_boot_settled && millis() >= kEnergyReportBootSettleMs) {
    energy_report_boot_settled = true;
  }
  return energy.cf1_voltage_pulse_length > 0 || energy_report_boot_settled;
}

bool mqttEnergyPowerChangedEnough() {
  if (config.energy_mqtt_change_percent_x10 == 0) return false;
  if (isnan(last_mqtt_energy_power)) return true;

  const float delta = fabs(energy.power - last_mqtt_energy_power);
  const float baseline = fabs(last_mqtt_energy_power);
  if (baseline < 0.001f) {
    return delta > 0.0f;
  }

  return (delta * 1000.0f) >= (baseline * static_cast<float>(config.energy_mqtt_change_percent_x10));
}

bool mqttPublishEnergyStatus() {
  if (!energy.present) return true;

  const String topic = mqttEnergyTopic();
  String payload;
  payload.reserve(150);
  payload += F("{\"StatusSNS\":{\"ENERGY\":{\"Total\":");
  payload += String(reportedEnergyTotalKwh(), 4);
  payload += F(",\"Power\":");
  payload += String(energy.power, 2);
  payload += F(",\"Voltage\":");
  payload += String(energy.voltage, 1);
  payload += F(",\"Current\":");
  payload += String(energy.current, 3);
  payload += F("}}}");

  const bool ok = mqttPublish(topic.c_str(), payload.c_str());
  if (ok) {
    last_mqtt_energy_publish = millis();
    last_mqtt_energy_power = energy.power;
  }
  return ok;
}

void maintainMqttEnergyReports(uint32_t now) {
  if (!mqttEnergyReportingEnabled()) {
    last_mqtt_energy_publish = 0;
    last_mqtt_energy_power = NAN;
    return;
  }
  if (!mqttEnergyReportReady()) return;

  bool publish = last_mqtt_energy_publish == 0 || isnan(last_mqtt_energy_power);
  if (!publish && config.energy_mqtt_interval > 0) {
    const uint32_t interval_ms = static_cast<uint32_t>(config.energy_mqtt_interval) * 1000UL;
    publish = now - last_mqtt_energy_publish >= interval_ms;
  }
  if (!publish) {
    publish = mqttEnergyPowerChangedEnough();
  }
  if (publish) {
    mqttPublishEnergyStatus();
  }
}

bool mqttProcessInboundPacket() {
  uint8_t packet_type = 0;
  uint32_t remaining = 0;
  const uint32_t deadline = millis() + kMqttInboundReadTimeoutMs;

  if (!mqttReadByteUntil(packet_type, deadline)) return false;
  if (packet_type != kMqttPacketPingresp) return false;
  if (!mqttReadRemainingLengthUntil(remaining, 0, deadline)) return false;
  if (remaining != 0) return false;

  last_mqtt_ping = 0;
  mqtt_ping_pending = false;
  return true;
}

bool mqttProcessInbound() {
  uint8_t packet_count = 0;
  while (mqtt_client.available() && packet_count < kMqttInboundPacketLimit) {
    if (!mqttProcessInboundPacket()) {
      mqttStop();
      return false;
    }
    packet_count++;
  }
  return true;
}

void maintainMqtt() {
  if (update_mqtt_paused) {
    mqttStop();
    return;
  }

  if (!mqttConfigured() || WiFi.status() != WL_CONNECTED) {
    mqttStop();
    clearMqttButtonQueue();
    last_mqtt_energy_publish = 0;
    last_mqtt_energy_power = NAN;
    return;
  }

  uint32_t now = millis();
  expireMqttButtonQueue(now);
  if (!mqttEnsureConnected()) return;

  now = millis();
  if (!mqttProcessInbound()) return;
  now = millis();

  if ((last_mqtt_rx && now - last_mqtt_rx >= kMqttBrokerSilenceTimeoutMs) ||
      (mqtt_ping_pending && last_mqtt_ping && now - last_mqtt_ping >= kMqttBrokerSilenceTimeoutMs)) {
    mqttStop();
    return;
  }

  if (now - last_mqtt_io >= (static_cast<uint32_t>(kMqttProtocolKeepaliveSec) * 1000UL)) {
    if (mqttWriteByte(kMqttPacketPingreq) && mqttWriteByte(0x00)) {
      last_mqtt_io = now;
      last_mqtt_ping = now;
      mqtt_ping_pending = true;
    } else {
      mqttStop();
      return;
    }
  }

  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    const uint16_t mask = 1U << i;
    if (!(mqtt_pending_relay_mask & mask)) continue;
    if (!mqttPublishRelayState(i)) return;
    mqtt_pending_relay_mask &= ~mask;
  }
  now = millis();

  while (mqtt_button_queue_count > 0) {
    MqttButtonPending &slot = mqtt_button_queue[mqtt_button_queue_head];
    if (mqttButtonQueueExpired(slot, now)) {
      dropMqttButtonQueueHead();
      continue;
    }
    if (!mqttPublish(slot.topic, slot.payload)) return;
    dropMqttButtonQueueHead();
  }
  now = millis();

  if (config.mqtt_keepalive > 0 && runtime_template.relay_count > 0) {
    const uint32_t interval_ms = static_cast<uint32_t>(config.mqtt_keepalive) * 1000UL;
    if (now - last_mqtt_state_publish >= interval_ms) {
      mqttPublishAllRelayStates();
    }
  }

  maintainMqttEnergyReports(now);
}

void IRAM_ATTR energyCfInterrupt() {
  const uint32_t us = micros();
  if (energy.load_off) {
    energy.cf_pulse_last_time = us;
    energy.load_off = false;
  } else {
    const uint32_t length = us - energy.cf_pulse_last_time;
    energy.cf_pulse_last_time = us;
    energy.cf_pulse_length = length;
    energy.cf_summed_pulse_length += length;
    energy.cf_pulse_counter++;
  }
}

void IRAM_ATTR energyCf1Interrupt() {
  const uint32_t us = micros();
  const uint32_t length = us - energy.cf1_pulse_last_time;
  energy.cf1_pulse_last_time = us;
  if (energy.cf1_timer > 2 && energy.cf1_timer < 8) {
    energy.cf1_pulse_length = length;
    energy.cf1_summed_pulse_length += length;
    energy.cf1_pulse_counter++;
    if (energy.cf1_pulse_counter >= kEnergyCf1SampleCount) {
      energy.cf1_timer = 8;
    }
  }
}

void writeEnergySelector(bool select_ui_flag) {
  energy.select_ui_flag = select_ui_flag;
  if (!energy.present || !digitalPinSupported(energy.sel_pin)) return;
  digitalWrite(energy.sel_pin, select_ui_flag ? HIGH : LOW);
}

void setupEnergyMonitor() {
  memset(&energy, 0, sizeof(energy));
  energy.cf_pin = runtime_template.energy_cf_pin;
  energy.cf1_pin = runtime_template.energy_cf1_pin;
  energy.sel_pin = runtime_template.energy_sel_pin;
  energy.hjl = runtime_template.energy_hjl;
  energy.present = interruptPinSupported(energy.cf_pin);
  energy.load_off = true;
  energy.power_ratio = energy.hjl ? kHjlPowerRatio : kHlwPowerRatio;
  energy.voltage_ratio = energy.hjl ? kHjlVoltageRatio : kHlwVoltageRatio;
  energy.current_ratio = energy.hjl ? kHjlCurrentRatio : kHlwCurrentRatio;
  energy.select_ui_flag = false;
  energy.ui_flag = !runtime_template.energy_sel_inverted;
  energy.total_kwh = ukwhToKwh(energy_journal_saved_ukwh);
  energy.last_update_ms = millis();
  energy.last_integrate_ms = millis();

  if (!energy.present) return;

  if (digitalPinSupported(energy.sel_pin)) {
    pinMode(energy.sel_pin, OUTPUT);
    writeEnergySelector(energy.select_ui_flag);
  }
  if (interruptPinSupported(energy.cf1_pin)) {
    pinMode(energy.cf1_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(energy.cf1_pin), energyCf1Interrupt, FALLING);
  }
  pinMode(energy.cf_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(energy.cf_pin), energyCfInterrupt, FALLING);
}

void updateDeviceLeds(bool force);

void setRelay(uint8_t relay, bool on) {
  if (relay >= kMaxRelays || !hasPin(runtime_template.relays[relay])) return;
  const bool changed = relay_state[relay] != on;
  const bool was_on = relay_state[relay];
  relay_state[relay] = on;
  writeAssignedPin(runtime_template.relays[relay], on);
  if (changed) {
    scheduleMqttRelayPublish(relay);
    if (energy.present && relay == 0 && was_on && !on) {
      energy_persist_requested = true;
    }
    updateDeviceLeds(true);
  }
}

void toggleRelay(uint8_t relay) {
  if (relay >= kMaxRelays) return;
  setRelay(relay, !relay_state[relay]);
}

bool mqttQueueButtonAction(uint8_t button, bool hold) {
  if (button >= kMaxButtons) return false;
  if (!mqttConfigured()) return false;
  String topic = expandButtonActionText(buttonActionTarget(button, hold), button, hold);
  String payload = expandButtonActionText(buttonActionPayload(button, hold), button, hold);
  topic.trim();
  if (topic.length() == 0 || payload.length() == 0) return false;
  return pushMqttButtonQueue(topic, payload);
}

bool parseHttpUrl(const String &url, String &host, uint16_t &port, String &path) {
  String value = url;
  value.trim();
  if (!value.startsWith(F("http://"))) return false;

  const int host_start = 7;
  const int path_start = value.indexOf('/', host_start);
  String host_port = path_start < 0 ? value.substring(host_start) : value.substring(host_start, path_start);
  host_port.trim();
  if (host_port.length() == 0) return false;

  port = 80;
  const int colon = host_port.lastIndexOf(':');
  if (colon >= 0) {
    const String port_text = host_port.substring(colon + 1);
    uint16_t parsed_port = 0;
    if (!parseUint16Input(port_text, 1, 65535U, parsed_port)) return false;
    port = parsed_port;
    host = host_port.substring(0, colon);
  } else {
    host = host_port;
  }
  host.trim();
  if (host.length() == 0 || host.indexOf(' ') >= 0) return false;

  path = path_start < 0 ? String(F("/")) : value.substring(path_start);
  if (path.length() == 0) path = F("/");
  return true;
}

bool runWebhookAction(uint8_t button, bool hold) {
  if (WiFi.status() != WL_CONNECTED) return false;

  const String url = expandButtonActionText(buttonActionTarget(button, hold), button, hold);
  String host;
  uint16_t port = 80;
  String path;
  if (!parseHttpUrl(url, host, port, path)) return false;

  WiFiClient client;
  client.setTimeout(kWebhookConnectTimeoutMs);
  if (!client.connect(host.c_str(), port)) return false;

  String request;
  request.reserve(path.length() + host.length() + 90);
  request += F("GET ");
  request += path;
  request += F(" HTTP/1.1\r\nHost: ");
  request += host;
  request += F("\r\nConnection: close\r\nUser-Agent: myMota/");
  request += F(MYMOTA_VERSION);
  request += F("\r\n\r\n");
  if (client.print(request) != request.length()) {
    client.stop(kWebhookStopTimeoutMs);
    return false;
  }
  client.flush(kWebhookFlushTimeoutMs);
  client.stop(kWebhookStopTimeoutMs);
  return true;
}

bool runButtonAction(uint8_t button, uint8_t action, bool hold) {
  if (action == kButtonActionRelayToggle) {
    uint8_t relay = 0;
    if (buttonRelayTarget(button, relay)) {
      toggleRelay(relay);
      return true;
    }
  } else if (action == kButtonActionMqtt) {
    mqttQueueButtonAction(button, hold);
  } else if (action == kButtonActionWebhook) {
    runWebhookAction(button, hold);
  }
  return false;
}

void updateDeviceLeds(bool force = false) {
  const uint32_t now = millis();
  if (!force && now - last_led_update < kLedUpdateMs) return;
  last_led_update = now;

  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    const PinAssignment *assignment = ledOutputAssignment(i);
    if (!assignment || !hasLedOutput(i)) continue;
    writeAssignedPin(*assignment, ledOutputOn(i));
  }
}

float readAdcTemperatureC(uint16_t raw) {
  if (raw == 0) return NAN;
  const float denominator = (1023.0f * 3.3f) - static_cast<float>(raw);
  if (denominator <= 0.0f) return NAN;
  const float resistance = kAnalogNtcBridgeResistance * static_cast<float>(raw) / denominator;
  if (resistance <= 0.0f) return NAN;
  const float kelvin = 1.0f / ((1.0f / kAnalogT0Kelvin) + (log(resistance / kAnalogNtcResistance) / kAnalogNtcBeta));
  return kelvin - 273.15f;
}

uint16_t readAdcRaw() {
  uint32_t total = 0;
  for (uint8_t i = 0; i < 4; i++) {
    total += analogRead(A0);
    delay(1);
  }
  return total / 4;
}

void maintainAdc() {
  if (!runtime_template.adc_temp) return;
  const uint32_t now = millis();
  if (now - last_adc_update < kAdcUpdateMs) return;
  last_adc_update = now;
  adc_raw = readAdcRaw();
  adc_temperature_c = readAdcTemperatureC(adc_raw);
}

void maintainButtons() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;
    const bool raw = readAssignedPressed(runtime_template.buttons[i]);
    if (raw != button_state[i].raw_pressed) {
      button_state[i].raw_pressed = raw;
      button_state[i].changed_at = now;
    }
    if ((now - button_state[i].changed_at) >= config.button_debounce_ms && raw != button_state[i].stable_pressed) {
      button_state[i].stable_pressed = raw;
      bool led_handled = false;
      if (raw) {
        button_state[i].pressed_at = now;
        button_state[i].hold_emitted = false;
        if (config.button_hold_action[i] == kButtonActionNone) {
          led_handled = runButtonAction(i, config.button_press_action[i], false);
          button_state[i].hold_emitted = true;
        }
      } else {
        if (!button_state[i].hold_emitted) {
          led_handled = runButtonAction(i, config.button_press_action[i], false);
        }
        button_state[i].hold_emitted = false;
      }
      if (!led_handled) updateDeviceLeds(true);
    }
    if (button_state[i].stable_pressed &&
        !button_state[i].hold_emitted &&
        now - button_state[i].pressed_at >= config.button_hold_ms) {
      button_state[i].hold_emitted = true;
      if (!runButtonAction(i, config.button_hold_action[i], true)) {
        updateDeviceLeds(true);
      }
    }
  }
}

void maintainEnergy() {
  if (!energy.present) return;
  const uint32_t now = millis();
  if (now - energy.last_update_ms >= kEnergyUpdateMs) {
    energy.last_update_ms = now;
    const uint32_t now_us = micros();

    noInterrupts();
    if (energy.cf_pulse_last_time && now_us - energy.cf_pulse_last_time > kEnergyNoPulseTimeoutUs) {
      energy.cf_pulse_length = 0;
      energy.load_off = true;
    }
    const uint32_t cf_length = energy.cf_pulse_length;
    const uint32_t cf_sum = energy.cf_summed_pulse_length;
    const uint32_t cf_count = energy.cf_pulse_counter;
    const bool load_off = energy.load_off;
    energy.cf_summed_pulse_length = 0;
    energy.cf_pulse_counter = 0;
    interrupts();

    energy.cf_power_pulse_length = (cf_count && !load_off) ? (cf_sum / cf_count) : cf_length;
    const bool power_on = (runtime_template.relay_count == 0) || relay_state[0];
    if (energy.cf_power_pulse_length && power_on && !load_off) {
      const uint32_t watts_x10 = (energy.power_ratio * kHlwPowerCal) / energy.cf_power_pulse_length;
      energy.power = static_cast<float>(watts_x10) / 10.0f;
      energy.power_retry = 1;
    } else if (energy.power_retry) {
      energy.power_retry--;
    } else {
      energy.power = 0.0f;
    }

    if (interruptPinSupported(energy.cf1_pin)) {
      noInterrupts();
      energy.cf1_timer++;
      const bool ready = energy.cf1_timer >= 8;
      uint32_t cf1_sum = 0;
      uint32_t cf1_count = 0;
      if (ready) {
        cf1_sum = energy.cf1_summed_pulse_length;
        cf1_count = energy.cf1_pulse_counter;
        energy.cf1_summed_pulse_length = 0;
        energy.cf1_pulse_counter = 0;
        energy.cf1_timer = 0;
      }
      interrupts();

      if (ready) {
        const bool has_selector = digitalPinSupported(energy.sel_pin);
        if (has_selector) {
          writeEnergySelector(!energy.select_ui_flag);
        }
        const uint32_t cf1_avg = cf1_count ? (cf1_sum / cf1_count) : 0;
        if (!has_selector || energy.select_ui_flag == energy.ui_flag) {
          energy.cf1_voltage_pulse_length = cf1_avg;
          if (cf1_avg && power_on) {
            const uint32_t volts_x10 = (energy.voltage_ratio * kHlwVoltageCal) / cf1_avg;
            energy.voltage = static_cast<float>(volts_x10) / 10.0f;
          } else {
            energy.voltage = 0.0f;
          }
        } else {
          energy.cf1_current_pulse_length = cf1_avg;
          if (cf1_avg && energy.power > 0.0f) {
            const uint32_t milliamps = (energy.current_ratio * kHlwCurrentCal) / cf1_avg;
            energy.current = static_cast<float>(milliamps) / 1000.0f;
          } else {
            energy.current = 0.0f;
          }
        }
      }
    }
  }

  if (now - energy.last_integrate_ms >= kEnergyIntegrateMs) {
    const uint32_t elapsed = now - energy.last_integrate_ms;
    energy.last_integrate_ms = now;
    energy.total_kwh += (energy.power * static_cast<float>(elapsed)) / 3600000000.0f;
  }

  persistEnergyTotal(false);
}

void setupDevicePins() {
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    relay_state[i] = false;
    if (!hasPin(runtime_template.relays[i])) continue;
    writeAssignedPin(runtime_template.relays[i], false);
    pinMode(runtime_template.relays[i].pin, OUTPUT);
    writeAssignedPin(runtime_template.relays[i], false);
  }

  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    const PinAssignment *assignment = ledOutputAssignment(i);
    if (!assignment || !hasLedOutput(i)) continue;
    writeAssignedPin(*assignment, false);
    pinMode(assignment->pin, OUTPUT);
    writeAssignedPin(*assignment, false);
  }

  for (uint8_t i = 0; i < kMaxButtons; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;
    pinMode(runtime_template.buttons[i].pin, runtime_template.buttons[i].no_pullup ? INPUT : INPUT_PULLUP);
    const bool pressed = readAssignedPressed(runtime_template.buttons[i]);
    button_state[i] = {pressed, pressed, false, millis(), millis()};
  }

  setupEnergyMonitor();
  maintainAdc();
  updateDeviceLeds(true);
}

void maintainDevice() {
  maintainButtons();
  maintainEnergy();
  maintainAdc();
  updateDeviceLeds();
}

void appendHeader(String &page, const __FlashStringHelper *title, bool show_spinner = false) {
  page += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>");
  page += F("myMota");
  if (config_ok && config.hostname[0] != '\0') {
    page += F(" &middot; ");
    page += htmlEscape(config.hostname);
  }
  page += F("</title><style>:root{--bg:#f6f7f9;--panel:#fff;--line:#d8dee8;--text:#17202a;--muted:#687386;--ok:#177245;--bad:#a23a36;--accent:#1f7a5f;--accent2:#205c8a}");
  page += F("*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:Arial,sans-serif;font-size:15px;line-height:1.4}");
  page += F(".top{background:#17202a;color:#fff;border-bottom:4px solid var(--accent);padding:18px 16px}.topin{max-width:1080px;margin:0 auto;display:flex;align-items:end;justify-content:space-between;gap:12px;flex-wrap:wrap}");
  page += F(".brand{font-size:28px;font-weight:700;letter-spacing:0}.brand span{color:#7dd3aa}.sub{color:#c7d0dc;font-size:13px}.meta{display:flex;align-items:center;gap:8px}");
  page += F(".spin{width:13px;height:13px;border:2px solid rgba(255,255,255,.35);border-top-color:#7dd3aa;border-radius:50%;opacity:.55}.spin.active{opacity:1;animation:rot .7s linear infinite}@keyframes rot{to{transform:rotate(360deg)}}main{max-width:1080px;margin:18px auto 28px;padding:0 14px}");
  page += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px;box-shadow:0 1px 2px rgba(0,0,0,.04)}.wide{grid-column:1/-1}");
  page += F(".panel h2{font-size:17px;margin:0 0 12px}.kv{display:grid;grid-template-columns:minmax(110px,42%) 1fr;gap:8px 12px}.kv span,.hint{color:var(--muted)}.kv div{min-width:0}");
  page += F("code{background:#eef2f6;border:1px solid #dce3ea;border-radius:4px;padding:1px 4px;word-break:break-word}.pill{display:inline-block;border-radius:999px;padding:2px 8px;background:#eef2f6;color:#364152}.pill.ok{background:var(--ok);color:#fff}.pill.bad{background:var(--bad);color:#fff}.panel h2 .pill{font-size:13px;font-weight:400;vertical-align:1px}.ok{color:var(--ok)}.bad{color:var(--bad)}.muted{color:var(--muted)}");
  page += F(".note{background:#eef2f6;border:1px solid #dce3ea;border-radius:6px;padding:10px;margin:10px 0}.note p{margin:0 0 7px}.tokens{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:8px}.tokens div{display:flex;flex-direction:column;gap:3px}.button-block{border-top:1px solid var(--line);margin-top:12px;padding-top:12px}.action-extra{display:none}.action-extra.show{display:block}.payload-row.hidden{display:none}");
  page += F("form{margin:0}.row{margin:10px 0}label{display:block;font-weight:600;color:#344054}input,button,select,textarea{font:inherit}input,select,textarea{width:100%;margin-top:4px;padding:9px;border:1px solid #b9c4d0;border-radius:6px;background:#fff}textarea{min-height:92px;resize:vertical}");
  page += F("button,.btn{display:inline-block;margin:4px 4px 0 0;padding:8px 12px;border:1px solid var(--accent);border-radius:6px;background:var(--accent);color:#fff;text-decoration:none;cursor:pointer}.secondary{background:#fff;color:var(--accent2);border-color:#9eb7cf}.danger{background:#fff;color:var(--bad);border-color:#d4aaa7}.inline{display:inline}.actions{display:flex;flex-wrap:wrap;gap:6px}.inline button{margin:0 4px 0 0}.list{margin:0;padding-left:18px}@media(max-width:520px){.kv{grid-template-columns:1fr}.brand{font-size:24px}}</style></head><body>");
  page += F("<header class='top'><div class='topin'><div><div class='brand'>my<span>Mota</span></div><div class='sub'>ESP8266/ESP8285 firmware</div></div><div class='sub meta'><span>");
  page += F(MYMOTA_VERSION);
  page += F(" / ");
  page += F(MYMOTA_TARGET);
  page += F("</span>");
  if (show_spinner) {
    page += F("<span id='poll-spin' class='spin active'></span>");
  }
  page += F("</div></div></header><main>");
}

void appendFooter(String &page, bool live_poll = true, bool reboot_wait = false) {
  page += F("<script>var ls=Date.now();function ok(){ls=Date.now();var e=document.getElementById('poll-spin');if(e)e.className='spin active';}");
  page += F("function ck(){var e=document.getElementById('poll-spin');if(e&&Date.now()-ls>5000)e.className='spin';}");
  page += F("function fh(){return fetch('/health',{cache:'no-store'}).then(function(r){if(!r.ok)throw Error();return r.json();}).then(function(d){ok();return d;});}");
  page += F("function t(i,v){var e=document.getElementById(i);if(e)e.textContent=v;}");
  page += F("function p(i,v,c){var e=document.getElementById(i);if(e){e.textContent=v;e.className=c;}}");
  page += F("function fmt(v,d,u){return v==null?'n/a':Number(v).toFixed(d)+(u||'');}");
  page += F("function live(){fh().then(function(d){");
  page += F("t('live-heap',d.heap+' bytes');t('live-uptime',d.uptime+'s');t('live-active-phy',d.active_phy);");
  page += F("if(d.perf){t('live-loop-load',d.perf.loop_load+'%');t('live-loop-hz',d.perf.loop_hz+'/s');t('live-loop-max',Number(d.perf.loop_max_us/1000).toFixed(1)+' ms');}");
  page += F("t('live-recovery',d.recovery.fast_boot_count+'/'+d.recovery.limit);");
  page += F("p('live-wifi',d.wifi?'connected':'disconnected',d.wifi?'pill ok':'pill bad');t('live-ssid',d.wifi_ssid||'n/a');t('live-ip',d.ip||'n/a');t('live-rssi',d.rssi==null?'n/a':d.rssi+' dBm');");
  page += F("p('live-mqtt',d.mqtt.enabled?(d.mqtt.connected?'connected':'disconnected'):'not configured',d.mqtt.enabled?(d.mqtt.connected?'pill ok':'pill bad'):'pill');");
  page += F("if(d.power){for(var i=0;i<d.power.length;i++){if(d.power[i]!==null)p('live-relay-'+i,d.power[i]?'on':'off',d.power[i]?'pill ok':'pill bad');}}");
  page += F("if(d.buttons){for(var b=0;b<d.buttons.length;b++){if(d.buttons[b])p('live-button-'+b,d.buttons[b].pressed?'pressed':'released',d.buttons[b].pressed?'pill ok':'pill bad');}}");
  page += F("if(d.leds){for(var l=0;l<d.leds.length;l++){if(d.leds[l])p('live-led-'+l,d.leds[l].on?'on':'off',d.leds[l].on?'pill ok':'pill bad');}}");
  page += F("if(d.energy){t('live-energy-power',fmt(d.energy.power,1,' W'));t('live-energy-voltage',fmt(d.energy.voltage,1,' V'));t('live-energy-current',fmt(d.energy.current,3,' A'));t('live-energy-total',fmt(d.energy.total_kwh,4,' kWh'));t('live-energy-offset',fmt(d.energy.offset_kwh,4,' kWh'));}");
  page += F("t('live-temp',d.temperature_c==null?'n/a':Number(d.temperature_c).toFixed(1)+' C');t('live-adc-raw',d.adc_raw==null?'n/a':d.adc_raw);");
  page += F("}).catch(function(){});}");
  page += F("function ba(s){var k=s.getAttribute('data-key'),v=s.value,b=document.getElementById('extra-'+k);if(!b)return;var t=b.querySelector('.target-input'),p=b.querySelector('.payload-input'),pr=b.querySelector('.payload-row'),tl=b.querySelector('.target-label'),h=b.querySelector('.action-hint');b.className=(v=='2'||v=='3')?'action-extra show':'action-extra';if(v=='2'){if(t&&(!t.value||t.value.indexOf('http://')==0))t.value=t.getAttribute('data-default-topic');if(p&&!p.value)p.value=p.getAttribute('data-default-payload');if(tl)tl.textContent='MQTT topic';if(pr)pr.className='payload-row';if(h)h.textContent='Publishes this topic and payload through the configured MQTT broker.';}else if(v=='3'){if(tl)tl.textContent='Webhook URL';if(pr)pr.className='payload-row hidden';if(h)h.textContent='Executes an HTTP GET request; only http:// URLs are supported.';}}");
  page += F("function tp(s){var o=s.options[s.selectedIndex],t=document.getElementById('template-json');if(o&&t&&o.getAttribute('data-json')){t.value=o.getAttribute('data-json');s.selectedIndex=0;}}");
  page += F("function bi(){var a=document.querySelectorAll('.button-action');for(var i=0;i<a.length;i++){a[i].onchange=function(){ba(this)};ba(a[i]);}}bi();");
  if (live_poll) {
    page += F("setInterval(live,1000);setInterval(ck,1000);live();");
  }
  if (reboot_wait) {
    page += F("var rb=");
    page += String(boot_id);
    page += F(";function rw(){fh().then(function(d){if(d.boot_id==null||d.boot_id!=rb)location.href='/';else setTimeout(rw,1000);}).catch(function(){setTimeout(rw,1000);});}setTimeout(rw,4000);");
  }
  page += F("</script></main></body></html>");
}

void sendHtml(String &page) {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("text/html"), page);
}

void beginStreamedResponse(const char *content_type) {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, content_type, String());
}

void flushStreamChunk(String &chunk) {
  if (chunk.length() == 0) return;
  server.sendContent(chunk);
  chunk.remove(0);
  delay(0);
}

void endStreamedResponse(String &chunk) {
  flushStreamChunk(chunk);
  server.chunkedResponseFinalize();
}

void appendStatusBlock(String &page) {
  page += F("<section class='panel wide'><h2>System Status</h2><div class='kv'>");
  page += F("<span>Version</span><div><code>");
  page += F(MYMOTA_VERSION);
  page += F("</code> <code>");
  page += F(MYMOTA_TARGET);
  page += F("</code></div><span>Chip</span><div><code>");
  page += chipIdHex();
  page += F("</code></div><span>Hostname</span><div><code>");
  page += htmlEscape(config.hostname);
  page += F("</code></div><span>Heap</span><div><code id='live-heap'>");
  page += String(ESP.getFreeHeap());
  page += F(" bytes</code></div><span>Uptime</span><div><code id='live-uptime'>");
  page += String(millis() / 1000);
  page += F("s</code></div><span>Loop load</span><div><code id='live-loop-load'>");
  page += String(perf_last_loop_load);
  page += F("%</code> app busy</div><span>Loop rate</span><div><code id='live-loop-hz'>");
  page += String(perf_last_loop_hz);
  page += F("/s</code></div><span>Slowest loop</span><div><code id='live-loop-max'>");
  page += String(static_cast<float>(perf_last_loop_max_us) / 1000.0f, 1);
  page += F(" ms</code></div><span>PHY mode</span><div><code>");
  page += phyModeName(config.phy_mode);
  page += F("</code> configured <code id='live-active-phy'>");
  page += phyModeName(WiFi.getPhyMode());
  page += F("</code> active</div><span>Recovery guard</span><div><code id='live-recovery'>");
  page += String(boot_recovery_count);
  page += F("/");
  page += String(kBootRecoveryLimit);
  page += F("</code> clears after <code>");
  page += String(kBootRecoveryStableMs / 1000);
  page += F("s</code>");
  if (boot_recovery_factory_reset) {
    page += F(" <span class='pill bad'>factory reset</span>");
  }
  page += F("</div>");

  if (WiFi.status() == WL_CONNECTED) {
    page += F("<span>Wi-Fi</span><div><span id='live-wifi' class='pill ok'>connected</span> <code id='live-ssid'>");
    page += htmlEscape(WiFi.SSID());
    page += F("</code></div><span>IP</span><div><code id='live-ip'>");
    page += ipToString(WiFi.localIP());
    page += F("</code></div><span>RSSI</span><div><code id='live-rssi'>");
    page += String(WiFi.RSSI());
    page += F(" dBm</code></div>");
  } else {
    page += F("<span>Wi-Fi</span><div><span id='live-wifi' class='pill bad'>disconnected</span> <code id='live-ssid'>n/a</code></div>");
    page += F("<span>IP</span><div><code id='live-ip'>n/a</code></div><span>RSSI</span><div><code id='live-rssi'>n/a</code></div>");
  }

  if (ap_started) {
    page += F("<span>Setup AP</span><div><code>");
    page += htmlEscape(WiFi.softAPSSID());
    page += F("</code> <span class='pill ok'>open</span> at <code>");
    page += ipToString(WiFi.softAPIP());
    page += F("</code></div>");
  }
  page += F("</div></section>");
}

void appendTemplateStatus(String &page) {
  page += F("<section class='panel'><h2>Template</h2>");
  if (!runtime_template.enabled) {
    page += F("<p class='muted'>No template configured.</p>");
  } else {
    page += F("<div class='kv'><span>Name</span><div><code>");
    page += htmlEscape(runtime_template.name);
    page += F("</code></div><span>Base</span><div><code>");
    page += String(runtime_template.base);
    page += F("</code> flag <code>");
    page += String(runtime_template.flag);
    page += F("</code></div><span>GPIO roles</span><div><code>");
    page += String(runtime_template.relay_count);
    page += F("</code> relays <code>");
    page += String(runtime_template.button_count);
    page += F("</code> buttons <code>");
	    page += String(runtime_template.led_count);
	    page += F("</code> LEDs</div>");
	    if (energy.present) {
	      page += F("<span>Energy</span><div><code>");
      if (energy.hjl) {
        page += F("HJL/BL0937");
      } else {
        page += F("HLW8012");
      }
      page += F("</code> CF <code>");
      page += pinName(energy.cf_pin);
      page += F("</code>, CF1 <code>");
      page += pinName(energy.cf1_pin);
      page += F("</code>, SEL <code>");
      page += pinName(energy.sel_pin);
      page += F("</code></div>");
    }
    if (runtime_template.adc_temp) {
      page += F("<span>ADC temperature</span><div><code id='live-temp'>");
      if (isnan(adc_temperature_c)) {
        page += F("n/a");
      } else {
        page += String(adc_temperature_c, 1);
        page += F(" C");
      }
      page += F("</code> raw <code id='live-adc-raw'>");
      page += String(adc_raw);
      page += F("</code></div>");
    }
    if (hasPin(runtime_template.link_led)) {
      page += F("<span>Link LED</span><div><code>");
      page += pinName(runtime_template.link_led.pin);
      page += F("</code></div>");
    }
    page += F("</div>");
    if (runtime_template.unsupported_count) {
      page += F("<p class='bad'>Unsupported GPIO functions:");
      for (uint8_t i = 0; i < runtime_template.unsupported_count; i++) {
        page += F(" <code>");
        page += pinName(runtime_template.unsupported_pin[i]);
        page += F("=");
        page += String(runtime_template.unsupported_code[i]);
        page += F("</code>");
      }
      page += F("</p>");
    }
  }
  page += F("</section>");
}

void appendDeviceControls(String &page) {
  if (!runtime_template.enabled || (runtime_template.relay_count == 0 && !energy.present)) return;
  page += F("<section class='panel'><h2>Device</h2>");
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (!hasPin(runtime_template.relays[i])) continue;
    page += F("<div class='row'><strong>Relay ");
    page += String(i + 1);
    page += F("</strong> <span class='hint'>on</span> <code>");
    page += pinName(runtime_template.relays[i].pin);
    page += F("</code> ");
    if (relay_state[i]) {
      page += F("<span id='live-relay-");
      page += String(i);
      page += F("' class='pill ok'>on</span>");
    } else {
      page += F("<span id='live-relay-");
      page += String(i);
      page += F("' class='pill bad'>off</span>");
    }
    page += F("<form class='inline' method='post' action='/power'><input type='hidden' name='relay' value='");
    page += String(i + 1);
    page += F("'><span class='actions'><button name='state' value='toggle'>Toggle</button><button name='state' value='on'>On</button><button class='secondary' name='state' value='off'>Off</button></span></form></div>");
  }
  if (energy.present) {
    page += F("<div class='kv'><span>Power</span><div><code id='live-energy-power'>");
    page += String(energy.power, 1);
    page += F(" W</code></div><span>Voltage</span><div><code id='live-energy-voltage'>");
    page += String(energy.voltage, 1);
    page += F(" V</code></div><span>Current</span><div><code id='live-energy-current'>");
    page += String(energy.current, 3);
    page += F(" A</code></div><span>Total</span><div><code id='live-energy-total'>");
    page += String(reportedEnergyTotalKwh(), 4);
    page += F(" kWh</code></div><span>Total offset</span><div><code id='live-energy-offset'>");
    page += String(config.energy_total_offset_kwh, 4);
    page += F(" kWh</code></div></div>");
    page += F("<form method='post' action='/energy'><div class='row'><label>Total kWh offset<br><input name='total_offset_kwh' type='number' min='");
    page += String(kEnergyTotalOffsetMinKwh, 0);
    page += F("' max='");
    page += String(kEnergyTotalOffsetMaxKwh, 0);
    page += F("' step='0.0001' value='");
    page += String(config.energy_total_offset_kwh, 4);
    page += F("'></label></div><div class='row'><label>MQTT report interval seconds<br><input name='energy_report_interval' type='number' min='0' max='");
    page += String(kMqttEnergyIntervalMax);
    page += F("' step='1' value='");
    page += String(config.energy_mqtt_interval);
    page += F("'></label></div><div class='row'><label>MQTT report power change percent<br><input name='energy_report_change_percent' type='number' min='0' max='");
    page += String(kMqttEnergyChangeMaxPercent, 1);
    page += F("' step='0.1' value='");
    page += String(energyMqttChangePercent(), 1);
    page += F("'></label></div><button type='submit'>Save energy</button></form>");
  }
  page += F("</section>");
}

void appendLedAttachmentOption(String &page, uint8_t value, const String &label, uint8_t selected) {
  page += F("<option value='");
  page += String(value);
  page += F("'");
  if (selected == value) {
    page += F(" selected");
  }
  page += F(">");
  page += htmlEscape(label);
  page += F("</option>");
}

void appendLedSettings(String &page) {
  if (!runtime_template.enabled || !hasConfigurableLedOutputs()) return;

  page += F("<section class='panel'><h2>LEDs</h2><form method='post' action='/leds'>");
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    const PinAssignment *assignment = ledOutputAssignment(i);
    if (!assignment || !hasLedOutput(i)) continue;

    const uint8_t selected = config.led_attach[i];
    page += F("<div class='row'><label>");
    page += htmlEscape(ledOutputName(i));
    page += F(" <span class='hint'>");
    page += pinName(assignment->pin);
    page += F("</span> ");
    if (ledOutputOn(i)) {
      page += F("<span id='live-led-");
      page += String(i);
      page += F("' class='pill ok'>on</span>");
    } else {
      page += F("<span id='live-led-");
      page += String(i);
      page += F("' class='pill bad'>off</span>");
    }
    page += F("<br><select name='led");
    page += String(i);
    page += F("'>");
    appendLedAttachmentOption(page, kLedAttachNone, F("Nothing"), selected);
    for (uint8_t relay = 0; relay < runtime_template.relay_count; relay++) {
      if (!hasPin(runtime_template.relays[relay])) continue;
      appendLedAttachmentOption(page, kLedAttachRelayBase + relay, String(F("Relay ")) + String(relay + 1), selected);
    }
    for (uint8_t button = 0; button < runtime_template.button_count; button++) {
      if (!hasPin(runtime_template.buttons[button])) continue;
      appendLedAttachmentOption(page, kLedAttachButtonBase + button, String(F("Button ")) + String(button + 1), selected);
    }
    page += F("</select></label></div>");
  }
  page += F("<button type='submit'>Save LEDs</button></form></section>");
}

void appendButtonActionOption(String &page, uint8_t value, const String &label, uint8_t selected) {
  page += F("<option value='");
  page += String(value);
  page += F("'");
  if (selected == value) {
    page += F(" selected");
  }
  page += F(">");
  page += htmlEscape(label);
  page += F("</option>");
}

void appendButtonActionSelect(String &page, uint8_t button, const char *name, uint8_t selected) {
  page += F("<select class='button-action' data-key='");
  page += name;
  page += String(button);
  page += F("' name='");
  page += name;
  page += String(button);
  page += F("'>");
  appendButtonActionOption(page, kButtonActionNone, F("Nothing"), selected);
  if (buttonActionAvailable(button, kButtonActionRelayToggle)) {
    appendButtonActionOption(page, kButtonActionRelayToggle, F("Relay toggle"), selected);
  }
  appendButtonActionOption(page, kButtonActionMqtt, F("MQTT broadcast"), selected);
  appendButtonActionOption(page, kButtonActionWebhook, F("Webhook exec"), selected);
  page += F("</select>");
}

void appendButtonActionExtra(String &page, uint8_t button, const char *name, bool hold) {
  page += F("<div id='extra-");
  page += name;
  page += String(button);
  page += F("' class='action-extra'><div class='row'><label><span class='target-label'>MQTT topic</span><br><input class='target-input' name='");
  page += name;
  page += F("_target");
  page += String(button);
  page += F("' maxlength='");
  page += String(kButtonActionTargetMaxLen);
  page += F("' data-default-topic='");
  page += htmlEscape(kDefaultButtonMqttTopic);
  page += F("' value='");
  page += htmlEscape(buttonActionTarget(button, hold));
  page += F("'></label></div><div class='row payload-row'><label>MQTT payload<br><textarea class='payload-input' name='");
  page += name;
  page += F("_payload");
  page += String(button);
  page += F("' maxlength='");
  page += String(kButtonActionPayloadMaxLen);
  page += F("' data-default-payload='");
  page += htmlEscape(hold ? kDefaultButtonMqttHoldPayload : kDefaultButtonMqttPressPayload);
  page += F("'>");
  page += htmlEscape(buttonActionPayload(button, hold));
  page += F("</textarea></label></div><p class='hint action-hint'></p></div>");
}

void appendButtonSettings(String &page) {
  if (!runtime_template.enabled || !hasConfigurableButtons()) return;

  page += F("<section class='panel'><h2>Buttons</h2><form method='post' action='/buttons'>");
  page += F("<div class='row'><label>Hold time ms<br><input name='hold_ms' type='number' min='");
  page += String(kButtonHoldMinMs);
  page += F("' max='");
  page += String(kButtonHoldMaxMs);
  page += F("' step='1' value='");
  page += String(config.button_hold_ms);
  page += F("'></label><label>Debounce ms<br><input name='debounce_ms' type='number' min='");
  page += String(kButtonDebounceMinMs);
  page += F("' max='");
  page += String(kButtonDebounceMaxMs);
  page += F("' step='1' value='");
  page += String(config.button_debounce_ms);
  page += F("'></label></div>");
  page += F("<div class='note'><p><strong>Action placeholders</strong></p><div class='tokens'>");
  page += F("<div><code>{BUTTONID}</code><span class='hint'>button number, starting at 1</span></div>");
  page += F("<div><code>{TYPE}</code><span class='hint'>TOGGLE on press, HOLD on hold</span></div>");
  page += F("<div><code>{TOPIC}</code><span class='hint'>current MQTT topic</span></div>");
  page += F("<div><code>{RELAYX_STATE}</code><span class='hint'>relay state, for example {RELAY1_STATE}</span></div>");
  page += F("</div><p class='hint'>MQTT broadcast sends a topic and payload through the configured broker. The default values match the switch action format used by tasmota.js: <code>stat/{TOPIC}/RESULT</code> with a <code>Switch{BUTTONID}</code> payload using <code>{TYPE}</code>.</p></div>");

  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;
    page += F("<div class='button-block'><strong>Button ");
    page += String(i + 1);
    page += F("</strong> <span class='hint'>");
    page += pinName(runtime_template.buttons[i].pin);
    page += F("</span> ");
    if (button_state[i].stable_pressed) {
      page += F("<span id='live-button-");
      page += String(i);
      page += F("' class='pill ok'>pressed</span>");
    } else {
      page += F("<span id='live-button-");
      page += String(i);
      page += F("' class='pill bad'>released</span>");
    }
    page += F("<div class='row'><label>Press<br>");
    appendButtonActionSelect(page, i, "press", config.button_press_action[i]);
    page += F("</label>");
    appendButtonActionExtra(page, i, "press", false);
    page += F("</div><div class='row'><label>Hold<br>");
    appendButtonActionSelect(page, i, "hold", config.button_hold_action[i]);
    page += F("</label>");
    appendButtonActionExtra(page, i, "hold", true);
    page += F("</div></div>");
  }
  page += F("<button type='submit'>Save buttons</button></form></section>");
}

void appendMqttStatus(String &page) {
  page += F("<section class='panel'><h2>MQTT ");
  if (config.mqtt_host[0] == '\0') {
    page += F("<span id='live-mqtt' class='pill'>not configured</span>");
  } else if (mqtt_client.connected()) {
    page += F("<span id='live-mqtt' class='pill ok'>connected</span>");
  } else {
    page += F("<span id='live-mqtt' class='pill bad'>disconnected</span>");
  }
  page += F("</h2><div class='kv'>");
  if (config.mqtt_host[0] == '\0') {
    page += F("<span>Broker</span><div><span class='muted'>not configured</span></div>");
  } else {
    page += F("<span>Broker</span><div><code>");
    page += htmlEscape(config.mqtt_host);
    page += F(":");
    page += String(config.mqtt_port);
    page += F("</code></div>");
  }
  page += F("<span>Topic</span><div><code>");
  page += htmlEscape(config.mqtt_topic);
  page += F("</code></div><span>State keepalive</span><div><code>");
  if (config.mqtt_keepalive == 0) {
    page += F("disabled");
  } else {
    page += String(config.mqtt_keepalive);
    page += F("s");
  }
  page += F("</code></div></div></section>");
}

void appendTemplateForm(String &page) {
  page += F("<section class='panel wide'><h2>Template</h2><form method='post' action='/template'>");
  page += F("<div class='row'><label>Known template<br><select onchange='tp(this)'><option value=''>Select a template</option>");
  page += F("<option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShellyPlugSJson)));
  page += F("'>Shelly Plug S</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateNousA1TJson)));
  page += F("'>NOUS A1T 16A</option></select></label></div>");
  page += F("<div class='row'><label>Tasmota ESP8266 template JSON<br><textarea id='template-json' name='template' rows='5' maxlength='");
  page += String(kTemplateJsonMaxLen);
  page += F("'>");
  page += htmlEscape(currentTemplateJson());
  page += F("</textarea></label></div>");
  page += F("<button type='submit'>Save template</button> <button class='danger' type='submit' name='clear' value='1'>Clear template</button></form></section>");
}

void appendMqttForm(String &page) {
  page += F("<section class='panel'><h2>MQTT Settings</h2><form method='post' action='/mqtt'>");
  page += F("<div class='row'><label>Host<br><input name='host' maxlength='");
  page += String(kMqttHostMaxLen);
  page += F("' value='");
  page += htmlEscape(config.mqtt_host);
  page += F("'></label></div><div class='row'><label>Port<br><input name='port' type='number' min='1' max='65535' value='");
  page += String(config.mqtt_port);
  page += F("'></label></div><div class='row'><label>Topic<br><input name='topic' maxlength='");
  page += String(kMqttTopicMaxLen);
  page += F("' required value='");
  page += htmlEscape(config.mqtt_topic);
  page += F("'></label></div><div class='row'><label>State keepalive seconds<br><input name='keepalive' type='number' min='0' max='");
  page += String(kMqttKeepaliveMax);
  page += F("' value='");
  page += String(config.mqtt_keepalive);
  page += F("'></label></div><button type='submit'>Save MQTT</button></form></section>");
}

void appendPhyModeOption(String &page, uint8_t mode) {
  page += F("<option value='");
  page += String(mode);
  page += F("'");
  if (config.phy_mode == mode) {
    page += F(" selected");
  }
  page += F(">");
  page += phyModeName(mode);
  page += F("</option>");
}

void appendPhyModeSelect(String &page) {
  page += F("<div class='row'><label>PHY mode<br><select name='phy_mode'>");
  appendPhyModeOption(page, kPhyModeAuto);
  appendPhyModeOption(page, WIFI_PHY_MODE_11B);
  appendPhyModeOption(page, WIFI_PHY_MODE_11G);
  appendPhyModeOption(page, WIFI_PHY_MODE_11N);
  page += F("</select></label></div>");
}

void handleRoot() {
  String page;
  page.reserve(kHtmlStreamChunkReserve);
  beginStreamedResponse("text/html");
  appendHeader(page, F("myMota"), true);
  page += F("<div class='grid'>");
  flushStreamChunk(page);
  appendStatusBlock(page);
  flushStreamChunk(page);
  appendTemplateStatus(page);
  flushStreamChunk(page);
  appendDeviceControls(page);
  flushStreamChunk(page);
  appendButtonSettings(page);
  flushStreamChunk(page);
  appendLedSettings(page);
  flushStreamChunk(page);
  appendMqttStatus(page);
  flushStreamChunk(page);
  page += F("<section class='panel'><h2>Wi-Fi</h2><form method='post' action='/wifi'>");
  page += F("<div class='row'><label>SSID<br><input name='ssid' maxlength='32' required value='");
  page += htmlEscape(config.ssid);
  page += F("'></label></div><div class='row'><label>Password<br><input id='wifi-password' type='password' name='password' maxlength='64' autocomplete='current-password' value='");
  page += htmlEscape(config.password);
  page += F("' onfocus=\"this.type='text'\" onclick=\"this.type='text'\"></label></div>");
  page += F("<div class='row'><label>Hostname<br><input name='hostname' maxlength='32' value='");
  page += htmlEscape(config.hostname);
  page += F("'></label></div>");
  appendPhyModeSelect(page);
  page += F("<button type='submit'>Save Wi-Fi</button></form>");
  page += F("<p><a class='btn secondary' href='/scan'>Scan networks</a></p></section>");
  flushStreamChunk(page);

  appendTemplateForm(page);
  flushStreamChunk(page);
  appendMqttForm(page);
  flushStreamChunk(page);

  page += F("<section class='panel'><h2>Firmware</h2><form method='post' action='/update' enctype='multipart/form-data'>");
  page += F("<input type='file' name='firmware' accept='.bin,.bin.gz' required><br><button type='submit'>Upload firmware</button></form>");
  page += F("<p><a class='btn secondary' href='/reboot'>Reboot</a></p>");
  page += F("<form method='post' action='/factory-reset' onsubmit=\"return confirm('Factory reset will delete Wi-Fi, template, MQTT, button, LED, and energy settings. Continue?')\"><button class='danger' type='submit'>Factory reset</button></form></section></div>");
  flushStreamChunk(page);
  appendFooter(page);
  endStreamedResponse(page);
}

void handleScan() {
  String page;
  page.reserve(2600);
  appendHeader(page, F("myMota Scan"));
  page += F("<section class='panel'><h2>Networks</h2>");

  int count = WiFi.scanComplete();
  if (count == WIFI_SCAN_FAILED) {
    WiFi.scanDelete();
    WiFi.scanNetworksAsync([](int) {}, true);
    count = WiFi.scanComplete();
  }

  if (count == WIFI_SCAN_RUNNING) {
    page += F("<p>Scanning for Wi-Fi networks...</p>");
    page += F("<p class='muted'>This page will refresh when results are ready.</p>");
    page += F("<script>setTimeout(function(){location.reload()},1000)</script>");
    page += F("<p><a class='btn secondary' href='/'>Back</a></p></section>");
    appendFooter(page, false);
    sendHtml(page);
    return;
  }

  if (count <= 0) {
    page += F("<p>No networks found.</p>");
  } else {
    page += F("<form method='post' action='/wifi'><div class='row'><label>Password<br><input type='password' name='password' maxlength='64'></label></div>");
    page += F("<div class='row'><label>Hostname<br><input name='hostname' maxlength='32' value='");
    page += htmlEscape(config.hostname);
    page += F("'></label></div>");
    appendPhyModeSelect(page);
    page += F("<ul class='list'>");
    for (int i = 0; i < count; i++) {
      page += F("<li><label><input type='radio' name='ssid' required value='");
      page += htmlEscape(WiFi.SSID(i));
      page += F("'> ");
      page += htmlEscape(WiFi.SSID(i));
      page += F(" <span class='muted'>");
      page += String(WiFi.RSSI(i));
      page += F(" dBm ch ");
      page += String(WiFi.channel(i));
      page += F("</span></label></li>");
    }
    page += F("</ul><button type='submit'>Save Wi-Fi</button></form>");
  }
  WiFi.scanDelete();
  page += F("<p><a class='btn secondary' href='/'>Back</a></p></section>");
  appendFooter(page);
  sendHtml(page);
}

void handleWifiSave() {
  const String ssid = server.arg("ssid");
  const String password = server.arg("password");
  const String hostname = server.arg("hostname");
  uint8_t phy_mode = config.phy_mode;
  char password_to_save[sizeof(config.password)];

  if (ssid.length() == 0 || ssid.length() > 32 || password.length() > 64 || hostname.length() > 32) {
    server.send(400, F("text/plain"), F("Invalid Wi-Fi settings"));
    return;
  }
  if (server.hasArg("phy_mode")) {
    phy_mode = sanitizePhyMode(static_cast<uint8_t>(server.arg("phy_mode").toInt()));
  }

  if (password.length() == 0 && ssid == config.ssid && config.password[0] != '\0') {
    strlcpy(password_to_save, config.password, sizeof(password_to_save));
  } else {
    strlcpy(password_to_save, password.c_str(), sizeof(password_to_save));
  }

  if (!saveWifiConfig(ssid.c_str(), password_to_save, hostname.c_str(), phy_mode)) {
    server.send(500, F("text/plain"), F("Could not save Wi-Fi settings"));
    return;
  }

  String page;
  page.reserve(800);
  appendHeader(page, F("myMota Wi-Fi"));
  page += F("<p class='ok'>Wi-Fi settings saved. Rebooting.</p>");
  page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
  page += F("<p class='muted'>If Wi-Fi or IP changed, reconnect to the device manually.</p>");
  appendFooter(page, false, true);
  sendHtml(page);
  scheduleRestart(1200);
}

void handleTemplateSave() {
  if (server.hasArg("clear")) {
    clearTemplateConfig();
    if (!commitConfig()) {
      server.send(500, F("text/plain"), F("Could not clear template"));
      return;
    }
    decodeTemplateConfig();
    String page;
    page.reserve(700);
    appendHeader(page, F("myMota Template"));
    page += F("<p class='ok'>Template cleared. Rebooting.</p>");
    page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
    appendFooter(page, false, true);
    sendHtml(page);
    scheduleRestart(1200);
    return;
  }

  const String template_json = server.arg("template");
  if (template_json.length() == 0) {
    server.send(400, F("text/plain"), F("Template JSON is empty"));
    return;
  }

  StoredConfig *candidate = new StoredConfig(config);
  if (!candidate) {
    server.send(500, F("text/plain"), F("Could not allocate template settings"));
    return;
  }

  String error;
  if (!parseTemplateJson(template_json, *candidate, error)) {
    delete candidate;
    String msg = F("Invalid template: ");
    msg += error;
    msg += '\n';
    server.send(400, F("text/plain"), msg);
    return;
  }

  config = *candidate;
  delete candidate;
  if (!commitConfig()) {
    server.send(500, F("text/plain"), F("Could not save template"));
    return;
  }
  decodeTemplateConfig();

  String page;
  page.reserve(800);
  appendHeader(page, F("myMota Template"));
  page += F("<p class='ok'>Template saved. Rebooting.</p>");
  if (runtime_template.unsupported_count) {
    page += F("<p class='bad'>The template contains unsupported GPIO functions. Check the Template section after reboot.</p>");
  }
  page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
  appendFooter(page, false, true);
  sendHtml(page);
  scheduleRestart(1200);
}

void handleMqttSave() {
  String host = server.arg("host");
  String port_arg = server.arg("port");
  String topic = server.arg("topic");
  String keepalive_arg = server.arg("keepalive");
  host.trim();
  port_arg.trim();
  topic.trim();
  keepalive_arg.trim();

  uint16_t port = kMqttDefaultPort;
  uint16_t keepalive = 0;
  if (!isValidMqttHost(host)) {
    server.send(400, F("text/plain"), F("Invalid MQTT host"));
    return;
  }
  if (!parseUint16Input(port_arg, 1, 65535U, port)) {
    server.send(400, F("text/plain"), F("Invalid MQTT port"));
    return;
  }
  if (!isValidMqttTopic(topic)) {
    server.send(400, F("text/plain"), F("Invalid MQTT topic"));
    return;
  }
  if (!parseUint16Input(keepalive_arg, 0, kMqttKeepaliveMax, keepalive)) {
    server.send(400, F("text/plain"), F("Invalid MQTT keepalive"));
    return;
  }

  if (!saveMqttConfig(host.c_str(), port, topic.c_str(), keepalive)) {
    server.send(500, F("text/plain"), F("Could not save MQTT settings"));
    return;
  }

  String page;
  page.reserve(700);
  appendHeader(page, F("myMota MQTT"));
  page += F("<p class='ok'>MQTT settings saved.</p>");
  page += F("<p><a href='/'>Back</a></p>");
  appendFooter(page);
  sendHtml(page);
}

void handleEnergySave() {
  if (!energy.present) {
    server.send(400, F("text/plain"), F("No energy monitor is configured"));
    return;
  }

  String offset_arg = server.arg("total_offset_kwh");
  float total_offset_kwh = 0.0f;
  uint16_t energy_report_interval = config.energy_mqtt_interval;
  uint16_t energy_report_change_percent_x10 = config.energy_mqtt_change_percent_x10;
  if (!parseFloatInput(offset_arg, kEnergyTotalOffsetMinKwh, kEnergyTotalOffsetMaxKwh, total_offset_kwh)) {
    server.send(400, F("text/plain"), F("Invalid total kWh offset"));
    return;
  }
  if (server.hasArg("energy_report_interval") &&
      !parseUint16Input(server.arg("energy_report_interval"), 0, kMqttEnergyIntervalMax, energy_report_interval)) {
    server.send(400, F("text/plain"), F("Invalid energy report interval"));
    return;
  }
  if (server.hasArg("energy_report_change_percent")) {
    float percent = 0.0f;
    if (!parseFloatInput(server.arg("energy_report_change_percent"), 0.0f, kMqttEnergyChangeMaxPercent, percent)) {
      server.send(400, F("text/plain"), F("Invalid energy report change percent"));
      return;
    }
    energy_report_change_percent_x10 = static_cast<uint16_t>((percent * 10.0f) + 0.5f);
  }

  if (!saveEnergyConfig(total_offset_kwh, energy_report_interval, energy_report_change_percent_x10)) {
    server.send(500, F("text/plain"), F("Could not save energy settings"));
    return;
  }

  String page;
  page.reserve(700);
  appendHeader(page, F("myMota Energy"));
  page += F("<p class='ok'>Energy settings saved.</p>");
  page += F("<p><a href='/'>Back</a></p>");
  appendFooter(page);
  sendHtml(page);
}

void handleLedSave() {
  if (!hasConfigurableLedOutputs()) {
    server.send(400, F("text/plain"), F("No configurable LEDs are available"));
    return;
  }

  uint8_t attachments[kMaxLedOutputs];
  memcpy(attachments, config.led_attach, sizeof(attachments));
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    if (!hasLedOutput(i)) continue;

    String arg_name = F("led");
    arg_name += String(i);
    if (!server.hasArg(arg_name)) {
      server.send(400, F("text/plain"), F("Missing LED setting"));
      return;
    }

    uint16_t raw_value = 0;
    if (!parseUint16Input(server.arg(arg_name), 0, 255, raw_value)) {
      server.send(400, F("text/plain"), F("Invalid LED setting"));
      return;
    }

    const uint8_t attachment = static_cast<uint8_t>(raw_value);
    if (!ledAttachmentAvailable(attachment)) {
      server.send(400, F("text/plain"), F("Invalid LED attachment"));
      return;
    }
    attachments[i] = attachment;
  }

  if (!saveLedConfig(attachments)) {
    server.send(500, F("text/plain"), F("Could not save LED settings"));
    return;
  }
  updateDeviceLeds(true);

  String page;
  page.reserve(700);
  appendHeader(page, F("myMota LEDs"));
  page += F("<p class='ok'>LED settings saved.</p>");
  page += F("<p><a href='/'>Back</a></p>");
  appendFooter(page);
  sendHtml(page);
}

bool readButtonEventText(uint8_t button, const char *prefix, bool hold, uint8_t action,
                         char targets[][kButtonActionTargetMaxLen + 1],
                         char payloads[][kButtonActionPayloadMaxLen + 1],
                         String &error) {
  String target_arg = prefix;
  target_arg += F("_target");
  target_arg += String(button);
  String payload_arg = prefix;
  payload_arg += F("_payload");
  payload_arg += String(button);

  String target = server.hasArg(target_arg) ? server.arg(target_arg) : String(targets[button]);
  String payload = server.hasArg(payload_arg) ? server.arg(payload_arg) : String(payloads[button]);
  target.trim();

  if (action == kButtonActionMqtt) {
    if (target.length() == 0) target = kDefaultButtonMqttTopic;
    if (payload.length() == 0) payload = hold ? kDefaultButtonMqttHoldPayload : kDefaultButtonMqttPressPayload;
    if (!isValidMqttPublishTopicTemplate(target)) {
      error = F("Invalid MQTT button topic");
      return false;
    }
    if (!isValidButtonActionText(payload, kButtonActionPayloadMaxLen, false, true)) {
      error = F("Invalid MQTT button payload");
      return false;
    }
  } else if (action == kButtonActionWebhook) {
    if (!isValidWebhookUrlTemplate(target)) {
      error = F("Invalid webhook URL");
      return false;
    }
  } else {
    if (!isValidButtonActionText(target, kButtonActionTargetMaxLen, true)) {
      error = F("Invalid button action target");
      return false;
    }
    if (!isValidButtonActionText(payload, kButtonActionPayloadMaxLen, true, true)) {
      error = F("Invalid button action payload");
      return false;
    }
  }

  strlcpy(targets[button], target.c_str(), kButtonActionTargetMaxLen + 1);
  strlcpy(payloads[button], payload.c_str(), kButtonActionPayloadMaxLen + 1);
  return true;
}

void handleButtonSave() {
  if (!hasConfigurableButtons()) {
    server.send(400, F("text/plain"), F("No configurable buttons are available"));
    return;
  }

  uint16_t hold_ms = kButtonHoldDefaultMs;
  if (!parseUint16Input(server.arg("hold_ms"), kButtonHoldMinMs, kButtonHoldMaxMs, hold_ms)) {
    server.send(400, F("text/plain"), F("Invalid button hold time"));
    return;
  }

  uint16_t debounce_ms = kButtonDebounceDefaultMs;
  if (!parseUint16Input(server.arg("debounce_ms"), kButtonDebounceMinMs, kButtonDebounceMaxMs, debounce_ms)) {
    server.send(400, F("text/plain"), F("Invalid button debounce time"));
    return;
  }

  StoredConfig *candidate = new StoredConfig(config);
  if (!candidate) {
    server.send(500, F("text/plain"), F("Could not allocate button settings"));
    return;
  }

  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;

    String press_arg = F("press");
    press_arg += String(i);
    String hold_arg = F("hold");
    hold_arg += String(i);
    if (!server.hasArg(press_arg) || !server.hasArg(hold_arg)) {
      delete candidate;
      server.send(400, F("text/plain"), F("Missing button setting"));
      return;
    }

    uint16_t press_value = 0;
    uint16_t hold_value = 0;
    if (!parseUint16Input(server.arg(press_arg), 0, 255, press_value) ||
        !parseUint16Input(server.arg(hold_arg), 0, 255, hold_value)) {
      delete candidate;
      server.send(400, F("text/plain"), F("Invalid button setting"));
      return;
    }

    const uint8_t press_action = static_cast<uint8_t>(press_value);
    const uint8_t hold_action = static_cast<uint8_t>(hold_value);
    if (!isButtonActionEncoding(press_action) ||
        !isButtonActionEncoding(hold_action) ||
        !buttonActionAvailable(i, press_action) ||
        !buttonActionAvailable(i, hold_action)) {
      delete candidate;
      server.send(400, F("text/plain"), F("Invalid button action"));
      return;
    }
    candidate->button_press_action[i] = press_action;
    candidate->button_hold_action[i] = hold_action;

    String error;
    if (!readButtonEventText(i, "press", false, press_action, candidate->button_press_target, candidate->button_press_payload, error) ||
        !readButtonEventText(i, "hold", true, hold_action, candidate->button_hold_target, candidate->button_hold_payload, error)) {
      delete candidate;
      server.send(400, F("text/plain"), error);
      return;
    }
  }

  const bool saved = saveButtonConfig(hold_ms, debounce_ms,
                                      candidate->button_press_action,
                                      candidate->button_hold_action,
                                      candidate->button_press_target,
                                      candidate->button_press_payload,
                                      candidate->button_hold_target,
                                      candidate->button_hold_payload);
  delete candidate;
  if (!saved) {
    server.send(500, F("text/plain"), F("Could not save button settings"));
    return;
  }
  updateDeviceLeds(true);

  String page;
  page.reserve(700);
  appendHeader(page, F("myMota Buttons"));
  page += F("<p class='ok'>Button settings saved.</p>");
  page += F("<p><a href='/'>Back</a></p>");
  appendFooter(page);
  sendHtml(page);
}

void handlePowerSave() {
  if (!server.hasArg("relay") || !server.hasArg("state")) {
    server.send(400, F("text/plain"), F("Missing relay or state"));
    return;
  }
  const int relay = server.arg("relay").toInt();
  const String state = server.arg("state");
  if (relay < 1 || relay > kMaxRelays || !hasPin(runtime_template.relays[relay - 1])) {
    server.send(400, F("text/plain"), F("Invalid relay"));
    return;
  }
  if (state == "on") {
    setRelay(relay - 1, true);
  } else if (state == "off") {
    setRelay(relay - 1, false);
  } else if (state == "toggle") {
    toggleRelay(relay - 1);
  } else {
    server.send(400, F("text/plain"), F("Invalid relay state"));
    return;
  }
  updateDeviceLeds(true);
  server.sendHeader(F("Location"), F("/"), true);
  server.send(303, F("text/plain"), "");
}

void handleCmnd() {
  if (!server.hasArg("cmnd")) {
    server.send(400, F("text/plain"), F("Missing cmnd"));
    return;
  }

  const String cmnd_str = server.arg("cmnd");
  const char *raw = cmnd_str.c_str();
  size_t total_len = cmnd_str.length();

  while (total_len > 0) {
    const char c = raw[0];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    raw++;
    total_len--;
  }
  while (total_len > 0) {
    const char c = raw[total_len - 1];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    total_len--;
  }

  size_t cmd_len = 0;
  while (cmd_len < total_len) {
    const char c = raw[cmd_len];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') break;
    cmd_len++;
  }
  size_t state_start = cmd_len;
  while (state_start < total_len) {
    const char c = raw[state_start];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    state_start++;
  }
  if (cmd_len == 0 || state_start >= total_len) {
    server.send(400, F("text/plain"), F("Invalid cmnd"));
    return;
  }
  const size_t state_len = total_len - state_start;

  uint8_t relay = 0;
  char response_key[12];
  if (!parsePowerCommand(raw, cmd_len, relay, response_key, sizeof(response_key))) {
    server.send(400, F("text/plain"), F("Unsupported command"));
    return;
  }
  bool on = false;
  if (!parsePowerState(raw + state_start, state_len, on)) {
    server.send(400, F("text/plain"), F("Invalid power state"));
    return;
  }
  if (relay >= kMaxRelays || !hasPin(runtime_template.relays[relay])) {
    server.send(400, F("text/plain"), F("Invalid relay"));
    return;
  }

  setRelay(relay, on);
  updateDeviceLeds(true);

  String out;
  out.reserve(24);
  out += F("{\"");
  out += response_key;
  out += F("\":\"");
  out += (on ? F("ON") : F("OFF"));
  out += F("\"}");
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), out);
}

void handleReboot() {
  String page;
  page.reserve(700);
  appendHeader(page, F("myMota Reboot"));
  page += F("<p class='ok'>Rebooting.</p>");
  page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
  appendFooter(page, false, true);
  sendHtml(page);
  scheduleRestart(500);
}

void handleFactoryReset() {
  if (!factoryResetConfig()) {
    server.send(500, F("text/plain"), F("Could not factory reset settings"));
    return;
  }
  clearBootRecoveryState();
  mqttStop();

  String page;
  page.reserve(900);
  appendHeader(page, F("myMota Factory Reset"));
  page += F("<p class='ok'>Factory reset complete. Rebooting.</p>");
  page += F("<p>All saved settings have been cleared. After reboot, use the setup AP if the device does not return on this address.</p>");
  appendFooter(page, false, true);
  sendHtml(page);
  scheduleRestart(800);
}

void handleHealth() {
  String out;
  out.reserve(kJsonStreamChunkReserve);
  beginStreamedResponse("application/json");
  out += F("{\"name\":\"myMota\",\"version\":\"");
  out += F(MYMOTA_VERSION);
  out += F("\",\"target\":\"");
  out += F(MYMOTA_TARGET);
  out += F("\",\"chip\":\"");
  out += chipIdHex();
  out += F("\",\"boot_id\":");
  out += boot_id;
  out += F(",\"heap\":");
  out += ESP.getFreeHeap();
  out += F(",\"uptime\":");
  out += millis() / 1000;
  out += F(",\"perf\":{\"loop_hz\":");
  out += perf_last_loop_hz;
  out += F(",\"loop_load\":");
  out += perf_last_loop_load;
  out += F(",\"loop_max_us\":");
  out += perf_last_loop_max_us;
  out += F("}");
  out += F(",\"wifi\":");
  out += ((WiFi.status() == WL_CONNECTED) ? F("true") : F("false"));
  out += F(",\"wifi_ssid\":\"");
  out += (WiFi.status() == WL_CONNECTED ? jsonEscape(WiFi.SSID().c_str()) : String());
  out += F("\",\"ip\":\"");
  out += (WiFi.status() == WL_CONNECTED ? ipToString(WiFi.localIP()) : String());
  out += F("\",\"rssi\":");
  if (WiFi.status() == WL_CONNECTED) {
    out += WiFi.RSSI();
  } else {
    out += F("null");
  }
  out += F(",\"configured_phy_mode\":");
  out += config.phy_mode;
  out += F(",\"configured_phy\":\"");
  out += phyModeName(config.phy_mode);
  out += F("\",\"active_phy_mode\":");
  out += WiFi.getPhyMode();
  out += F(",\"active_phy\":\"");
  out += phyModeName(WiFi.getPhyMode());
  out += F("\",\"recovery\":{\"fast_boot_count\":");
  out += boot_recovery_count;
  out += F(",\"limit\":");
  out += kBootRecoveryLimit;
  out += F(",\"stable_seconds\":");
  out += kBootRecoveryStableMs / 1000;
  out += F(",\"factory_reset\":");
  out += (boot_recovery_factory_reset ? F("true") : F("false"));
  out += F("}");
  flushStreamChunk(out);
  out += F(",\"mqtt\":{\"enabled\":");
  out += (config.mqtt_host[0] ? F("true") : F("false"));
  out += F(",\"connected\":");
  out += (mqtt_client.connected() ? F("true") : F("false"));
  out += F(",\"host\":\"");
  out += jsonEscape(config.mqtt_host);
  out += F("\",\"port\":");
  out += config.mqtt_port;
  out += F(",\"topic\":\"");
  out += jsonEscape(config.mqtt_topic);
  out += F("\",\"keepalive\":");
  out += config.mqtt_keepalive;
  out += F(",\"pending\":");
  out += mqtt_pending_relay_mask;
  out += F(",\"last_connect_result\":\"");
  out += mqttConnectResultName(last_mqtt_connect_result);
  out += F("\",\"last_connect_ms\":");
  out += last_mqtt_connect_duration;
  out += F(",\"last_attempt_ms_ago\":");
  if (last_mqtt_connect_attempt == 0) {
    out += F("null");
  } else {
    out += millis() - last_mqtt_connect_attempt;
  }
  out += F("}");
  flushStreamChunk(out);
  out += F(",\"template\":{\"enabled\":");
  if (runtime_template.enabled) {
    out += F("true");
  } else {
    out += F("false");
  }
  if (runtime_template.enabled) {
    out += F(",\"name\":\"");
    out += jsonEscape(runtime_template.name);
    out += F("\",\"base\":");
    out += runtime_template.base;
    out += F(",\"flag\":");
    out += runtime_template.flag;
    out += F(",\"unsupported\":");
    out += runtime_template.unsupported_count;
  }
  out += F("},\"power\":[");
  bool first = true;
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (!first) out += ',';
    first = false;
    if (!hasPin(runtime_template.relays[i])) {
      out += F("null");
    } else if (relay_state[i]) {
      out += F("true");
    } else {
      out += F("false");
    }
  }
  out += F("]");
  flushStreamChunk(out);
  out += F(",\"button_hold_ms\":");
  out += config.button_hold_ms;
  out += F(",\"button_debounce_ms\":");
  out += config.button_debounce_ms;
  out += F(",\"buttons\":[");
  first = true;
  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (!first) out += ',';
    first = false;
    if (!hasPin(runtime_template.buttons[i])) {
      out += F("null");
    } else {
      out += F("{\"pressed\":");
      out += (button_state[i].stable_pressed ? F("true") : F("false"));
      out += F(",\"press_action\":\"");
      out += buttonActionName(config.button_press_action[i]);
      out += F("\",\"hold_action\":\"");
      out += buttonActionName(config.button_hold_action[i]);
      out += F("\"}");
    }
  }
  out += F("]");
  flushStreamChunk(out);
  out += F(",\"leds\":[");
  first = true;
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    if (!first) out += ',';
    first = false;
    if (!hasLedOutput(i)) {
      out += F("null");
    } else {
      out += F("{\"on\":");
      out += (ledOutputOn(i) ? F("true") : F("false"));
      out += F(",\"attach\":\"");
      out += ledAttachmentName(config.led_attach[i]);
      out += F("\"}");
    }
  }
  out += F("]");
  flushStreamChunk(out);
  out += F(",\"energy\":");
  if (energy.present) {
    out += F("{\"voltage\":");
    out += String(energy.voltage, 1);
    out += F(",\"current\":");
    out += String(energy.current, 3);
    out += F(",\"power\":");
    out += String(energy.power, 1);
    out += F(",\"total_kwh\":");
    out += String(reportedEnergyTotalKwh(), 4);
    out += F(",\"recorded_total_kwh\":");
    out += String(energy.total_kwh, 4);
    out += F(",\"offset_kwh\":");
    out += String(config.energy_total_offset_kwh, 4);
    out += F(",\"report_interval\":");
    out += config.energy_mqtt_interval;
    out += F(",\"report_change_percent\":");
    out += String(energyMqttChangePercent(), 1);
    out += F(",\"debug\":{\"cf_us\":");
    out += energy.cf_power_pulse_length;
    out += F(",\"cf1_last_us\":");
    out += energy.cf1_pulse_length;
    out += F(",\"cf1_voltage_us\":");
    out += energy.cf1_voltage_pulse_length;
    out += F(",\"cf1_current_us\":");
    out += energy.cf1_current_pulse_length;
    out += F(",\"cf1_timer\":");
    out += energy.cf1_timer;
    out += F(",\"selector\":");
    out += (energy.select_ui_flag ? F("true") : F("false"));
    out += F(",\"voltage_flag\":");
    out += (energy.ui_flag ? F("true") : F("false"));
    out += F("}");
    out += F("}");
  } else {
    out += F("null");
  }
  flushStreamChunk(out);
  out += F(",\"temperature_c\":");
  if (runtime_template.adc_temp && !isnan(adc_temperature_c)) {
    out += String(adc_temperature_c, 1);
  } else {
    out += F("null");
  }
  out += F(",\"adc_raw\":");
  if (runtime_template.adc_temp) {
    out += adc_raw;
  } else {
    out += F("null");
  }
  out += F("}");
  endStreamedResponse(out);
}

void handleUpdateDone() {
  if (update_ok && !Update.hasError()) {
    String page;
    page.reserve(700);
    appendHeader(page, F("myMota Update"));
    page += F("<p class='ok'>Firmware uploaded. Rebooting.</p>");
    page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
    appendFooter(page, false, true);
    sendHtml(page);
    scheduleRestart(1200);
    return;
  }

  String page;
  page.reserve(800);
  appendHeader(page, F("myMota Update Failed"));
  page += F("<p class='bad'>Firmware upload failed: ");
  page += updateErrorName(update_error);
  page += F("</p><p><a href='/'>Back</a></p>");
  appendFooter(page);
  sendHtml(page);
}

void handleUpdateUpload() {
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START) {
    update_started = false;
    update_ok = false;
    update_mqtt_paused = true;
    update_error = UPDATE_ERROR_OK;
    update_max_size = 0;
    persistEnergyTotal(true);
    mqttStop();
    WiFiUDP::stopAll();
    if (upload.filename.length() == 0) {
      update_error = UPDATE_ERROR_SIZE;
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_WRITE && update_error != UPDATE_ERROR_OK) {
    return;
  }

  if (upload.status == UPLOAD_FILE_WRITE) {
    if (!update_started && upload.totalSize == 0) {
      if (upload.currentSize < 4) {
        update_error = UPDATE_ERROR_SIZE;
        return;
      }

      if ((upload.buf[0] != 0xE9) && (upload.buf[0] != 0x1F)) {
        update_error = UPDATE_ERROR_MAGIC_BYTE;
        return;
      }

      if (upload.buf[0] == 0xE9) {
        const uint32_t bin_flash_size = ESP.magicFlashChipSize((upload.buf[3] & 0xF0) >> 4);
        if (bin_flash_size > ESP.getFlashChipRealSize()) {
          update_error = UPDATE_ERROR_NEW_FLASH_CONFIG;
          return;
        }
      }

      const uint32_t free_sketch_space = ESP.getFreeSketchSpace();
      if (free_sketch_space <= 0x1000) {
        update_error = UPDATE_ERROR_SPACE;
        return;
      }
      const uint32_t max_sketch_space = (free_sketch_space - 0x1000) & 0xFFFFF000;
      update_max_size = max_sketch_space;
      if (!Update.begin(max_sketch_space)) {
        update_error = Update.getError();
        return;
      }
      update_started = true;
    }

    if (update_max_size == 0 ||
        upload.totalSize > update_max_size ||
        upload.currentSize > update_max_size - upload.totalSize) {
      if (update_started) {
        Update.end();
        update_started = false;
      }
      update_error = UPDATE_ERROR_SPACE;
      return;
    }

    if (Update.hasError()) {
      update_error = Update.getError();
      return;
    }

    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      update_error = Update.getError();
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_END) {
    if (update_error != UPDATE_ERROR_OK) {
      if (update_started) {
        Update.end();
        update_started = false;
      }
      update_mqtt_paused = false;
    } else if (!update_started) {
      update_error = UPDATE_ERROR_SIZE;
      update_mqtt_paused = false;
    } else if (Update.end(true)) {
      update_ok = true;
      update_started = false;
      update_mqtt_paused = true;
    } else {
      update_error = Update.getError();
      update_started = false;
      update_mqtt_paused = false;
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_ABORTED) {
    if (update_started) {
      Update.end();
    }
    update_started = false;
    update_ok = false;
    update_mqtt_paused = false;
    update_max_size = 0;
    update_error = UPDATE_ERROR_STREAM;
  }
}

void handleNotFound() {
  server.sendHeader(F("Location"), F("/"), true);
  server.send(302, F("text/plain"), "");
}

void startAp() {
  if (ap_started) return;
  const uint32_t now = millis();
  if (last_ap_attempt && (now - last_ap_attempt < kApRetryMs)) return;
  last_ap_attempt = now;
  const String ap_name = defaultHostname();
  WiFi.mode(WIFI_AP_STA);
  ap_started = WiFi.softAP(ap_name.c_str());
}

void applyPhyMode(uint8_t phy_mode) {
  phy_mode = sanitizePhyMode(phy_mode);
  if (phy_mode != kPhyModeAuto) {
    WiFi.setPhyMode(static_cast<WiFiPhyMode_t>(phy_mode));
  }
}

bool waitForWifi(uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeout_ms) {
    delay(100);
    ESP.wdtFeed();
  }
  return WiFi.status() == WL_CONNECTED;
}

bool connectWifiWithPhy(uint8_t phy_mode, uint32_t timeout_ms) {
  WiFi.disconnect(false);
  delay(100);
  WiFi.mode(WIFI_STA);
  applyPhyMode(phy_mode);
  WiFi.begin(config.ssid, config.password);
  return waitForWifi(timeout_ms);
}

void prepareWifi() {
  WiFi.persistent(false);
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(true);
  WiFi.hostname(config.hostname);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  wifi_set_sleep_type(NONE_SLEEP_T);
}

void connectWifi() {
  prepareWifi();

  if (!config_ok) {
    WiFi.mode(WIFI_AP);
    startAp();
    return;
  }

  if (connectWifiWithPhy(config.phy_mode, kConnectTimeoutMs)) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (config.phy_mode != kPhyModeAuto && config.phy_mode != kPhyModeFailsafe) {
      if (connectWifiWithPhy(kPhyModeFailsafe, kConnectTimeoutMs)) {
        return;
      }
    }
    startAp();
    disconnected_since = millis();
    disconnected_timer_active = true;
  }
}

void setupRoutes() {
  server.on(F("/"), HTTP_GET, handleRoot);
  server.on(F("/scan"), HTTP_GET, handleScan);
  server.on(F("/wifi"), HTTP_POST, handleWifiSave);
  server.on(F("/template"), HTTP_POST, handleTemplateSave);
  server.on(F("/mqtt"), HTTP_POST, handleMqttSave);
  server.on(F("/energy"), HTTP_POST, handleEnergySave);
  server.on(F("/leds"), HTTP_POST, handleLedSave);
  server.on(F("/buttons"), HTTP_POST, handleButtonSave);
  server.on(F("/power"), HTTP_POST, handlePowerSave);
  server.on(F("/cm"), HTTP_GET, handleCmnd);
  server.on(F("/reboot"), HTTP_GET, handleReboot);
  server.on(F("/factory-reset"), HTTP_POST, handleFactoryReset);
  server.on(F("/health"), HTTP_GET, handleHealth);
  server.on(F("/update"), HTTP_POST, handleUpdateDone, handleUpdateUpload);
  server.onNotFound(handleNotFound);
}

void maintainWifi() {
  if (!config_ok) {
    startAp();
    return;
  }
  if (WiFi.status() == WL_CONNECTED) {
    disconnected_since = 0;
    disconnected_timer_active = false;
    return;
  }
  if (!disconnected_timer_active) {
    disconnected_since = millis();
    disconnected_timer_active = true;
  }
  if (!ap_started && millis() - disconnected_since > kReconnectStartApMs) {
    startAp();
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(20);
  Serial.println();
  Serial.printf("myMota %s %s chip %06X\n", MYMOTA_VERSION, MYMOTA_TARGET, ESP.getChipId());

  loadConfig();
  loadEnergyJournal();
  decodeTemplateConfig();
  setupDevicePins();
  connectWifi();
  boot_id = makeBootId();
  setupRoutes();
  server.begin();

  Serial.printf("HTTP server started; STA %s AP %s\n",
                WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "not-connected",
                ap_started ? WiFi.softAPIP().toString().c_str() : "off");
}

void loop() {
  const uint32_t loop_started_us = micros();

  server.handleClient();
  maintainBootRecovery();
  maintainWifi();
  maintainDevice();
  server.handleClient();
  maintainMqtt();

  if (restartDue()) {
    persistEnergyTotal(true);
    delay(50);
    ESP.restart();
  }

  recordLoopPerf(loop_started_us, micros());
  yield();
}
