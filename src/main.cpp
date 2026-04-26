#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <Updater.h>
#include <WiFiUdp.h>
#include <math.h>

extern "C" {
#include <user_interface.h>
}

#ifndef MYMOTA_VERSION
#define MYMOTA_VERSION "dev"
#endif

#ifndef MYMOTA_TARGET
#define MYMOTA_TARGET "esp8266"
#endif

namespace {

constexpr uint32_t kConfigMagic = 0x4d594d4f;  // MYMO
constexpr uint32_t kBootRecoveryMagic = 0x4d595246;  // MYRF
constexpr uint16_t kConfigVersionV1 = 1;
constexpr uint16_t kConfigVersionV2 = 2;
constexpr uint16_t kConfigVersionV3 = 3;
constexpr uint16_t kConfigVersionV4 = 4;
constexpr uint16_t kConfigVersion = 5;
constexpr size_t kEepromSize = 512;
constexpr size_t kBootRecoveryOffset = 480;
constexpr uint32_t kConnectTimeoutMs = 20000;
constexpr uint32_t kReconnectStartApMs = 90000;
constexpr uint32_t kApRetryMs = 10000;
constexpr uint32_t kBootRecoveryStableMs = 30000;
constexpr const char *kApPassword = "mymota-setup";
constexpr uint8_t kPhyModeAuto = 0;
constexpr uint8_t kPhyModeFailsafe = WIFI_PHY_MODE_11G;
constexpr uint8_t kBootRecoveryLimit = 5;
constexpr size_t kTemplateSlotCount = 14;
constexpr size_t kTemplateJsonMaxLen = 640;
constexpr size_t kMqttHostMaxLen = 64;
constexpr size_t kMqttTopicMaxLen = 150;
constexpr uint16_t kMqttDefaultPort = 1883;
constexpr uint16_t kMqttKeepaliveMax = 65535U;
constexpr uint16_t kMqttProtocolKeepaliveSec = 30;
constexpr uint32_t kMqttReconnectMs = 5000;
constexpr uint32_t kMqttReadTimeoutMs = 1000;
constexpr uint8_t kInvalidPin = 0xff;
constexpr uint8_t kAdc0Pin = 17;
constexpr uint8_t kMaxRelays = 8;
constexpr uint8_t kMaxButtons = 4;
constexpr uint8_t kMaxLeds = 4;
constexpr uint32_t kButtonDebounceMs = 50;
constexpr uint32_t kLedUpdateMs = 250;
constexpr uint32_t kAdcUpdateMs = 2000;
constexpr uint32_t kEnergyUpdateMs = 200;
constexpr uint32_t kEnergyIntegrateMs = 1000;
constexpr float kEnergyTotalOffsetMinKwh = 0.0f;
constexpr float kEnergyTotalOffsetMaxKwh = 1000000.0f;

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
  uint8_t reserved[14];
  uint32_t crc;
};

struct BootRecoveryState {
  uint32_t magic;
  uint8_t boot_count;
  uint8_t reserved[3];
  uint32_t crc;
};

static_assert(sizeof(StoredConfig) <= kEepromSize, "StoredConfig exceeds EEPROM size");
static_assert(sizeof(StoredConfig) <= kBootRecoveryOffset, "StoredConfig overlaps boot recovery state");
static_assert(kBootRecoveryOffset + sizeof(BootRecoveryState) <= kEepromSize, "BootRecoveryState exceeds EEPROM size");
static_assert(sizeof(kTemplateSlotToPin) == kTemplateSlotCount, "Template pin map size mismatch");

struct PinAssignment {
  uint8_t pin;
  bool inverted;
  bool no_pullup;
};

struct ButtonState {
  bool raw_pressed;
  bool stable_pressed;
  bool emitted_pressed;
  uint32_t changed_at;
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
  bool sel_inverted;
  bool select_voltage;
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
uint8_t update_error = UPDATE_ERROR_OK;
uint32_t restart_at = 0;
uint32_t disconnected_since = 0;
uint32_t last_ap_attempt = 0;
uint32_t last_led_update = 0;
uint32_t last_adc_update = 0;
uint32_t next_mqtt_reconnect = 0;
uint32_t last_mqtt_io = 0;
uint32_t last_mqtt_state_publish = 0;
uint16_t mqtt_pending_relay_mask = 0;
uint8_t boot_recovery_count = 0;
bool boot_recovery_armed = false;
bool boot_recovery_factory_reset = false;
bool relay_state[kMaxRelays]{};
float adc_temperature_c = NAN;
uint16_t adc_raw = 0;

uint32_t fnv1a(const uint8_t *data, size_t len) {
  uint32_t hash = 2166136261UL;
  for (size_t i = 0; i < len; i++) {
    hash ^= data[i];
    hash *= 16777619UL;
  }
  return hash;
}

template <typename ConfigT>
uint32_t configCrc(const ConfigT &cfg) {
  ConfigT tmp = cfg;
  tmp.crc = 0;
  return fnv1a(reinterpret_cast<const uint8_t *>(&tmp), sizeof(tmp));
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

void setDefaultConfig() {
  memset(&config, 0, sizeof(config));
  config.magic = kConfigMagic;
  config.version = kConfigVersion;
  config.size = sizeof(StoredConfig);
  config.phy_mode = kPhyModeAuto;
  strlcpy(config.hostname, defaultHostname().c_str(), sizeof(config.hostname));
  setDefaultMqttConfig();
  config.crc = configCrc(config);
  config_ok = false;
}

bool commitConfig();

bool loadBootRecoveryState(BootRecoveryState &state) {
  EEPROM.get(kBootRecoveryOffset, state);
  return state.magic == kBootRecoveryMagic && state.crc == configCrc(state);
}

bool saveBootRecoveryState(uint8_t boot_count) {
  BootRecoveryState state{};
  state.magic = kBootRecoveryMagic;
  state.boot_count = boot_count;
  state.crc = configCrc(state);
  EEPROM.put(kBootRecoveryOffset, state);
  const bool committed = EEPROM.commit();
  if (committed) {
    boot_recovery_count = boot_count;
    boot_recovery_armed = boot_count > 0;
  }
  return committed;
}

bool clearBootRecoveryState() {
  return saveBootRecoveryState(0);
}

bool recordBootRecoveryStart() {
  BootRecoveryState state{};
  uint8_t boot_count = 0;
  if (loadBootRecoveryState(state)) {
    boot_count = state.boot_count;
  }
  if (boot_count < 255) {
    boot_count++;
  }

  if (boot_count >= kBootRecoveryLimit) {
    boot_recovery_factory_reset = true;
    clearBootRecoveryState();
    return true;
  }

  saveBootRecoveryState(boot_count);
  return false;
}

bool factoryResetConfig() {
  setDefaultConfig();
  return commitConfig();
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
}

void normalizeConfigStrings() {
  config.ssid[sizeof(config.ssid) - 1] = '\0';
  config.password[sizeof(config.password) - 1] = '\0';
  config.hostname[sizeof(config.hostname) - 1] = '\0';
  config.template_name[sizeof(config.template_name) - 1] = '\0';
  config.mqtt_host[sizeof(config.mqtt_host) - 1] = '\0';
  config.mqtt_topic[sizeof(config.mqtt_topic) - 1] = '\0';
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

bool commitConfig() {
  config.magic = kConfigMagic;
  config.version = kConfigVersion;
  config.size = sizeof(StoredConfig);
  normalizeConfigStrings();
  config.crc = configCrc(config);
  EEPROM.put(0, config);
  const bool committed = EEPROM.commit();
  config_ok = committed && config.ssid[0] != '\0';
  return committed;
}

bool saveWifiConfig(const char *ssid, const char *password, const char *hostname, uint8_t phy_mode);
bool saveMqttConfig(const char *host, uint16_t port, const char *topic, uint16_t keepalive);
bool saveEnergyConfig(float total_offset_kwh);

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
  last_mqtt_state_publish = 0;
  mqtt_pending_relay_mask = 0;
  return commitConfig();
}

bool saveEnergyConfig(float total_offset_kwh) {
  config.energy_total_offset_kwh = total_offset_kwh;
  return commitConfig();
}

float reportedEnergyTotalKwh() {
  return energy.total_kwh + config.energy_total_offset_kwh;
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

void decodeTemplateConfig() {
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

const char *skipJsonSpaces(const char *p) {
  while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') p++;
  return p;
}

const char *findJsonValue(const String &json, const char *key) {
  String needle = F("\"");
  needle += key;
  needle += F("\"");
  const int pos = json.indexOf(needle);
  if (pos < 0) return nullptr;
  const char *p = json.c_str() + pos + needle.length();
  p = skipJsonSpaces(p);
  if (*p != ':') return nullptr;
  return skipJsonSpaces(p + 1);
}

bool parseJsonStringValue(const String &json, const char *key, char *out, size_t out_size, bool required, String &error) {
  const char *p = findJsonValue(json, key);
  if (!p) {
    if (required) {
      error = F("Missing ");
      error += key;
    }
    return !required;
  }
  if (*p != '"') {
    error = F("Invalid string for ");
    error += key;
    return false;
  }
  p++;
  size_t written = 0;
  while (*p && *p != '"') {
    char c = *p++;
    if (c == '\\' && *p) {
      c = *p++;
    }
    if (written + 1 < out_size) {
      out[written++] = c;
    }
  }
  if (*p != '"') {
    error = F("Unterminated string for ");
    error += key;
    return false;
  }
  out[written] = '\0';
  return true;
}

bool parseJsonUIntAt(const char *&p, uint32_t &value) {
  p = skipJsonSpaces(p);
  if (*p < '0' || *p > '9') return false;
  uint32_t parsed = 0;
  while (*p >= '0' && *p <= '9') {
    parsed = (parsed * 10U) + static_cast<uint32_t>(*p - '0');
    p++;
  }
  value = parsed;
  return true;
}

bool parseJsonUIntValue(const String &json, const char *key, uint32_t &value, bool required, String &error) {
  const char *p = findJsonValue(json, key);
  if (!p) {
    if (required) {
      error = F("Missing ");
      error += key;
    }
    return !required;
  }
  if (!parseJsonUIntAt(p, value)) {
    error = F("Invalid number for ");
    error += key;
    return false;
  }
  return true;
}

bool parseTemplateGpioArray(const String &json, uint16_t *gpio, String &error) {
  const char *p = findJsonValue(json, "GPIO");
  if (!p) {
    error = F("Missing GPIO");
    return false;
  }
  if (*p != '[') {
    error = F("Invalid GPIO array");
    return false;
  }
  p++;
  for (uint8_t i = 0; i < kTemplateSlotCount; i++) {
    uint32_t value = 0;
    if (!parseJsonUIntAt(p, value) || value > 65535U) {
      error = F("Invalid GPIO value");
      return false;
    }
    gpio[i] = static_cast<uint16_t>(value);
    p = skipJsonSpaces(p);
    if (i + 1 < kTemplateSlotCount) {
      if (*p != ',') {
        error = F("GPIO must contain 14 ESP8266 entries");
        return false;
      }
      p++;
    }
  }
  p = skipJsonSpaces(p);
  if (*p != ']') {
    error = F("GPIO must contain exactly 14 ESP8266 entries");
    return false;
  }
  return true;
}

bool parseTemplateJson(const String &json, StoredConfig &target, String &error) {
  if (json.length() < 9 || json.length() > kTemplateJsonMaxLen) {
    error = F("Template JSON length is invalid");
    return false;
  }
  if (json.indexOf('{') < 0 || json.indexOf('}') < 0) {
    error = F("Template must be JSON");
    return false;
  }

  char arch[16] = "";
  if (!parseJsonStringValue(json, "ARCH", arch, sizeof(arch), false, error)) return false;
  if (arch[0] && strcmp(arch, "ESP8266") && strcmp(arch, "ESP8285") && strcmp(arch, "ESP82XX")) {
    error = F("Template ARCH is not ESP8266/ESP8285");
    return false;
  }

  char name[sizeof(target.template_name)] = "";
  uint16_t gpio[kTemplateSlotCount]{};
  uint32_t base = 0;
  uint32_t flag = 0;
  if (!parseJsonStringValue(json, "NAME", name, sizeof(name), true, error)) return false;
  if (name[0] == '\0') {
    error = F("Template NAME is empty");
    return false;
  }
  if (!parseTemplateGpioArray(json, gpio, error)) return false;
  if (!parseJsonUIntValue(json, "BASE", base, true, error)) return false;
  if (!parseJsonUIntValue(json, "FLAG", flag, false, error)) return false;
  if (base == 0 || base > 255) {
    error = F("Template BASE is invalid");
    return false;
  }

  target.template_enabled = 1;
  target.template_base = static_cast<uint16_t>(base);
  target.template_flag = flag;
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

bool parsePowerCommand(const String &input, uint8_t &relay, String &response_key) {
  String cmd = input;
  cmd.trim();
  cmd.toLowerCase();
  if (cmd == F("power")) {
    relay = 0;
    response_key = F("POWER");
    return true;
  }
  if (!cmd.startsWith(F("power")) || cmd.length() == 5) {
    return false;
  }

  uint16_t relay_number = 0;
  for (size_t i = 5; i < cmd.length(); i++) {
    const char c = cmd[i];
    if (c < '0' || c > '9') return false;
    relay_number = (relay_number * 10U) + static_cast<uint16_t>(c - '0');
    if (relay_number > kMaxRelays) return false;
  }
  if (relay_number == 0) return false;
  relay = static_cast<uint8_t>(relay_number - 1);
  response_key = F("POWER");
  response_key += String(relay_number);
  return true;
}

bool parsePowerState(const String &input, bool &on) {
  String state = input;
  state.trim();
  state.toLowerCase();
  if (state == F("on")) {
    on = true;
    return true;
  }
  if (state == F("off")) {
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

bool mqttReadByte(uint8_t &value, uint32_t timeout_ms = kMqttReadTimeoutMs) {
  const uint32_t start = millis();
  while (!mqtt_client.available()) {
    if (!mqtt_client.connected() || millis() - start >= timeout_ms) {
      return false;
    }
    delay(1);
  }
  const int read_value = mqtt_client.read();
  if (read_value < 0) return false;
  value = static_cast<uint8_t>(read_value);
  return true;
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
}

bool mqttConnect() {
  if (!mqttConfigured() || WiFi.status() != WL_CONNECTED) return false;

  mqttStop();
  mqtt_client.setTimeout(kMqttReadTimeoutMs);
  if (!mqtt_client.connect(config.mqtt_host, config.mqtt_port)) {
    return false;
  }

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
    return false;
  }
  last_mqtt_io = millis();

  uint8_t packet_type = 0;
  uint8_t remaining = 0;
  uint8_t flags = 0;
  uint8_t return_code = 0;
  ok = mqttReadByte(packet_type) && mqttReadByte(remaining) && mqttReadByte(flags) && mqttReadByte(return_code);
  if (!ok || packet_type != 0x20 || remaining != 0x02 || return_code != 0x00) {
    mqttStop();
    return false;
  }
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

void maintainMqtt() {
  if (!mqttConfigured() || WiFi.status() != WL_CONNECTED) {
    mqttStop();
    return;
  }

  if (!mqttEnsureConnected()) return;

  while (mqtt_client.available()) {
    mqtt_client.read();
  }

  const uint32_t now = millis();
  if (now - last_mqtt_io >= (static_cast<uint32_t>(kMqttProtocolKeepaliveSec) * 1000UL)) {
    if (mqttWriteByte(0xc0) && mqttWriteByte(0x00)) {
      last_mqtt_io = now;
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

  if (config.mqtt_keepalive > 0 && runtime_template.relay_count > 0) {
    const uint32_t interval_ms = static_cast<uint32_t>(config.mqtt_keepalive) * 1000UL;
    if (now - last_mqtt_state_publish >= interval_ms) {
      mqttPublishAllRelayStates();
    }
  }
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

void selectEnergyCf1Mode(bool voltage) {
  energy.select_voltage = voltage;
  if (!energy.present || !digitalPinSupported(energy.sel_pin)) return;
  const bool level = voltage ? !energy.sel_inverted : energy.sel_inverted;
  digitalWrite(energy.sel_pin, level ? HIGH : LOW);
}

void setupEnergyMonitor() {
  memset(&energy, 0, sizeof(energy));
  energy.cf_pin = runtime_template.energy_cf_pin;
  energy.cf1_pin = runtime_template.energy_cf1_pin;
  energy.sel_pin = runtime_template.energy_sel_pin;
  energy.sel_inverted = runtime_template.energy_sel_inverted;
  energy.hjl = runtime_template.energy_hjl;
  energy.present = interruptPinSupported(energy.cf_pin);
  energy.load_off = true;
  energy.power_ratio = energy.hjl ? kHjlPowerRatio : kHlwPowerRatio;
  energy.voltage_ratio = energy.hjl ? kHjlVoltageRatio : kHlwVoltageRatio;
  energy.current_ratio = energy.hjl ? kHjlCurrentRatio : kHlwCurrentRatio;
  energy.select_voltage = true;
  energy.last_update_ms = millis();
  energy.last_integrate_ms = millis();

  if (!energy.present) return;

  if (digitalPinSupported(energy.sel_pin)) {
    selectEnergyCf1Mode(true);
    pinMode(energy.sel_pin, OUTPUT);
    selectEnergyCf1Mode(true);
  }
  if (interruptPinSupported(energy.cf1_pin)) {
    pinMode(energy.cf1_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(energy.cf1_pin), energyCf1Interrupt, FALLING);
  }
  pinMode(energy.cf_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(energy.cf_pin), energyCfInterrupt, FALLING);
}

void setRelay(uint8_t relay, bool on) {
  if (relay >= kMaxRelays || !hasPin(runtime_template.relays[relay])) return;
  const bool changed = relay_state[relay] != on;
  relay_state[relay] = on;
  writeAssignedPin(runtime_template.relays[relay], on);
  if (changed) {
    scheduleMqttRelayPublish(relay);
  }
}

void toggleRelay(uint8_t relay) {
  if (relay >= kMaxRelays) return;
  setRelay(relay, !relay_state[relay]);
}

void updateDeviceLeds(bool force = false) {
  const uint32_t now = millis();
  if (!force && now - last_led_update < kLedUpdateMs) return;
  last_led_update = now;

  for (uint8_t i = 0; i < runtime_template.led_count; i++) {
    if (!hasPin(runtime_template.leds[i])) continue;
    const bool on = i < runtime_template.relay_count ? relay_state[i] : false;
    writeAssignedPin(runtime_template.leds[i], on);
  }

  if (hasPin(runtime_template.link_led)) {
    bool on = WiFi.status() == WL_CONNECTED;
    if (!on && ap_started) {
      on = ((now / 500U) & 1U) == 0;
    }
    writeAssignedPin(runtime_template.link_led, on);
  }
}

float readAdcTemperatureC(uint16_t raw) {
  if (raw == 0 || raw >= 1023) return NAN;
  const float resistance = kAnalogNtcBridgeResistance * static_cast<float>(raw) / static_cast<float>(1023U - raw);
  if (resistance <= 0.0f) return NAN;
  const float kelvin = 1.0f / ((1.0f / kAnalogT0Kelvin) + (log(resistance / kAnalogNtcResistance) / kAnalogNtcBeta));
  return kelvin - 273.15f;
}

void maintainAdc() {
  if (!runtime_template.adc_temp) return;
  const uint32_t now = millis();
  if (now - last_adc_update < kAdcUpdateMs) return;
  last_adc_update = now;
  adc_raw = analogRead(A0);
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
    if ((now - button_state[i].changed_at) >= kButtonDebounceMs && raw != button_state[i].stable_pressed) {
      button_state[i].stable_pressed = raw;
      if (raw && !button_state[i].emitted_pressed) {
        const uint8_t relay = (i < runtime_template.relay_count) ? i : 0;
        toggleRelay(relay);
        updateDeviceLeds(true);
      }
      button_state[i].emitted_pressed = raw;
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
        const uint32_t cf1_avg = cf1_count ? (cf1_sum / cf1_count) : 0;
        if (energy.select_voltage) {
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
        selectEnergyCf1Mode(!energy.select_voltage);
      }
    }
  }

  if (now - energy.last_integrate_ms >= kEnergyIntegrateMs) {
    const uint32_t elapsed = now - energy.last_integrate_ms;
    energy.last_integrate_ms = now;
    energy.total_kwh += (energy.power * static_cast<float>(elapsed)) / 3600000000.0f;
  }
}

void setupDevicePins() {
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    relay_state[i] = false;
    if (!hasPin(runtime_template.relays[i])) continue;
    writeAssignedPin(runtime_template.relays[i], false);
    pinMode(runtime_template.relays[i].pin, OUTPUT);
    writeAssignedPin(runtime_template.relays[i], false);
  }

  for (uint8_t i = 0; i < kMaxLeds; i++) {
    if (!hasPin(runtime_template.leds[i])) continue;
    writeAssignedPin(runtime_template.leds[i], false);
    pinMode(runtime_template.leds[i].pin, OUTPUT);
    writeAssignedPin(runtime_template.leds[i], false);
  }
  if (hasPin(runtime_template.link_led)) {
    writeAssignedPin(runtime_template.link_led, false);
    pinMode(runtime_template.link_led.pin, OUTPUT);
    writeAssignedPin(runtime_template.link_led, false);
  }

  for (uint8_t i = 0; i < kMaxButtons; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;
    pinMode(runtime_template.buttons[i].pin, runtime_template.buttons[i].no_pullup ? INPUT : INPUT_PULLUP);
    const bool pressed = readAssignedPressed(runtime_template.buttons[i]);
    button_state[i] = {pressed, pressed, pressed, millis()};
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

void appendHeader(String &page, const __FlashStringHelper *title) {
  page += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>");
  page += title;
  page += F("</title><style>:root{--bg:#f6f7f9;--panel:#fff;--line:#d8dee8;--text:#17202a;--muted:#687386;--ok:#177245;--bad:#a23a36;--accent:#1f7a5f;--accent2:#205c8a}");
  page += F("*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--text);font-family:Arial,sans-serif;font-size:15px;line-height:1.4}");
  page += F(".top{background:#17202a;color:#fff;border-bottom:4px solid var(--accent);padding:18px 16px}.topin{max-width:1080px;margin:0 auto;display:flex;align-items:end;justify-content:space-between;gap:12px;flex-wrap:wrap}");
  page += F(".brand{font-size:28px;font-weight:700;letter-spacing:0}.brand span{color:#7dd3aa}.sub{color:#c7d0dc;font-size:13px}main{max-width:1080px;margin:18px auto 28px;padding:0 14px}");
  page += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px;box-shadow:0 1px 2px rgba(0,0,0,.04)}.wide{grid-column:1/-1}");
  page += F(".panel h2{font-size:17px;margin:0 0 12px}.kv{display:grid;grid-template-columns:minmax(110px,42%) 1fr;gap:8px 12px}.kv span,.hint{color:var(--muted)}.kv div{min-width:0}");
  page += F("code{background:#eef2f6;border:1px solid #dce3ea;border-radius:4px;padding:1px 4px;word-break:break-word}.pill{display:inline-block;border-radius:999px;padding:2px 8px;background:#eef2f6;color:#364152}.ok{color:var(--ok)}.bad{color:var(--bad)}.muted{color:var(--muted)}");
  page += F("form{margin:0}.row{margin:10px 0}label{display:block;font-weight:600;color:#344054}input,button,select,textarea{font:inherit}input,select,textarea{width:100%;margin-top:4px;padding:9px;border:1px solid #b9c4d0;border-radius:6px;background:#fff}textarea{min-height:92px;resize:vertical}");
  page += F("button,.btn{display:inline-block;margin:4px 4px 0 0;padding:8px 12px;border:1px solid var(--accent);border-radius:6px;background:var(--accent);color:#fff;text-decoration:none;cursor:pointer}.secondary{background:#fff;color:var(--accent2);border-color:#9eb7cf}.danger{background:#fff;color:var(--bad);border-color:#d4aaa7}.inline{display:inline}.actions{display:flex;flex-wrap:wrap;gap:6px}.inline button{margin:0 4px 0 0}.list{margin:0;padding-left:18px}@media(max-width:520px){.kv{grid-template-columns:1fr}.brand{font-size:24px}}</style></head><body>");
  page += F("<header class='top'><div class='topin'><div><div class='brand'>my<span>Mota</span></div><div class='sub'>ESP8266/ESP8285 firmware</div></div><div class='sub'>");
  page += F(MYMOTA_VERSION);
  page += F(" / ");
  page += F(MYMOTA_TARGET);
  page += F("</div></div></header><main>");
}

void appendFooter(String &page) {
  page += F("<script>function t(i,v){var e=document.getElementById(i);if(e)e.textContent=v;}");
  page += F("function p(i,v,c){var e=document.getElementById(i);if(e){e.textContent=v;e.className=c;}}");
  page += F("function fmt(v,d,u){return v==null?'n/a':Number(v).toFixed(d)+(u||'');}");
  page += F("function live(){fetch('/health',{cache:'no-store'}).then(function(r){return r.json();}).then(function(d){");
  page += F("t('live-heap',d.heap+' bytes');t('live-uptime',d.uptime+'s');t('live-active-phy',d.active_phy);");
  page += F("t('live-recovery',d.recovery.fast_boot_count+'/'+d.recovery.limit);");
  page += F("p('live-wifi',d.wifi?'connected':'not connected',d.wifi?'pill ok':'pill bad');t('live-ssid',d.wifi_ssid||'n/a');t('live-ip',d.ip||'n/a');t('live-rssi',d.rssi==null?'n/a':d.rssi+' dBm');");
  page += F("p('live-mqtt',d.mqtt.enabled?(d.mqtt.connected?'connected':'not connected'):'not configured',d.mqtt.enabled&&d.mqtt.connected?'pill ok':'pill');");
  page += F("if(d.power){for(var i=0;i<d.power.length;i++){if(d.power[i]!==null)p('live-relay-'+i,d.power[i]?'on':'off',d.power[i]?'pill ok':'pill');}}");
  page += F("if(d.energy){t('live-energy-power',fmt(d.energy.power,1,' W'));t('live-energy-voltage',fmt(d.energy.voltage,1,' V'));t('live-energy-current',fmt(d.energy.current,3,' A'));t('live-energy-total',fmt(d.energy.total_kwh,4,' kWh'));t('live-energy-offset',fmt(d.energy.offset_kwh,4,' kWh'));}");
  page += F("t('live-temp',d.temperature_c==null?'n/a':Number(d.temperature_c).toFixed(1)+' C');t('live-adc-raw',d.adc_raw==null?'n/a':d.adc_raw);");
  page += F("}).catch(function(){});}setInterval(live,1000);live();</script></main></body></html>");
}

void sendHtml(String &page) {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("text/html"), page);
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
  page += F("s</code></div><span>PHY mode</span><div><code>");
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
    page += F("<span>Wi-Fi</span><div><span id='live-wifi' class='pill bad'>not connected</span> <code id='live-ssid'>n/a</code></div>");
    page += F("<span>IP</span><div><code id='live-ip'>n/a</code></div><span>RSSI</span><div><code id='live-rssi'>n/a</code></div>");
  }

  if (ap_started) {
    page += F("<span>Setup AP</span><div><code>");
    page += htmlEscape(WiFi.softAPSSID());
    page += F("</code> <code>");
    page += kApPassword;
    page += F("</code> at <code>");
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
      page += F("' class='pill'>off</span>");
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
    page += F("'></label></div><button type='submit'>Save energy</button></form>");
  }
  page += F("</section>");
}

void appendMqttStatus(String &page) {
  page += F("<section class='panel'><h2>MQTT</h2><div class='kv'>");
  if (config.mqtt_host[0] == '\0') {
    page += F("<span>Broker</span><div><span id='live-mqtt' class='pill'>not configured</span></div>");
  } else {
    page += F("<span>Broker</span><div><code>");
    page += htmlEscape(config.mqtt_host);
    page += F(":");
    page += String(config.mqtt_port);
    page += F("</code> ");
    if (mqtt_client.connected()) {
      page += F("<span id='live-mqtt' class='pill ok'>connected</span>");
    } else {
      page += F("<span id='live-mqtt' class='pill'>not connected</span>");
    }
    page += F("</div>");
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
  page += F("<div class='row'><label>Tasmota ESP8266 template JSON<br><textarea name='template' rows='5' maxlength='");
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
  page.reserve(9800);
  appendHeader(page, F("myMota"));
  page += F("<div class='grid'>");
  appendStatusBlock(page);
  appendTemplateStatus(page);
  appendDeviceControls(page);
  appendMqttStatus(page);
  page += F("<section class='panel'><h2>Wi-Fi</h2><form method='post' action='/wifi'>");
  page += F("<div class='row'><label>SSID<br><input name='ssid' maxlength='32' required value='");
  page += htmlEscape(config.ssid);
  page += F("'></label></div><div class='row'><label>Password<br><input type='password' name='password' maxlength='64'></label></div>");
  page += F("<div class='row'><label>Hostname<br><input name='hostname' maxlength='32' value='");
  page += htmlEscape(config.hostname);
  page += F("'></label></div>");
  appendPhyModeSelect(page);
  page += F("<button type='submit'>Save Wi-Fi</button></form>");
  page += F("<p><a class='btn secondary' href='/scan'>Scan networks</a></p></section>");

  appendTemplateForm(page);
  appendMqttForm(page);

  page += F("<section class='panel'><h2>Firmware</h2><form method='post' action='/update' enctype='multipart/form-data'>");
  page += F("<input type='file' name='firmware' accept='.bin,.bin.gz' required><br><button type='submit'>Upload firmware</button></form>");
  page += F("<p><a class='btn secondary' href='/reboot'>Reboot</a></p></section></div>");
  appendFooter(page);
  sendHtml(page);
}

void handleScan() {
  String page;
  page.reserve(2600);
  appendHeader(page, F("myMota Scan"));
  page += F("<section class='panel'><h2>Networks</h2>");
  const int count = WiFi.scanNetworks(false, true);
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
  page += F("<p>Reconnect to the device after it joins the configured network.</p>");
  appendFooter(page);
  sendHtml(page);
  restart_at = millis() + 1200;
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
    appendFooter(page);
    sendHtml(page);
    restart_at = millis() + 1200;
    return;
  }

  const String template_json = server.arg("template");
  if (template_json.length() == 0) {
    server.send(400, F("text/plain"), F("Template JSON is empty"));
    return;
  }

  StoredConfig candidate = config;
  String error;
  if (!parseTemplateJson(template_json, candidate, error)) {
    String msg = F("Invalid template: ");
    msg += error;
    msg += '\n';
    server.send(400, F("text/plain"), msg);
    return;
  }

  config = candidate;
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
  appendFooter(page);
  sendHtml(page);
  restart_at = millis() + 1200;
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
  if (!parseFloatInput(offset_arg, kEnergyTotalOffsetMinKwh, kEnergyTotalOffsetMaxKwh, total_offset_kwh)) {
    server.send(400, F("text/plain"), F("Invalid total kWh offset"));
    return;
  }

  if (!saveEnergyConfig(total_offset_kwh)) {
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

  String cmnd = server.arg("cmnd");
  cmnd.trim();
  int separator = -1;
  for (size_t i = 0; i < cmnd.length(); i++) {
    const char c = cmnd[i];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
      separator = static_cast<int>(i);
      break;
    }
  }
  if (separator <= 0 || separator >= static_cast<int>(cmnd.length()) - 1) {
    server.send(400, F("text/plain"), F("Invalid cmnd"));
    return;
  }

  const String command = cmnd.substring(0, separator);
  const String state_arg = cmnd.substring(separator + 1);
  uint8_t relay = 0;
  String response_key;
  bool on = false;
  if (!parsePowerCommand(command, relay, response_key)) {
    server.send(400, F("text/plain"), F("Unsupported command"));
    return;
  }
  if (!parsePowerState(state_arg, on)) {
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
  server.send(200, F("text/plain"), F("Rebooting\n"));
  restart_at = millis() + 500;
}

void handleHealth() {
  String out;
  out.reserve(1600);
  out += F("{\"name\":\"myMota\",\"version\":\"");
  out += F(MYMOTA_VERSION);
  out += F("\",\"target\":\"");
  out += F(MYMOTA_TARGET);
  out += F("\",\"chip\":\"");
  out += chipIdHex();
  out += F("\",\"heap\":");
  out += ESP.getFreeHeap();
  out += F(",\"uptime\":");
  out += millis() / 1000;
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
  out += F("}");
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
    out += F("}");
  } else {
    out += F("null");
  }
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
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), out);
}

void handleUpdateDone() {
  if (update_ok && !Update.hasError()) {
    String page;
    page.reserve(700);
    appendHeader(page, F("myMota Update"));
    page += F("<p class='ok'>Firmware uploaded. Rebooting.</p>");
    appendFooter(page);
    sendHtml(page);
    restart_at = millis() + 1200;
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
    update_error = UPDATE_ERROR_OK;
    WiFiUDP::stopAll();
    if (upload.filename.length() == 0) {
      update_error = UPDATE_ERROR_SIZE;
    }
    return;
  }

  if (update_error != UPDATE_ERROR_OK) {
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
      if (!Update.begin(max_sketch_space)) {
        update_error = Update.getError();
        return;
      }
      update_started = true;
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
    if (!update_started) {
      update_error = UPDATE_ERROR_SIZE;
    } else if (Update.end(true)) {
      update_ok = true;
    } else {
      update_error = Update.getError();
    }
    return;
  }

  if (upload.status == UPLOAD_FILE_ABORTED) {
    if (update_started) {
      Update.end();
    }
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
  ap_started = WiFi.softAP(ap_name.c_str(), kApPassword);
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
    if (config.phy_mode != kPhyModeAuto) {
      saveWifiConfig(config.ssid, config.password, config.hostname, kPhyModeFailsafe);
      prepareWifi();
      if (connectWifiWithPhy(config.phy_mode, kConnectTimeoutMs)) {
        return;
      }
    }
    startAp();
    disconnected_since = millis();
  }
}

void setupRoutes() {
  server.on(F("/"), HTTP_GET, handleRoot);
  server.on(F("/scan"), HTTP_GET, handleScan);
  server.on(F("/wifi"), HTTP_POST, handleWifiSave);
  server.on(F("/template"), HTTP_POST, handleTemplateSave);
  server.on(F("/mqtt"), HTTP_POST, handleMqttSave);
  server.on(F("/energy"), HTTP_POST, handleEnergySave);
  server.on(F("/power"), HTTP_POST, handlePowerSave);
  server.on(F("/cm"), HTTP_GET, handleCmnd);
  server.on(F("/reboot"), HTTP_GET, handleReboot);
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
    return;
  }
  if (disconnected_since == 0) {
    disconnected_since = millis();
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
  decodeTemplateConfig();
  setupDevicePins();
  connectWifi();
  setupRoutes();
  server.begin();

  Serial.printf("HTTP server started; STA %s AP %s\n",
                WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "not-connected",
                ap_started ? WiFi.softAPIP().toString().c_str() : "off");
}

void loop() {
  server.handleClient();
  maintainBootRecovery();
  maintainWifi();
  maintainDevice();
  maintainMqtt();

  if (restart_at && millis() > restart_at) {
    delay(50);
    ESP.restart();
  }

  delay(1);
}
