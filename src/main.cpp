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
constexpr uint16_t kConfigVersionV1 = 1;
constexpr uint16_t kConfigVersionV2 = 2;
constexpr uint16_t kConfigVersion = 3;
constexpr size_t kEepromSize = 512;
constexpr uint32_t kConnectTimeoutMs = 20000;
constexpr uint32_t kReconnectStartApMs = 90000;
constexpr uint32_t kApRetryMs = 10000;
constexpr const char *kApPassword = "mymota-setup";
constexpr uint8_t kPhyModeAuto = 0;
constexpr uint8_t kPhyModeFailsafe = WIFI_PHY_MODE_11G;
constexpr size_t kTemplateSlotCount = 14;
constexpr size_t kTemplateJsonMaxLen = 640;
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
  uint8_t reserved[3];
  uint32_t crc;
};

static_assert(sizeof(StoredConfig) <= kEepromSize, "StoredConfig exceeds EEPROM size");
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

void setDefaultConfig() {
  memset(&config, 0, sizeof(config));
  config.magic = kConfigMagic;
  config.version = kConfigVersion;
  config.size = sizeof(StoredConfig);
  config.phy_mode = kPhyModeAuto;
  strlcpy(config.hostname, defaultHostname().c_str(), sizeof(config.hostname));
  config.crc = configCrc(config);
  config_ok = false;
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
  if (config.hostname[0] == '\0') {
    strlcpy(config.hostname, defaultHostname().c_str(), sizeof(config.hostname));
  }
  config.phy_mode = sanitizePhyMode(config.phy_mode);
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

bool loadConfig() {
  EEPROM.begin(kEepromSize);
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
  relay_state[relay] = on;
  writeAssignedPin(runtime_template.relays[relay], on);
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
  page += F("</title><style>body{font-family:sans-serif;max-width:720px;margin:24px auto;padding:0 14px;line-height:1.35}");
  page += F("input,button,select,textarea{font:inherit;padding:8px;margin:4px 0;box-sizing:border-box}input[type=text],input[type=password],input[type=file],select,textarea{width:100%;max-width:560px}");
  page += F("button{cursor:pointer}.row{margin:12px 0}.muted{color:#666}.ok{color:#176b32}.bad{color:#9b1c1c}code{background:#eee;padding:2px 4px}.inline{display:inline}</style></head><body>");
  page += F("<h1>Mymota</h1>");
}

void appendFooter(String &page) {
  page += F("</body></html>");
}

void sendHtml(String &page) {
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("text/html"), page);
}

void appendStatusBlock(String &page) {
  page += F("<h2>Status</h2><ul>");
  page += F("<li>Version: <code>");
  page += F(MYMOTA_VERSION);
  page += F("</code> / <code>");
  page += F(MYMOTA_TARGET);
  page += F("</code></li><li>Chip: <code>");
  page += chipIdHex();
  page += F("</code></li><li>Hostname: <code>");
  page += htmlEscape(config.hostname);
  page += F("</code></li><li>Heap: <code>");
  page += String(ESP.getFreeHeap());
  page += F("</code> bytes</li><li>Uptime: <code>");
  page += String(millis() / 1000);
  page += F("</code>s</li>");
  page += F("<li>PHY mode: <code>");
  page += phyModeName(config.phy_mode);
  page += F("</code> configured / <code>");
  page += phyModeName(WiFi.getPhyMode());
  page += F("</code> active</li>");

  if (WiFi.status() == WL_CONNECTED) {
    page += F("<li>Wi-Fi: <span class='ok'>connected</span> to <code>");
    page += htmlEscape(WiFi.SSID());
    page += F("</code></li><li>IP: <code>");
    page += ipToString(WiFi.localIP());
    page += F("</code></li><li>RSSI: <code>");
    page += String(WiFi.RSSI());
    page += F("</code> dBm</li>");
  } else {
    page += F("<li>Wi-Fi: <span class='bad'>not connected</span></li>");
  }

  if (ap_started) {
    page += F("<li>Setup AP: <code>");
    page += htmlEscape(WiFi.softAPSSID());
    page += F("</code> / <code>");
    page += kApPassword;
    page += F("</code> at <code>");
    page += ipToString(WiFi.softAPIP());
    page += F("</code></li>");
  }
  page += F("</ul>");
}

void appendTemplateStatus(String &page) {
  page += F("<h2>Template</h2><ul>");
  if (!runtime_template.enabled) {
    page += F("<li>No template configured.</li>");
  } else {
    page += F("<li>Name: <code>");
    page += htmlEscape(runtime_template.name);
    page += F("</code></li><li>Base: <code>");
    page += String(runtime_template.base);
    page += F("</code>, flag: <code>");
    page += String(runtime_template.flag);
    page += F("</code></li><li>Relays: <code>");
    page += String(runtime_template.relay_count);
    page += F("</code>, buttons: <code>");
    page += String(runtime_template.button_count);
    page += F("</code>, LEDs: <code>");
    page += String(runtime_template.led_count);
    page += F("</code></li>");
    if (energy.present) {
      page += F("<li>Energy: <code>");
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
      page += F("</code></li>");
    }
    if (runtime_template.adc_temp) {
      page += F("<li>ADC temperature: <code>");
      if (isnan(adc_temperature_c)) {
        page += F("n/a");
      } else {
        page += String(adc_temperature_c, 1);
        page += F(" C");
      }
      page += F("</code> raw <code>");
      page += String(adc_raw);
      page += F("</code></li>");
    }
    if (runtime_template.unsupported_count) {
      page += F("<li class='bad'>Unsupported GPIO functions:");
      for (uint8_t i = 0; i < runtime_template.unsupported_count; i++) {
        page += F(" <code>");
        page += pinName(runtime_template.unsupported_pin[i]);
        page += F("=");
        page += String(runtime_template.unsupported_code[i]);
        page += F("</code>");
      }
      page += F("</li>");
    }
  }
  page += F("</ul>");
}

void appendDeviceControls(String &page) {
  if (!runtime_template.enabled || runtime_template.relay_count == 0) return;
  page += F("<h2>Device</h2><ul>");
  for (uint8_t i = 0; i < runtime_template.relay_count; i++) {
    if (!hasPin(runtime_template.relays[i])) continue;
    page += F("<li>Relay ");
    page += String(i + 1);
    page += F(" on <code>");
    page += pinName(runtime_template.relays[i].pin);
    page += F("</code>: ");
    if (relay_state[i]) {
      page += F("<span class='ok'>on</span>");
    } else {
      page += F("<span class='muted'>off</span>");
    }
    page += F(" <form class='inline' method='post' action='/power'><input type='hidden' name='relay' value='");
    page += String(i + 1);
    page += F("'><button name='state' value='toggle'>Toggle</button><button name='state' value='on'>On</button><button name='state' value='off'>Off</button></form></li>");
  }
  if (energy.present) {
    page += F("<li>Power: <code>");
    page += String(energy.power, 1);
    page += F("</code> W, voltage: <code>");
    page += String(energy.voltage, 1);
    page += F("</code> V, current: <code>");
    page += String(energy.current, 3);
    page += F("</code> A, total: <code>");
    page += String(energy.total_kwh, 4);
    page += F("</code> kWh</li>");
  }
  page += F("</ul>");
}

void appendTemplateForm(String &page) {
  page += F("<h2>Template</h2><form method='post' action='/template'>");
  page += F("<div class='row'><label>Tasmota ESP8266 template JSON<br><textarea name='template' rows='5' maxlength='");
  page += String(kTemplateJsonMaxLen);
  page += F("'>");
  page += htmlEscape(currentTemplateJson());
  page += F("</textarea></label></div>");
  page += F("<button type='submit'>Save template</button> <button type='submit' name='clear' value='1'>Clear template</button></form>");
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
  page.reserve(5200);
  appendHeader(page, F("Mymota"));
  appendStatusBlock(page);
  appendTemplateStatus(page);
  appendDeviceControls(page);
  page += F("<h2>Wi-Fi</h2><form method='post' action='/wifi'>");
  page += F("<div class='row'><label>SSID<br><input name='ssid' maxlength='32' required value='");
  page += htmlEscape(config.ssid);
  page += F("'></label></div><div class='row'><label>Password<br><input type='password' name='password' maxlength='64'></label></div>");
  page += F("<div class='row'><label>Hostname<br><input name='hostname' maxlength='32' value='");
  page += htmlEscape(config.hostname);
  page += F("'></label></div>");
  appendPhyModeSelect(page);
  page += F("<button type='submit'>Save Wi-Fi</button></form>");
  page += F("<p><a href='/scan'>Scan networks</a></p>");

  appendTemplateForm(page);

  page += F("<h2>Firmware</h2><form method='post' action='/update' enctype='multipart/form-data'>");
  page += F("<input type='file' name='firmware' accept='.bin,.bin.gz' required><br><button type='submit'>Upload firmware</button></form>");
  page += F("<p><a href='/reboot'>Reboot</a></p>");
  appendFooter(page);
  sendHtml(page);
}

void handleScan() {
  String page;
  page.reserve(2600);
  appendHeader(page, F("Mymota Scan"));
  page += F("<h2>Networks</h2>");
  const int count = WiFi.scanNetworks(false, true);
  if (count <= 0) {
    page += F("<p>No networks found.</p>");
  } else {
    page += F("<form method='post' action='/wifi'><div class='row'><label>Password<br><input type='password' name='password' maxlength='64'></label></div>");
    page += F("<div class='row'><label>Hostname<br><input name='hostname' maxlength='32' value='");
    page += htmlEscape(config.hostname);
    page += F("'></label></div>");
    appendPhyModeSelect(page);
    page += F("<ul>");
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
  page += F("<p><a href='/'>Back</a></p>");
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
  appendHeader(page, F("Mymota Wi-Fi"));
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
    appendHeader(page, F("Mymota Template"));
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
  appendHeader(page, F("Mymota Template"));
  page += F("<p class='ok'>Template saved. Rebooting.</p>");
  if (runtime_template.unsupported_count) {
    page += F("<p class='bad'>The template contains unsupported GPIO functions. Check the Template section after reboot.</p>");
  }
  appendFooter(page);
  sendHtml(page);
  restart_at = millis() + 1200;
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

void handleReboot() {
  server.send(200, F("text/plain"), F("Rebooting\n"));
  restart_at = millis() + 500;
}

void handleHealth() {
  String out;
  out.reserve(900);
  out += F("{\"name\":\"mymota\",\"version\":\"");
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
  out += (WiFi.status() == WL_CONNECTED) ? F("true") : F("false");
  out += F(",\"configured_phy_mode\":");
  out += config.phy_mode;
  out += F(",\"configured_phy\":\"");
  out += phyModeName(config.phy_mode);
  out += F("\",\"active_phy_mode\":");
  out += WiFi.getPhyMode();
  out += F(",\"active_phy\":\"");
  out += phyModeName(WiFi.getPhyMode());
  out += F("\",\"template\":{\"enabled\":");
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
    if (!hasPin(runtime_template.relays[i])) continue;
    if (!first) out += ',';
    first = false;
    if (relay_state[i]) {
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
    out += String(energy.total_kwh, 4);
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
  out += F("}");
  server.send(200, F("application/json"), out);
}

void handleUpdateDone() {
  if (update_ok && !Update.hasError()) {
    String page;
    page.reserve(700);
    appendHeader(page, F("Mymota Update"));
    page += F("<p class='ok'>Firmware uploaded. Rebooting.</p>");
    appendFooter(page);
    sendHtml(page);
    restart_at = millis() + 1200;
    return;
  }

  String page;
  page.reserve(800);
  appendHeader(page, F("Mymota Update Failed"));
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
  server.on(F("/power"), HTTP_POST, handlePowerSave);
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
  Serial.printf("Mymota %s %s chip %06X\n", MYMOTA_VERSION, MYMOTA_TARGET, ESP.getChipId());

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
  maintainWifi();
  maintainDevice();

  if (restart_at && millis() > restart_at) {
    delay(50);
    ESP.restart();
  }

  delay(1);
}
