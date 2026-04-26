#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <Updater.h>
#include <WiFiUdp.h>

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
constexpr uint16_t kConfigVersion = 2;
constexpr size_t kEepromSize = 512;
constexpr uint32_t kConnectTimeoutMs = 20000;
constexpr uint32_t kReconnectStartApMs = 90000;
constexpr uint32_t kApRetryMs = 10000;
constexpr const char *kApPassword = "mymota-setup";
constexpr uint8_t kPhyModeAuto = 0;
constexpr uint8_t kPhyModeFailsafe = WIFI_PHY_MODE_11G;

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

struct StoredConfig {
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

ESP8266WebServer server(80);
StoredConfig config{};

bool config_ok = false;
bool ap_started = false;
bool update_started = false;
bool update_ok = false;
uint8_t update_error = UPDATE_ERROR_OK;
uint32_t restart_at = 0;
uint32_t disconnected_since = 0;
uint32_t last_ap_attempt = 0;

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

void normalizeConfigStrings() {
  config.ssid[sizeof(config.ssid) - 1] = '\0';
  config.password[sizeof(config.password) - 1] = '\0';
  config.hostname[sizeof(config.hostname) - 1] = '\0';
  if (config.hostname[0] == '\0') {
    strlcpy(config.hostname, defaultHostname().c_str(), sizeof(config.hostname));
  }
  config.phy_mode = sanitizePhyMode(config.phy_mode);
}

bool saveConfig(const char *ssid, const char *password, const char *hostname, uint8_t phy_mode);

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
    normalizeConfigStrings();
    config.crc = configCrc(config);
    config_ok = config.ssid[0] != '\0';
    if (config_ok) {
      saveConfig(config.ssid, config.password, config.hostname, config.phy_mode);
    }
    return config_ok;
  }

  setDefaultConfig();
  return false;
}

bool saveConfig(const char *ssid, const char *password, const char *hostname, uint8_t phy_mode) {
  memset(&config, 0, sizeof(config));
  config.magic = kConfigMagic;
  config.version = kConfigVersion;
  config.size = sizeof(StoredConfig);
  strlcpy(config.ssid, ssid, sizeof(config.ssid));
  strlcpy(config.password, password, sizeof(config.password));
  if (hostname && hostname[0]) {
    strlcpy(config.hostname, hostname, sizeof(config.hostname));
  } else {
    strlcpy(config.hostname, defaultHostname().c_str(), sizeof(config.hostname));
  }
  config.phy_mode = sanitizePhyMode(phy_mode);
  config.crc = configCrc(config);

  EEPROM.put(0, config);
  const bool committed = EEPROM.commit();
  config_ok = committed && config.ssid[0] != '\0';
  return committed;
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

void appendHeader(String &page, const __FlashStringHelper *title) {
  page += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  page += F("<title>");
  page += title;
  page += F("</title><style>body{font-family:sans-serif;max-width:720px;margin:24px auto;padding:0 14px;line-height:1.35}");
  page += F("input,button,select{font:inherit;padding:8px;margin:4px 0;box-sizing:border-box}input[type=text],input[type=password],input[type=file],select{width:100%;max-width:460px}");
  page += F("button{cursor:pointer}.row{margin:12px 0}.muted{color:#666}.ok{color:#176b32}.bad{color:#9b1c1c}code{background:#eee;padding:2px 4px}</style></head><body>");
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
  page.reserve(2800);
  appendHeader(page, F("Mymota"));
  appendStatusBlock(page);
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

  if (!saveConfig(ssid.c_str(), password_to_save, hostname.c_str(), phy_mode)) {
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

void handleReboot() {
  server.send(200, F("text/plain"), F("Rebooting\n"));
  restart_at = millis() + 500;
}

void handleHealth() {
  String out;
  out.reserve(260);
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
  out += F("\"");
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
      saveConfig(config.ssid, config.password, config.hostname, kPhyModeFailsafe);
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

  if (restart_at && millis() > restart_at) {
    delay(50);
    ESP.restart();
  }

  delay(1);
}
