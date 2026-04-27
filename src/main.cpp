#include <Arduino.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <Updater.h>
#include <WiFiUdp.h>
#include <Wire.h>
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
constexpr uint16_t kConfigVersionV10 = 10;
constexpr uint16_t kConfigVersionV11 = 11;
constexpr uint16_t kConfigVersionV12 = 12;
constexpr uint16_t kConfigVersionV13 = 13;
constexpr uint16_t kConfigVersionV14 = 14;
constexpr uint16_t kConfigVersion = 15;
constexpr size_t kEepromSize = 4096;
constexpr size_t kFlashSectorSize = 4096;
constexpr uint8_t kEnergyJournalSectorCount = 2;
constexpr size_t kBootRecoveryOffset = 3072;
constexpr uint32_t kConnectTimeoutMs = 20000;
constexpr uint32_t kWifiReconnectBeginMs = 60000;
constexpr uint32_t kInitialFallbackApMs = 300000;
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
constexpr uint32_t kMqttSubackMaxRemainingLength = 16;
constexpr uint32_t kMqttInboundMaxRemainingLength = 512;
constexpr uint8_t kMqttInboundPacketLimit = 4;
constexpr uint16_t kMqttCommandPacketId = 1;
constexpr size_t kMqttCommandTopicMaxLen = 5 + kMqttTopicMaxLen + 2;
constexpr size_t kMqttInboundTopicMaxLen = kMqttCommandTopicMaxLen + 32;
constexpr size_t kMqttInboundPayloadMaxLen = 96;
constexpr uint16_t kMqttEnergyIntervalMax = 65535U;
constexpr float kMqttEnergyChangeMaxPercent = 1000.0f;
constexpr uint8_t kMqttPacketConnack = 0x20;
constexpr uint8_t kMqttPacketPublish = 0x30;
constexpr uint8_t kMqttPacketPuback = 0x40;
constexpr uint8_t kMqttPacketSubscribe = 0x82;
constexpr uint8_t kMqttPacketSuback = 0x90;
constexpr uint8_t kMqttPacketPingreq = 0xc0;
constexpr uint8_t kMqttPacketPingresp = 0xd0;
constexpr uint8_t kInvalidPin = 0xff;
constexpr uint8_t kAdc0Pin = 17;
constexpr uint8_t kMaxRelays = 8;
constexpr uint8_t kMaxButtons = 4;
constexpr uint8_t kMaxLeds = 4;
constexpr uint8_t kMaxLedOutputs = kMaxLeds + 1;
constexpr uint8_t kMaxLightPwms = 2;
constexpr uint8_t kMaxRotaries = 1;
constexpr uint16_t kRelayEnforcementMinSeconds = 1;
constexpr uint16_t kRelayEnforcementMaxSeconds = 65535U;
constexpr uint16_t kButtonDebounceDefaultMs = 50;
constexpr uint16_t kButtonDebounceMinMs = 5;
constexpr uint16_t kButtonDebounceMaxMs = 200;
constexpr uint16_t kButtonHoldDefaultMs = 500;
constexpr uint16_t kButtonHoldMinMs = 1;
constexpr uint16_t kButtonHoldMaxMs = 60000;
constexpr uint32_t kLedUpdateMs = 250;
constexpr uint16_t kLightPwmRange = 1023;
constexpr uint16_t kLightPwmFrequency = 1000;
constexpr uint8_t kLightDimmerOff = 0;
constexpr uint8_t kLightDimmerMin = 1;
constexpr uint8_t kLightDimmerMax = 100;
constexpr uint8_t kLightDimmerDefault = 50;
constexpr uint8_t kLightPowerOnDimmerDefault = 50;
constexpr uint16_t kLightCtMin = 153;
constexpr uint16_t kLightCtMax = 500;
constexpr uint16_t kLightCtDefault = 326;
constexpr uint32_t kLightPersistDelayMs = 2000;
constexpr uint32_t kRotaryHandlerMs = 50;
constexpr uint8_t kRotaryOffset = 128;
constexpr uint8_t kRotaryMaxSteps = 10;
constexpr uint8_t kRotaryMiDeskStepScale = 3;
constexpr uint32_t kShellyDimmerPollMs = 1000;
constexpr uint32_t kShellyDimmerRetryMs = 1000;
constexpr uint32_t kShellyDimmerStaleMs = 5000;
constexpr uint32_t kShellyDimmerResponseTimeoutMs = 200;
constexpr uint8_t kShellyDimmerBufferSize = 64;
constexpr uint8_t kShellyDimmerMaxPayloadSize = 17;
constexpr uint8_t kShellyDimmerEdgeAuto = 0;
constexpr uint8_t kShellyDimmerEdgeTrailing = 1;
constexpr uint8_t kShellyDimmerEdgeLeading = 2;
constexpr uint8_t kShellyDimmerEdgeDefault = kShellyDimmerEdgeAuto;
constexpr uint8_t kShellyDimmerRangeMinDefault = 0;
constexpr uint8_t kShellyDimmerRangeMaxDefault = 100;
constexpr uint8_t kShellyDimmerRangeMaxLimit = 255;
constexpr uint8_t kShellyDimmerStartByte = 0x01;
constexpr uint8_t kShellyDimmerEndByte = 0x04;
constexpr uint8_t kShellyDimmerSwitchCmd = 0x01;
constexpr uint8_t kShellyDimmerPollCmd = 0x10;
constexpr uint8_t kShellyDimmerVersionCmd = 0x11;
constexpr uint8_t kShellyDimmerSettingsCmd = 0x20;
constexpr uint16_t kShellyDimmerDefaultWarmupBrightness = 0;
constexpr uint16_t kShellyDimmerDefaultWarmupMs = 0;
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
constexpr uint8_t kButtonRelayUnset = 255;
constexpr uint8_t kInputKindButton = 0;
constexpr uint8_t kInputKindSwitch = 1;
constexpr uint8_t kInputModeButton = 0;
constexpr uint8_t kInputModeSwitch = 1;
constexpr uint8_t kInputModeUnset = 255;
constexpr uint8_t kInputOnLevelLow = 0;
constexpr uint8_t kInputOnLevelHigh = 1;
constexpr uint8_t kInputOnLevelUnset = 255;
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
constexpr size_t kSettingsImportJsonMaxLen = 8192;
constexpr size_t kSettingsImportDocCapacity = 12288;
constexpr uint16_t kSettingsFormatVersion = 1;
constexpr size_t kApiSettingsDocCapacity = 4096;
constexpr uint16_t kApiSettingsVersion = 2;
constexpr const char *kDefaultButtonMqttTopic = "stat/{TOPIC}/RESULT";
constexpr const char *kDefaultButtonMqttPressPayload = "{\"Switch{BUTTONID}\":{\"Action\":\"{TYPE}\"}}";
constexpr const char *kDefaultButtonMqttHoldPayload = "{\"Switch{BUTTONID}\":{\"Action\":\"{TYPE}\"}}";
constexpr uint8_t kMqttConnectIdle = 0;
constexpr uint8_t kMqttConnectOk = 1;
constexpr uint8_t kMqttConnectTcpFailed = 2;
constexpr uint8_t kMqttConnectWriteFailed = 3;
constexpr uint8_t kMqttConnectConnackTimeout = 4;
constexpr uint8_t kMqttConnectConnackRejected = 5;
constexpr uint8_t kMqttConnectSubscribeFailed = 6;
constexpr uint8_t kMqttLightPendingDimmer = 0x01;
constexpr uint8_t kMqttLightPendingCt = 0x02;
constexpr uint8_t kMqttLightPendingAll = kMqttLightPendingDimmer | kMqttLightPendingCt;
constexpr uint8_t kMqttEnergyReportReasonNone = 0;
constexpr uint8_t kMqttEnergyReportReasonInitial = 1;
constexpr uint8_t kMqttEnergyReportReasonInterval = 2;
constexpr uint8_t kMqttEnergyReportReasonPowerChange = 3;
constexpr uint8_t kMqttEnergyReportReasonIntervalPowerChange = 4;
constexpr uint8_t kPowerStateOff = 0;
constexpr uint8_t kPowerStateOn = 1;
constexpr uint8_t kPowerStateToggle = 2;

constexpr uint16_t kTplNone = 0;
constexpr uint16_t kTplUser = 1;
constexpr uint16_t kTplKey1 = 32;
constexpr uint16_t kTplKey1Np = 64;
constexpr uint16_t kTplKey1Inv = 96;
constexpr uint16_t kTplKey1InvNp = 128;
constexpr uint16_t kTplSwt1 = 160;
constexpr uint16_t kTplSwt1Np = 192;
constexpr uint16_t kTplRel1 = 224;
constexpr uint16_t kTplRel1Inv = 256;
constexpr uint16_t kTplLed1 = 288;
constexpr uint16_t kTplLed1Inv = 320;
constexpr uint16_t kTplPwm1 = 416;
constexpr uint16_t kTplPwm1Inv = 448;
constexpr uint16_t kTplLedLnk = 544;
constexpr uint16_t kTplLedLnkInv = 576;
constexpr uint16_t kTplI2cScl = 608;
constexpr uint16_t kTplI2cSda = 640;
constexpr uint16_t kTplNrgSel = 2592;
constexpr uint16_t kTplNrgSelInv = 2624;
constexpr uint16_t kTplNrgCf1 = 2656;
constexpr uint16_t kTplHlwCf = 2688;
constexpr uint16_t kTplHjlCf = 2720;
constexpr uint16_t kTplSerialTxd = 3200;
constexpr uint16_t kTplSerialRxd = 3232;
constexpr uint16_t kTplRot1A = 3264;
constexpr uint16_t kTplRot1B = 3296;
constexpr uint16_t kTplAde7953Irq = 3456;
constexpr uint16_t kTplAdcTemp = 4736;
constexpr uint16_t kTplShellyDimmerBoot0 = 5568;
constexpr uint16_t kTplShellyDimmerResetInv = 5600;

const char kTemplateMiDeskLampJson[] PROGMEM =
  "{\"NAME\":\"Mi Desk Lamp\",\"GPIO\":[0,0,32,0,416,417,0,0,3264,3296,0,0,0,0],\"FLAG\":0,\"BASE\":66}";
const char kTemplateShellyPlugSJson[] PROGMEM =
  "{\"NAME\":\"Shelly Plug S\",\"GPIO\":[320,1,576,1,1,2720,0,0,2624,32,2656,224,1,4736],\"FLAG\":0,\"BASE\":45}";
const char kTemplateNousA1TJson[] PROGMEM =
  "{\"NAME\":\"NOUS A1T\",\"GPIO\":[32,0,0,0,2720,2656,0,0,2624,320,224,0,0,0],\"FLAG\":0,\"BASE\":18}";
const char kTemplateShelly25Json[] PROGMEM =
  "{\"NAME\":\"Shelly 2.5\",\"GPIO\":[320,0,0,0,224,193,0,0,640,192,608,225,3456,4736],\"FLAG\":0,\"BASE\":18}";
const char kTemplateShelly1Json[] PROGMEM =
  "{\"NAME\":\"Shelly 1\",\"GPIO\":[1,1,0,1,224,192,0,0,0,0,0,0,0,0],\"FLAG\":0,\"BASE\":46}";
const char kTemplateShelly1LJson[] PROGMEM =
  "{\"NAME\":\"Shelly 1L\",\"GPIO\":[320,0,0,0,192,224,0,0,0,0,193,0,0,4736],\"FLAG\":0,\"BASE\":18}";
const char kTemplateShellyDimmer2Json[] PROGMEM =
  "{\"NAME\":\"Shelly Dimmer 2\",\"GPIO\":[0,3200,0,3232,5568,5600,0,0,193,0,192,0,320,4736],\"FLAG\":0,\"BASE\":18}";

enum Ade7953Register : uint16_t {
  kAde7953DisNoLoad = 0x001,
  kAde7953Config = 0x102,
  kAde7953PhcalA = 0x108,
  kAde7953PhcalB = 0x109,
  kAde7953Reserved120 = 0x120,
  kAde7953Unlock120 = 0x0fe,
  kAde7953AccMode = 0x301,
  kAde7953ApNoLoad = 0x303,
  kAde7953VarNoLoad = 0x304,
  kAde7953IrmsA = 0x31a,
  kAde7953IrmsB = 0x31b,
  kAde7953Vrms = 0x31c,
  kAde7953AEnergyA = 0x31e,
  kAde7953AEnergyB = 0x31f,
  kAde7953ApEnergyA = 0x322,
  kAde7953ApEnergyB = 0x323,
  kAde7953AIGain = 0x380,
  kAde7953AVGain = 0x381,
  kAde7953AWGain = 0x382,
  kAde7953AVarGain = 0x383,
  kAde7953AVAGain = 0x384,
  kAde7953BIGain = 0x38c,
  kAde7953BWGain = 0x38e,
  kAde7953BVarGain = 0x38f,
  kAde7953BVAGain = 0x390
};

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
constexpr uint8_t kEnergyDriverNone = 0;
constexpr uint8_t kEnergyDriverPulse = 1;
constexpr uint8_t kEnergyDriverAde7953 = 2;
constexpr uint8_t kEnergyDriverShellyDimmer = 3;
constexpr uint8_t kEnergyMaxChannels = 2;
constexpr uint8_t kAde7953Address = 0x38;
constexpr uint8_t kAde7953ModelShelly25 = 0;
constexpr uint32_t kAde7953UpdateMs = 1000;
constexpr uint32_t kAde7953StaleMs = 5000;
constexpr uint8_t kAde7953SkipInitialReads = 1;
constexpr uint32_t kAde7953PowerCal = 1540;
constexpr uint32_t kAde7953VoltageCal = 26000;
constexpr uint32_t kAde7953CurrentCal = 10000;
constexpr uint32_t kAde7953GainDefault = 4194304;
constexpr uint32_t kAde7953NoLoadThreshold = 29196;
constexpr float kAde7953PowerCorrection = 23.41494f;
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

struct StoredConfigV10 {
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

struct StoredConfigV11 {
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
  uint8_t input_mode[kMaxButtons];
  uint8_t input_relay[kMaxButtons];
  uint8_t input_on_level[kMaxButtons];
  uint8_t reserved[1];
  uint32_t crc;
};

struct StoredConfigV12 {
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
  uint8_t input_mode[kMaxButtons];
  uint8_t input_relay[kMaxButtons];
  uint8_t input_on_level[kMaxButtons];
  uint8_t reserved[1];
  uint8_t button_press_relay[kMaxButtons];
  uint8_t button_hold_relay[kMaxButtons];
  uint32_t crc;
};

struct StoredConfigV13 {
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
  uint8_t input_mode[kMaxButtons];
  uint8_t input_relay[kMaxButtons];
  uint8_t input_on_level[kMaxButtons];
  uint8_t reserved[1];
  uint8_t button_press_relay[kMaxButtons];
  uint8_t button_hold_relay[kMaxButtons];
  uint8_t light_power;
  uint8_t light_dimmer;
  uint16_t light_ct;
  uint32_t crc;
};

struct StoredConfigV14 {
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
  uint8_t input_mode[kMaxButtons];
  uint8_t input_relay[kMaxButtons];
  uint8_t input_on_level[kMaxButtons];
  uint8_t reserved[1];
  uint8_t button_press_relay[kMaxButtons];
  uint8_t button_hold_relay[kMaxButtons];
  uint8_t light_power;
  uint8_t light_dimmer;
  uint16_t light_ct;
  uint8_t light_on_dimmer;
  uint8_t light_reserved[3];
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
  uint8_t input_mode[kMaxButtons];
  uint8_t input_relay[kMaxButtons];
  uint8_t input_on_level[kMaxButtons];
  uint8_t reserved[1];
  uint8_t button_press_relay[kMaxButtons];
  uint8_t button_hold_relay[kMaxButtons];
  uint8_t light_power;
  uint8_t light_dimmer;
  uint16_t light_ct;
  uint8_t light_on_dimmer;
  uint8_t shelly_dimmer_edge;
  uint8_t shelly_dimmer_range_min;
  uint8_t shelly_dimmer_range_max;
  uint8_t relay_on_boot[kMaxRelays];
  uint8_t relay_time_enabled[kMaxRelays];
  uint16_t relay_time_seconds[kMaxRelays];
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
  uint8_t input_kind[kMaxButtons];
  PinAssignment leds[kMaxLeds];
  PinAssignment link_led;
  PinAssignment light_pwm[kMaxLightPwms];
  PinAssignment rotary_a[kMaxRotaries];
  PinAssignment rotary_b[kMaxRotaries];
  uint8_t relay_count;
  uint8_t button_count;
  uint8_t led_count;
  uint8_t pwm_count;
  uint8_t rotary_count;
  uint8_t i2c_scl_pin;
  uint8_t i2c_sda_pin;
  uint8_t ade7953_irq_pin;
  uint8_t ade7953_model;
  uint8_t energy_cf_pin;
  uint8_t energy_cf1_pin;
  uint8_t energy_sel_pin;
  uint8_t serial_tx_pin;
  uint8_t serial_rx_pin;
  uint8_t shelly_dimmer_boot0_pin;
  uint8_t shelly_dimmer_reset_pin;
  bool energy_sel_inverted;
  bool energy_hjl;
  bool adc_temp;
  bool shelly_dimmer;
  uint8_t unsupported_count;
  uint8_t unsupported_pin[8];
  uint16_t unsupported_code[8];
};

struct LightState {
  bool present;
  bool power;
  uint8_t dimmer;
  uint16_t ct;
  bool config_dirty;
  uint32_t config_save_at;
};

struct RotaryEncoderState {
  bool present;
  PinAssignment a;
  PinAssignment b;
  volatile uint8_t state;
  volatile int16_t position;
  bool changed_while_pressed;
};

struct ShellyDimmerState {
  bool present;
  bool serial_claimed;
  uint8_t counter;
  uint8_t buffer[kShellyDimmerBufferSize];
  uint8_t byte_count;
  uint8_t expected_frame_len;
  uint8_t version_major;
  uint8_t version_minor;
  uint8_t hw_version;
  uint16_t actual_brightness;
  uint16_t requested_brightness;
  uint8_t fade_rate;
  uint32_t wattage_raw;
  uint32_t voltage_raw;
  uint32_t current_raw;
  uint32_t last_poll_ms;
  uint32_t last_command_ms;
  uint32_t last_rx_ms;
  uint32_t timeout_count;
  uint32_t error_count;
};

struct EnergyChannelState {
  float voltage;
  float current;
  float power;
  uint32_t voltage_raw;
  uint32_t current_raw;
  uint32_t active_power_raw;
  uint32_t apparent_power_raw;
};

struct EnergyState {
  bool present;
  uint8_t driver;
  uint8_t channel_count;
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
  uint32_t last_success_ms;
  uint8_t power_retry;
  uint8_t ade7953_model;
  uint8_t ade7953_skip_reads;
  uint16_t i2c_error_count;
  uint32_t ade7953_acc_mode;
  uint32_t ade7953_sample_ms;
  EnergyChannelState channel[kEnergyMaxChannels];
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
LightState light{};
RotaryEncoderState rotary_encoder{};
ShellyDimmerState shelly_dimmer{};
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
uint32_t last_wifi_begin_attempt = 0;
bool sta_connected_once = false;
uint32_t last_ap_attempt = 0;
uint32_t last_led_update = 0;
uint32_t last_rotary_handler = 0;
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
uint8_t last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
uint16_t mqtt_pending_relay_mask = 0;
uint8_t mqtt_pending_light_mask = 0;
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
bool relay_enforcement_pending[kMaxRelays]{};
uint32_t relay_enforcement_due[kMaxRelays]{};
bool rotary_suppress_button[kMaxButtons]{};
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
bool relayAvailable(uint8_t relay);
bool lightAvailableIn(const RuntimeTemplate &rt);
bool defaultButtonRelayTarget(uint8_t button, uint8_t &relay);
bool parseUint16Input(const String &input, uint16_t min_value, uint16_t max_value, uint16_t &out);
void cancelRelayEnforcement(uint8_t relay);
void refreshRelayEnforcementRuntime(bool schedule_off_relays);
bool executeDeviceCommand(const char *raw, size_t cmd_len, const char *arg, size_t arg_len, String &out, String &error);
void scheduleMqttLightPublish(uint8_t mask);
void toggleLightPower(bool persist = true);

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
  return "tasmota_" + chipIdHex();
}

void setDefaultMqttConfig() {
  memset(config.mqtt_host, 0, sizeof(config.mqtt_host));
  config.mqtt_port = kMqttDefaultPort;
  config.mqtt_keepalive = 600;
  strlcpy(config.mqtt_topic, defaultMqttTopic().c_str(), sizeof(config.mqtt_topic));
}

void setDefaultEnergyMqttConfig() {
  config.energy_mqtt_interval = 0;
  config.energy_mqtt_change_percent_x10 = 0;
}

void setDefaultLightConfig(StoredConfig &target) {
  target.light_power = 0;
  target.light_dimmer = kLightDimmerOff;
  target.light_ct = kLightCtDefault;
  target.light_on_dimmer = kLightPowerOnDimmerDefault;
  target.shelly_dimmer_edge = kShellyDimmerEdgeDefault;
  target.shelly_dimmer_range_min = kShellyDimmerRangeMinDefault;
  target.shelly_dimmer_range_max = kShellyDimmerRangeMaxDefault;
}

void setDefaultLightConfig() {
  setDefaultLightConfig(config);
}

void setDefaultRelayEnforcementConfig(StoredConfig &target) {
  memset(target.relay_on_boot, 0, sizeof(target.relay_on_boot));
  memset(target.relay_time_enabled, 0, sizeof(target.relay_time_enabled));
  memset(target.relay_time_seconds, 0, sizeof(target.relay_time_seconds));
}

void setDefaultRelayEnforcementConfig() {
  setDefaultRelayEnforcementConfig(config);
}

uint8_t defaultLedAttachment(uint8_t led) {
  if (led < kMaxLeds && led < kMaxRelays) {
    return kLedAttachRelayBase + led;
  }
  return kLedAttachNone;
}

void setDefaultLedConfig(StoredConfig &target) {
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    target.led_attach[i] = defaultLedAttachment(i);
  }
}

void setDefaultLedConfig() {
  setDefaultLedConfig(config);
}

void setDefaultButtonActionText(StoredConfig &target, uint8_t button) {
  if (button >= kMaxButtons) return;
  strlcpy(target.button_press_target[button], kDefaultButtonMqttTopic, sizeof(target.button_press_target[button]));
  strlcpy(target.button_press_payload[button], kDefaultButtonMqttPressPayload, sizeof(target.button_press_payload[button]));
  strlcpy(target.button_hold_target[button], kDefaultButtonMqttTopic, sizeof(target.button_hold_target[button]));
  strlcpy(target.button_hold_payload[button], kDefaultButtonMqttHoldPayload, sizeof(target.button_hold_payload[button]));
}

void setDefaultButtonActionText(uint8_t button) {
  setDefaultButtonActionText(config, button);
}

void setDefaultButtonActionTexts(StoredConfig &target) {
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    setDefaultButtonActionText(target, i);
  }
}

void setDefaultButtonActionTexts() {
  setDefaultButtonActionTexts(config);
}

void setDefaultButtonRelayConfig(StoredConfig &target) {
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    target.button_press_relay[i] = kButtonRelayUnset;
    target.button_hold_relay[i] = kButtonRelayUnset;
  }
}

void setDefaultButtonRelayConfig() {
  setDefaultButtonRelayConfig(config);
}

void setDefaultInputConfig(StoredConfig &target) {
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    target.input_mode[i] = kInputModeUnset;
    target.input_relay[i] = i;
    target.input_on_level[i] = kInputOnLevelUnset;
  }
  memset(target.reserved, 0, sizeof(target.reserved));
}

void setDefaultInputConfig() {
  setDefaultInputConfig(config);
}

void setDefaultButtonConfig(StoredConfig &target) {
  target.button_hold_ms = kButtonHoldDefaultMs;
  target.button_debounce_ms = kButtonDebounceDefaultMs;
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    target.button_press_action[i] = kButtonActionRelayToggle;
    target.button_hold_action[i] = kButtonActionNone;
  }
  setDefaultButtonActionTexts(target);
  setDefaultButtonRelayConfig(target);
  setDefaultInputConfig(target);
}

void setDefaultButtonConfig() {
  setDefaultButtonConfig(config);
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

bool isInputModeEncoding(uint8_t value) {
  return value == kInputModeUnset || value == kInputModeButton || value == kInputModeSwitch;
}

bool isInputOnLevelEncoding(uint8_t value) {
  return value == kInputOnLevelUnset || value == kInputOnLevelLow || value == kInputOnLevelHigh;
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
  setDefaultLightConfig();
  setDefaultRelayEnforcementConfig();
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

void clearTemplateConfig(StoredConfig &target) {
  target.template_enabled = 0;
  target.template_base = 0;
  target.template_flag = 0;
  memset(target.template_name, 0, sizeof(target.template_name));
  memset(target.template_gpio, 0, sizeof(target.template_gpio));
  setDefaultLedConfig(target);
  setDefaultButtonConfig(target);
  setDefaultLightConfig(target);
  setDefaultRelayEnforcementConfig(target);
}

void clearTemplateConfig() {
  clearTemplateConfig(config);
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
  config.light_power = config.light_power ? 1 : 0;
  if (config.light_dimmer > kLightDimmerMax) {
    config.light_dimmer = kLightDimmerDefault;
  }
  if (config.light_on_dimmer < kLightDimmerMin || config.light_on_dimmer > kLightDimmerMax) {
    config.light_on_dimmer = kLightPowerOnDimmerDefault;
  }
  if (!config.light_power) {
    config.light_dimmer = kLightDimmerOff;
  } else if (config.light_dimmer < kLightDimmerMin) {
    config.light_dimmer = config.light_on_dimmer;
  }
  if (config.light_ct < kLightCtMin || config.light_ct > kLightCtMax) {
    config.light_ct = kLightCtDefault;
  }
  if (config.shelly_dimmer_edge > kShellyDimmerEdgeLeading) {
    config.shelly_dimmer_edge = kShellyDimmerEdgeDefault;
  }
  if (config.shelly_dimmer_range_max == 0 ||
      config.shelly_dimmer_range_min >= config.shelly_dimmer_range_max) {
    config.shelly_dimmer_range_min = kShellyDimmerRangeMinDefault;
    config.shelly_dimmer_range_max = kShellyDimmerRangeMaxDefault;
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
    if (config.button_press_relay[i] != kButtonRelayUnset && config.button_press_relay[i] >= kMaxRelays) {
      config.button_press_relay[i] = kButtonRelayUnset;
    }
    if (config.button_hold_relay[i] != kButtonRelayUnset && config.button_hold_relay[i] >= kMaxRelays) {
      config.button_hold_relay[i] = kButtonRelayUnset;
    }
    if (!isInputModeEncoding(config.input_mode[i])) {
      config.input_mode[i] = kInputModeUnset;
    }
    if (config.input_relay[i] >= kMaxRelays) {
      config.input_relay[i] = i;
    }
    if (!isInputOnLevelEncoding(config.input_on_level[i])) {
      config.input_on_level[i] = kInputOnLevelUnset;
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
  memset(config.reserved, 0, sizeof(config.reserved));
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    config.relay_on_boot[i] = config.relay_on_boot[i] ? 1 : 0;
    config.relay_time_enabled[i] = config.relay_time_enabled[i] ? 1 : 0;
    if (config.relay_time_enabled[i] &&
        (config.relay_time_seconds[i] < kRelayEnforcementMinSeconds ||
         config.relay_time_seconds[i] > kRelayEnforcementMaxSeconds)) {
      config.relay_time_enabled[i] = 0;
      config.relay_time_seconds[i] = 0;
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
bool saveRelayEnforcementConfig(const uint8_t *on_boot, const uint8_t *time_enabled, const uint16_t *time_seconds);
bool saveButtonConfig(uint16_t hold_ms, uint16_t debounce_ms,
                      const uint8_t *press_actions, const uint8_t *hold_actions,
                      const char press_targets[][kButtonActionTargetMaxLen + 1],
                      const char press_payloads[][kButtonActionPayloadMaxLen + 1],
                      const char hold_targets[][kButtonActionTargetMaxLen + 1],
                      const char hold_payloads[][kButtonActionPayloadMaxLen + 1],
                      const uint8_t *input_modes,
                      const uint8_t *input_relays,
                      const uint8_t *input_on_levels,
                      const uint8_t *press_relays,
                      const uint8_t *hold_relays);

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

  if (header.version == kConfigVersionV14 && header.size == sizeof(StoredConfigV14)) {
    StoredConfigV14 *old_config = new StoredConfigV14;
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
    memset(&config, 0, sizeof(config));
    memcpy(&config, old_config, offsetof(StoredConfigV14, crc));
    config.shelly_dimmer_edge = kShellyDimmerEdgeDefault;
    config.shelly_dimmer_range_min = kShellyDimmerRangeMinDefault;
    config.shelly_dimmer_range_max = kShellyDimmerRangeMaxDefault;
    setDefaultRelayEnforcementConfig();
    delete old_config;
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV13 && header.size == sizeof(StoredConfigV13)) {
    StoredConfigV13 *old_config = new StoredConfigV13;
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
    memset(&config, 0, sizeof(config));
    memcpy(&config, old_config, offsetof(StoredConfigV13, crc));
    config.light_on_dimmer = config.light_dimmer;
    config.shelly_dimmer_edge = kShellyDimmerEdgeDefault;
    config.shelly_dimmer_range_min = kShellyDimmerRangeMinDefault;
    config.shelly_dimmer_range_max = kShellyDimmerRangeMaxDefault;
    setDefaultRelayEnforcementConfig();
    delete old_config;
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV12 && header.size == sizeof(StoredConfigV12)) {
    StoredConfigV12 *old_config = new StoredConfigV12;
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
    memset(&config, 0, sizeof(config));
    memcpy(&config, old_config, offsetof(StoredConfigV12, crc));
    setDefaultLightConfig();
    setDefaultRelayEnforcementConfig();
    delete old_config;
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV11 && header.size == sizeof(StoredConfigV11)) {
    StoredConfigV11 *old_config = new StoredConfigV11;
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
    memset(&config, 0, sizeof(config));
    memcpy(&config, old_config, offsetof(StoredConfigV11, crc));
    setDefaultButtonRelayConfig();
    setDefaultLightConfig();
    setDefaultRelayEnforcementConfig();
    delete old_config;
    commitConfig();
    return config_ok;
  }

  if (header.version == kConfigVersionV10 && header.size == sizeof(StoredConfigV10)) {
    StoredConfigV10 *old_config = new StoredConfigV10;
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
    memset(&config, 0, sizeof(config));
    memcpy(&config, old_config, offsetof(StoredConfigV10, crc));
    setDefaultButtonRelayConfig();
    setDefaultInputConfig();
    setDefaultLightConfig();
    setDefaultRelayEnforcementConfig();
    delete old_config;
    commitConfig();
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
    setDefaultButtonRelayConfig();
    setDefaultInputConfig();
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
    setDefaultButtonRelayConfig();
    setDefaultInputConfig();
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
    setDefaultButtonRelayConfig();
    setDefaultInputConfig();
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

void resetMqttRuntimeState() {
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
  last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
  mqtt_pending_relay_mask = 0;
  mqtt_pending_light_mask = 0;
  mqtt_button_queue_head = 0;
  mqtt_button_queue_count = 0;
  mqtt_ping_pending = false;
}

bool saveMqttConfig(const char *host, uint16_t port, const char *topic, uint16_t keepalive) {
  strlcpy(config.mqtt_host, host ? host : "", sizeof(config.mqtt_host));
  config.mqtt_port = port;
  strlcpy(config.mqtt_topic, topic ? topic : "", sizeof(config.mqtt_topic));
  config.mqtt_keepalive = keepalive;
  resetMqttRuntimeState();
  return commitConfig();
}

bool saveEnergyConfig(float total_offset_kwh, uint16_t mqtt_interval, uint16_t mqtt_change_percent_x10) {
  config.energy_total_offset_kwh = total_offset_kwh;
  config.energy_mqtt_interval = mqtt_interval;
  config.energy_mqtt_change_percent_x10 = mqtt_change_percent_x10;
  last_mqtt_energy_publish = 0;
  last_mqtt_energy_power = NAN;
  last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
  return commitConfig();
}

bool saveLedConfig(const uint8_t *attachments) {
  memcpy(config.led_attach, attachments, sizeof(config.led_attach));
  return commitConfig();
}

bool saveRelayEnforcementConfig(const uint8_t *on_boot, const uint8_t *time_enabled, const uint16_t *time_seconds) {
  memcpy(config.relay_on_boot, on_boot, sizeof(config.relay_on_boot));
  memcpy(config.relay_time_enabled, time_enabled, sizeof(config.relay_time_enabled));
  memcpy(config.relay_time_seconds, time_seconds, sizeof(config.relay_time_seconds));
  if (!commitConfig()) return false;
  refreshRelayEnforcementRuntime(true);
  return true;
}

bool saveButtonConfig(uint16_t hold_ms, uint16_t debounce_ms,
                      const uint8_t *press_actions, const uint8_t *hold_actions,
                      const char press_targets[][kButtonActionTargetMaxLen + 1],
                      const char press_payloads[][kButtonActionPayloadMaxLen + 1],
                      const char hold_targets[][kButtonActionTargetMaxLen + 1],
                      const char hold_payloads[][kButtonActionPayloadMaxLen + 1],
                      const uint8_t *input_modes,
                      const uint8_t *input_relays,
                      const uint8_t *input_on_levels,
                      const uint8_t *press_relays,
                      const uint8_t *hold_relays) {
  config.button_hold_ms = hold_ms;
  config.button_debounce_ms = debounce_ms;
  memcpy(config.button_press_action, press_actions, sizeof(config.button_press_action));
  memcpy(config.button_hold_action, hold_actions, sizeof(config.button_hold_action));
  memcpy(config.button_press_target, press_targets, sizeof(config.button_press_target));
  memcpy(config.button_press_payload, press_payloads, sizeof(config.button_press_payload));
  memcpy(config.button_hold_target, hold_targets, sizeof(config.button_hold_target));
  memcpy(config.button_hold_payload, hold_payloads, sizeof(config.button_hold_payload));
  memcpy(config.input_mode, input_modes, sizeof(config.input_mode));
  memcpy(config.input_relay, input_relays, sizeof(config.input_relay));
  memcpy(config.input_on_level, input_on_levels, sizeof(config.input_on_level));
  memcpy(config.button_press_relay, press_relays, sizeof(config.button_press_relay));
  memcpy(config.button_hold_relay, hold_relays, sizeof(config.button_hold_relay));
  return commitConfig();
}

float reportedEnergyTotalKwh() {
  return energy.total_kwh + config.energy_total_offset_kwh;
}

float energyMqttChangePercent() {
  return static_cast<float>(config.energy_mqtt_change_percent_x10) / 10.0f;
}

const __FlashStringHelper *shellyDimmerEdgeName(uint8_t edge) {
  switch (edge) {
    case kShellyDimmerEdgeTrailing: return F("trailing");
    case kShellyDimmerEdgeLeading: return F("leading");
    default: return F("auto");
  }
}

bool parseShellyDimmerEdgeName(String name, uint8_t &edge) {
  name.trim();
  name.toLowerCase();
  if (name == F("auto") || name == F("default") || name == F("firmware")) {
    edge = kShellyDimmerEdgeAuto;
  } else if (name == F("trailing") || name == F("trailing_edge")) {
    edge = kShellyDimmerEdgeTrailing;
  } else if (name == F("leading") || name == F("leading_edge")) {
    edge = kShellyDimmerEdgeLeading;
  } else {
    return false;
  }
  return true;
}

uint16_t shellyDimmerSettingsEdgePayload() {
  if (config.shelly_dimmer_edge == kShellyDimmerEdgeTrailing) return 2;
  if (config.shelly_dimmer_edge == kShellyDimmerEdgeLeading) return 1;
  return 0;
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

bool readButtonRelayTargetInput(uint8_t button, const char *prefix, uint8_t action,
                                uint8_t relays[], String &error) {
  if (button >= kMaxButtons) return false;

  String relay_arg = prefix;
  relay_arg += F("_relay");
  relay_arg += String(button);

  if (action != kButtonActionRelayToggle) {
    if (server.hasArg(relay_arg)) {
      uint16_t relay_value = 0;
      if (parseUint16Input(server.arg(relay_arg), 0, kMaxRelays - 1, relay_value)) {
        relays[button] = static_cast<uint8_t>(relay_value);
      }
    }
    return true;
  }

  uint8_t default_relay = 0;
  if (!defaultButtonRelayTarget(button, default_relay) && lightAvailableIn(runtime_template)) {
    relays[button] = kButtonRelayUnset;
    return true;
  }

  if (!server.hasArg(relay_arg)) {
    error = F("Missing relay target");
    return false;
  }

  uint16_t relay_value = 0;
  if (!parseUint16Input(server.arg(relay_arg), 0, kMaxRelays - 1, relay_value)) {
    error = F("Invalid relay target");
    return false;
  }

  const uint8_t relay = static_cast<uint8_t>(relay_value);
  if (!relayAvailable(relay)) {
    error = F("Invalid relay target");
    return false;
  }

  relays[button] = relay;
  return true;
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

bool isSwitchInput(uint8_t input) {
  return input < kMaxButtons && runtime_template.input_kind[input] == kInputKindSwitch;
}

uint8_t defaultInputMode(uint8_t input) {
  if (runtime_template.shelly_dimmer && isSwitchInput(input)) return kInputModeButton;
  return isSwitchInput(input) ? kInputModeSwitch : kInputModeButton;
}

uint8_t effectiveInputMode(uint8_t input) {
  if (input >= kMaxButtons) return kInputModeButton;
  if (config.input_mode[input] == kInputModeButton || config.input_mode[input] == kInputModeSwitch) {
    return config.input_mode[input];
  }
  return defaultInputMode(input);
}

uint8_t defaultInputOnLevel(uint8_t input) {
  if (input >= kMaxButtons || !hasPin(runtime_template.buttons[input])) return kInputOnLevelLow;
  if (isSwitchInput(input)) return kInputOnLevelHigh;
  return runtime_template.buttons[input].inverted ? kInputOnLevelHigh : kInputOnLevelLow;
}

uint8_t effectiveInputOnLevel(uint8_t input) {
  if (input >= kMaxButtons) return kInputOnLevelLow;
  if (config.input_on_level[input] == kInputOnLevelLow || config.input_on_level[input] == kInputOnLevelHigh) {
    return config.input_on_level[input];
  }
  return defaultInputOnLevel(input);
}

bool readInputActive(uint8_t input) {
  if (input >= kMaxButtons || !digitalPinSupported(runtime_template.buttons[input].pin)) return false;
  const bool high = digitalRead(runtime_template.buttons[input].pin) == HIGH;
  return high == (effectiveInputOnLevel(input) == kInputOnLevelHigh);
}

String inputKindName(uint8_t input) {
  return isSwitchInput(input) ? F("switch") : F("button");
}

String inputModeName(uint8_t input) {
  return effectiveInputMode(input) == kInputModeSwitch ? F("switch follow") : F("button actions");
}

String inputDisplayName(uint8_t input) {
  String name = isSwitchInput(input) ? F("Switch ") : F("Button ");
  name += String(input + 1);
  return name;
}

String inputStateName(uint8_t input, bool active) {
  if (effectiveInputMode(input) == kInputModeSwitch) {
    return active ? F("on") : F("off");
  }
  return active ? F("pressed") : F("released");
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
    return String(F("input")) + String(index + 1);
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

bool hasConfigurableRelays() {
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (relayAvailable(i)) return true;
  }
  return false;
}

String buttonActionName(uint8_t action) {
  if (action == kButtonActionRelayToggle) return F("relay toggle");
  if (action == kButtonActionMqtt) return F("mqtt broadcast");
  if (action == kButtonActionWebhook) return F("webhook exec");
  return F("nothing");
}

bool relayAvailable(uint8_t relay) {
  return relay < runtime_template.relay_count && hasPin(runtime_template.relays[relay]);
}

bool lightAvailableIn(const RuntimeTemplate &rt) {
  if (rt.shelly_dimmer) return true;
  for (uint8_t i = 0; i < rt.pwm_count && i < kMaxLightPwms; i++) {
    if (hasPin(rt.light_pwm[i])) return true;
  }
  return false;
}

bool lightSupportsColorTemperatureIn(const RuntimeTemplate &rt) {
  if (rt.shelly_dimmer || rt.pwm_count < 2) return false;
  return hasPin(rt.light_pwm[0]) && hasPin(rt.light_pwm[1]);
}

bool lightSupportsColorTemperature() {
  return lightSupportsColorTemperatureIn(runtime_template);
}

bool defaultButtonRelayTarget(uint8_t button, uint8_t &relay) {
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

uint8_t configuredButtonRelayTarget(uint8_t button, bool hold) {
  if (button >= kMaxButtons) return kButtonRelayUnset;
  return hold ? config.button_hold_relay[button] : config.button_press_relay[button];
}

bool buttonRelayTarget(uint8_t button, bool hold, uint8_t &relay) {
  const uint8_t configured = configuredButtonRelayTarget(button, hold);
  if (relayAvailable(configured)) {
    relay = configured;
    return true;
  }
  return defaultButtonRelayTarget(button, relay);
}

bool inputRelayTarget(uint8_t input, uint8_t &relay) {
  if (input >= kMaxButtons) return false;
  const uint8_t configured = config.input_relay[input];
  if (relayAvailable(configured)) {
    relay = configured;
    return true;
  }
  return defaultButtonRelayTarget(input, relay);
}

bool inputCanFollowOutput(uint8_t input) {
  uint8_t relay = 0;
  return defaultButtonRelayTarget(input, relay) || lightAvailableIn(runtime_template);
}

bool buttonActionAvailable(uint8_t button, uint8_t action) {
  if (action == kButtonActionNone) return true;
  if (action == kButtonActionRelayToggle) {
    uint8_t relay = 0;
    return defaultButtonRelayTarget(button, relay) || lightAvailableIn(runtime_template);
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

void addUnsupportedTemplatePin(RuntimeTemplate &target, uint8_t pin, uint16_t code) {
  if (target.unsupported_count >= sizeof(target.unsupported_code) / sizeof(target.unsupported_code[0])) {
    return;
  }
  const uint8_t index = target.unsupported_count++;
  target.unsupported_pin[index] = pin;
  target.unsupported_code[index] = code;
}

void addUnsupportedTemplatePin(uint8_t pin, uint16_t code) {
  addUnsupportedTemplatePin(runtime_template, pin, code);
}

void parseTemplateFunction(RuntimeTemplate &target, uint8_t pin, uint16_t code) {
  if (code == kTplNone || code == kTplUser || code == 65504U) {
    return;
  }

  const uint16_t base = code & 0xffe0U;
  const uint8_t index = code & 0x1fU;
  if (pin == kAdc0Pin) {
    if (code == kTplAdcTemp) {
      target.adc_temp = true;
    } else {
      addUnsupportedTemplatePin(target, pin, code);
    }
    return;
  }

  if (!digitalPinSupported(pin)) {
    addUnsupportedTemplatePin(target, pin, code);
    return;
  }

  if (base == kTplKey1 || base == kTplKey1Np || base == kTplKey1Inv || base == kTplKey1InvNp) {
    if (index >= kMaxButtons) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    target.buttons[index] = {
      pin,
      base == kTplKey1Inv || base == kTplKey1InvNp,
      base == kTplKey1Np || base == kTplKey1InvNp
    };
    target.input_kind[index] = kInputKindButton;
    if (target.button_count <= index) target.button_count = index + 1;
    return;
  }

  if (base == kTplSwt1 || base == kTplSwt1Np) {
    if (index >= kMaxButtons) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    target.buttons[index] = {
      pin,
      false,
      base == kTplSwt1Np
    };
    target.input_kind[index] = kInputKindSwitch;
    if (target.button_count <= index) target.button_count = index + 1;
    return;
  }

  if (base == kTplRel1 || base == kTplRel1Inv) {
    if (index >= kMaxRelays) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    target.relays[index] = {pin, base == kTplRel1Inv, false};
    if (target.relay_count <= index) target.relay_count = index + 1;
    return;
  }

  if (base == kTplPwm1 || base == kTplPwm1Inv) {
    if (index >= kMaxLightPwms) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    target.light_pwm[index] = {pin, base == kTplPwm1Inv, false};
    if (target.pwm_count <= index) target.pwm_count = index + 1;
    return;
  }

  if (base == kTplLed1 || base == kTplLed1Inv) {
    if (index >= kMaxLeds) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    target.leds[index] = {pin, base == kTplLed1Inv, false};
    if (target.led_count <= index) target.led_count = index + 1;
    return;
  }

  if (code == kTplLedLnk || code == kTplLedLnkInv) {
    target.link_led = {pin, code == kTplLedLnkInv, false};
    return;
  }

  if (code == kTplI2cScl) {
    target.i2c_scl_pin = pin;
    return;
  }

  if (code == kTplI2cSda) {
    target.i2c_sda_pin = pin;
    return;
  }

  if (code == kTplSerialTxd) {
    target.serial_tx_pin = pin;
    return;
  }

  if (code == kTplSerialRxd) {
    target.serial_rx_pin = pin;
    return;
  }

  if (code == kTplShellyDimmerBoot0) {
    target.shelly_dimmer_boot0_pin = pin;
    return;
  }

  if (code == kTplShellyDimmerResetInv) {
    target.shelly_dimmer_reset_pin = pin;
    return;
  }

  if (code == kTplNrgSel || code == kTplNrgSelInv) {
    target.energy_sel_pin = pin;
    target.energy_sel_inverted = code == kTplNrgSelInv;
    return;
  }

  if (code == kTplNrgCf1) {
    if (!interruptPinSupported(pin)) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    target.energy_cf1_pin = pin;
    return;
  }

  if (code == kTplHlwCf || code == kTplHjlCf) {
    if (!interruptPinSupported(pin)) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    target.energy_cf_pin = pin;
    target.energy_hjl = code == kTplHjlCf;
    return;
  }

  if (base == kTplRot1A || base == kTplRot1B) {
    if (index >= kMaxRotaries || !interruptPinSupported(pin)) {
      addUnsupportedTemplatePin(target, pin, code);
      return;
    }
    if (base == kTplRot1A) {
      target.rotary_a[index] = {pin, false, false};
    } else {
      target.rotary_b[index] = {pin, false, false};
    }
    if (target.rotary_count <= index) target.rotary_count = index + 1;
    return;
  }

  if (base == kTplAde7953Irq) {
    target.ade7953_irq_pin = pin;
    target.ade7953_model = index;
    if (index != kAde7953ModelShelly25) {
      addUnsupportedTemplatePin(target, pin, code);
    }
    return;
  }

  addUnsupportedTemplatePin(target, pin, code);
}

void parseTemplateFunction(uint8_t pin, uint16_t code) {
  parseTemplateFunction(runtime_template, pin, code);
}

void detachEnergyInterrupts() {
  if (!energy.present) return;
  if (energy.driver == kEnergyDriverPulse && interruptPinSupported(runtime_template.energy_cf_pin)) {
    detachInterrupt(digitalPinToInterrupt(runtime_template.energy_cf_pin));
  }
  if (energy.driver == kEnergyDriverPulse &&
      runtime_template.energy_cf1_pin != runtime_template.energy_cf_pin &&
      interruptPinSupported(runtime_template.energy_cf1_pin)) {
    detachInterrupt(digitalPinToInterrupt(runtime_template.energy_cf1_pin));
  }
  energy.present = false;
  energy.driver = kEnergyDriverNone;
}

void resetRuntimeTemplate(RuntimeTemplate &target) {
  memset(&target, 0, sizeof(target));
  for (uint8_t i = 0; i < kMaxRelays; i++) resetPinAssignment(target.relays[i]);
  for (uint8_t i = 0; i < kMaxButtons; i++) resetPinAssignment(target.buttons[i]);
  for (uint8_t i = 0; i < kMaxLeds; i++) resetPinAssignment(target.leds[i]);
  for (uint8_t i = 0; i < kMaxLightPwms; i++) resetPinAssignment(target.light_pwm[i]);
  for (uint8_t i = 0; i < kMaxRotaries; i++) {
    resetPinAssignment(target.rotary_a[i]);
    resetPinAssignment(target.rotary_b[i]);
  }
  resetPinAssignment(target.link_led);
  target.i2c_scl_pin = kInvalidPin;
  target.i2c_sda_pin = kInvalidPin;
  target.ade7953_irq_pin = kInvalidPin;
  target.energy_cf_pin = kInvalidPin;
  target.energy_cf1_pin = kInvalidPin;
  target.energy_sel_pin = kInvalidPin;
  target.serial_tx_pin = kInvalidPin;
  target.serial_rx_pin = kInvalidPin;
  target.shelly_dimmer_boot0_pin = kInvalidPin;
  target.shelly_dimmer_reset_pin = kInvalidPin;
}

void finalizeRuntimeTemplate(RuntimeTemplate &target) {
  const bool has_shelly_serial = target.serial_tx_pin == 1 && target.serial_rx_pin == 3;
  const bool has_shelly_control =
    digitalPinSupported(target.shelly_dimmer_boot0_pin) &&
    digitalPinSupported(target.shelly_dimmer_reset_pin);
  target.shelly_dimmer = has_shelly_serial && has_shelly_control;

  if (!target.shelly_dimmer) {
    if (digitalPinSupported(target.serial_tx_pin)) {
      addUnsupportedTemplatePin(target, target.serial_tx_pin, kTplSerialTxd);
    }
    if (digitalPinSupported(target.serial_rx_pin)) {
      addUnsupportedTemplatePin(target, target.serial_rx_pin, kTplSerialRxd);
    }
    if (digitalPinSupported(target.shelly_dimmer_boot0_pin)) {
      addUnsupportedTemplatePin(target, target.shelly_dimmer_boot0_pin, kTplShellyDimmerBoot0);
    }
    if (digitalPinSupported(target.shelly_dimmer_reset_pin)) {
      addUnsupportedTemplatePin(target, target.shelly_dimmer_reset_pin, kTplShellyDimmerResetInv);
    }
  }
}

void decodeTemplateConfigInto(const StoredConfig &source, RuntimeTemplate &target) {
  resetRuntimeTemplate(target);
  if (!source.template_enabled) return;

  target.enabled = true;
  strlcpy(target.name, source.template_name, sizeof(target.name));
  target.base = source.template_base;
  target.flag = source.template_flag;
  for (uint8_t i = 0; i < kTemplateSlotCount; i++) {
    parseTemplateFunction(target, kTemplateSlotToPin[i], source.template_gpio[i]);
  }
  finalizeRuntimeTemplate(target);
}

void decodeTemplateConfig() {
  detachEnergyInterrupts();
  decodeTemplateConfigInto(config, runtime_template);
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
  if (relay_number == 1 && runtime_template.relay_count <= 1) {
    strlcpy(response_key, "POWER", key_size);
    return true;
  }
  if (snprintf(response_key, key_size, "POWER%u", static_cast<unsigned>(relay_number)) >= static_cast<int>(key_size)) {
    return false;
  }
  return true;
}

bool parsePowerState(const char *p, size_t len, uint8_t &state) {
  if (len == 2 && (p[0] | 0x20) == 'o' && (p[1] | 0x20) == 'n') {
    state = kPowerStateOn;
    return true;
  }
  if (len == 3 && (p[0] | 0x20) == 'o' && (p[1] | 0x20) == 'f' && (p[2] | 0x20) == 'f') {
    state = kPowerStateOff;
    return true;
  }
  if (len == 6 &&
      (p[0] | 0x20) == 't' &&
      (p[1] | 0x20) == 'o' &&
      (p[2] | 0x20) == 'g' &&
      (p[3] | 0x20) == 'g' &&
      (p[4] | 0x20) == 'l' &&
      (p[5] | 0x20) == 'e') {
    state = kPowerStateToggle;
    return true;
  }
  return false;
}

bool commandEquals(const char *p, size_t len, const char *name) {
  return len == strlen(name) && strncasecmp(p, name, len) == 0;
}

String commandArgument(const char *p, size_t len) {
  String out;
  out.reserve(len);
  for (size_t i = 0; i < len; i++) {
    out += p[i];
  }
  return out;
}

bool parseDimmerRangeCommandArgument(const char *p, size_t len, uint8_t &range_min, uint8_t &range_max) {
  String value = commandArgument(p, len);
  value.trim();
  if (value.length() == 0) return false;

  int split = value.indexOf(',');
  if (split < 0) split = value.indexOf(' ');
  String min_text = split < 0 ? value : value.substring(0, split);
  String max_text = split < 0 ? String(config.shelly_dimmer_range_max) : value.substring(split + 1);
  min_text.trim();
  max_text.trim();

  uint16_t parsed_min = 0;
  uint16_t parsed_max = 0;
  if (!parseUint16Input(min_text, 0, kShellyDimmerRangeMaxLimit, parsed_min) ||
      !parseUint16Input(max_text, 0, kShellyDimmerRangeMaxLimit, parsed_max)) {
    return false;
  }
  if (parsed_min > parsed_max) {
    const uint16_t swapped = parsed_min;
    parsed_min = parsed_max;
    parsed_max = swapped;
  }
  if (parsed_min == parsed_max || parsed_max == 0) return false;
  range_min = static_cast<uint8_t>(parsed_min);
  range_max = static_cast<uint8_t>(parsed_max);
  return true;
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

// Settings export/import is an explicit schema. When adding persisted config
// fields, update these helpers so backups include the new setting safely.
struct SettingsImportStats {
  uint16_t applied;
  uint16_t skipped;
  String skipped_fields;
};

void recordSettingsApplied(SettingsImportStats &stats) {
  stats.applied++;
}

void recordSettingsSkipped(SettingsImportStats &stats, const String &field) {
  stats.skipped++;
  if (stats.skipped_fields.length() >= 240) return;
  if (stats.skipped_fields.length()) stats.skipped_fields += F(", ");
  stats.skipped_fields += field;
}

String settingsJsonEscape(const char *input) {
  String out;
  if (!input) return out;
  out.reserve(strlen(input) + 8);
  for (const char *p = input; *p; p++) {
    const char c = *p;
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += F("\\n");
    } else if (c == '\r') {
      out += F("\\r");
    } else if (c == '\t') {
      out += F("\\t");
    } else if (static_cast<uint8_t>(c) < 0x20 || c == 0x7f) {
      out += ' ';
    } else {
      out += c;
    }
  }
  return out;
}

bool settingsReadString(JsonVariantConst value, String &out, size_t max_len, bool trim_value = true) {
  if (!value.is<const char *>()) return false;
  out = value.as<const char *>();
  if (trim_value) out.trim();
  return out.length() <= max_len;
}

bool settingsReadUint16(JsonVariantConst value, uint16_t min_value, uint16_t max_value, uint16_t &out) {
  if (!value.is<long>() && !value.is<unsigned long>() && !value.is<int>() && !value.is<unsigned int>()) return false;
  const long raw = value.as<long>();
  if (raw < static_cast<long>(min_value) || raw > static_cast<long>(max_value)) return false;
  out = static_cast<uint16_t>(raw);
  return true;
}

bool settingsReadFloat(JsonVariantConst value, float min_value, float max_value, float &out) {
  if (!value.is<float>() && !value.is<double>() && !value.is<long>() && !value.is<unsigned long>() &&
      !value.is<int>() && !value.is<unsigned int>()) {
    return false;
  }
  const float raw = value.as<float>();
  if (isnan(raw) || raw < min_value || raw > max_value) return false;
  out = raw;
  return true;
}

const __FlashStringHelper *settingsActionName(uint8_t action) {
  switch (action) {
    case kButtonActionRelayToggle: return F("relay_toggle");
    case kButtonActionMqtt: return F("mqtt");
    case kButtonActionWebhook: return F("webhook");
    default: return F("none");
  }
}

bool parseSettingsActionName(const String &name, uint8_t &action) {
  if (name == F("none") || name == F("nothing")) {
    action = kButtonActionNone;
  } else if (name == F("relay_toggle") || name == F("relay toggle")) {
    action = kButtonActionRelayToggle;
  } else if (name == F("mqtt") || name == F("mqtt broadcast")) {
    action = kButtonActionMqtt;
  } else if (name == F("webhook") || name == F("webhook exec")) {
    action = kButtonActionWebhook;
  } else {
    return false;
  }
  return true;
}

const __FlashStringHelper *settingsInputModeName(uint8_t mode) {
  return mode == kInputModeSwitch ? F("switch") : F("button");
}

bool parseSettingsInputMode(const String &name, uint8_t &mode) {
  if (name == F("button") || name == F("button actions")) {
    mode = kInputModeButton;
  } else if (name == F("switch") || name == F("switch follow") || name == F("switch follows relay")) {
    mode = kInputModeSwitch;
  } else {
    return false;
  }
  return true;
}

String settingsLedAttachmentName(uint8_t value) {
  uint8_t index = 0;
  if (ledAttachmentRelayIndex(value, index)) {
    return String(F("relay")) + String(index + 1);
  }
  if (ledAttachmentButtonIndex(value, index)) {
    return String(F("input")) + String(index + 1);
  }
  return F("none");
}

bool parseSettingsLedAttachment(const String &name, uint8_t &value) {
  if (name == F("none") || name == F("nothing")) {
    value = kLedAttachNone;
    return true;
  }
  if (name.startsWith(F("relay"))) {
    uint16_t index = 0;
    if (!parseUint16Input(name.substring(5), 1, kMaxRelays, index)) return false;
    value = kLedAttachRelayBase + static_cast<uint8_t>(index - 1);
    return true;
  }
  if (name.startsWith(F("input"))) {
    uint16_t index = 0;
    if (!parseUint16Input(name.substring(5), 1, kMaxButtons, index)) return false;
    value = kLedAttachButtonBase + static_cast<uint8_t>(index - 1);
    return true;
  }
  return false;
}

bool relayAvailableIn(const RuntimeTemplate &rt, uint8_t relay) {
  return relay < rt.relay_count && hasPin(rt.relays[relay]);
}

bool buttonAvailableIn(const RuntimeTemplate &rt, uint8_t button) {
  return button < rt.button_count && hasPin(rt.buttons[button]);
}

const PinAssignment *ledOutputAssignmentIn(const RuntimeTemplate &rt, uint8_t led) {
  if (led < kMaxLeds) return &rt.leds[led];
  if (led == kMaxLeds) return &rt.link_led;
  return nullptr;
}

bool hasLedOutputIn(const RuntimeTemplate &rt, uint8_t led) {
  const PinAssignment *assignment = ledOutputAssignmentIn(rt, led);
  if (!assignment || !hasPin(*assignment)) return false;
  return led >= kMaxLeds || led < rt.led_count;
}

bool ledAttachmentAvailableIn(const RuntimeTemplate &rt, uint8_t value) {
  uint8_t index = 0;
  if (value == kLedAttachNone) return true;
  if (ledAttachmentRelayIndex(value, index)) return relayAvailableIn(rt, index);
  if (ledAttachmentButtonIndex(value, index)) return buttonAvailableIn(rt, index);
  return false;
}

bool defaultButtonRelayTargetIn(const RuntimeTemplate &rt, uint8_t button, uint8_t &relay) {
  if (button < rt.relay_count && hasPin(rt.relays[button])) {
    relay = button;
    return true;
  }
  if (hasPin(rt.relays[0])) {
    relay = 0;
    return true;
  }
  for (uint8_t i = 0; i < rt.relay_count; i++) {
    if (hasPin(rt.relays[i])) {
      relay = i;
      return true;
    }
  }
  return false;
}

bool buttonActionAvailableIn(const RuntimeTemplate &rt, uint8_t button, uint8_t action) {
  if (action == kButtonActionNone || action == kButtonActionMqtt || action == kButtonActionWebhook) return true;
  if (action == kButtonActionRelayToggle) {
    uint8_t relay = 0;
    return defaultButtonRelayTargetIn(rt, button, relay) || lightAvailableIn(rt);
  }
  return false;
}

bool templatesDiffer(const StoredConfig &a, const StoredConfig &b) {
  return a.template_enabled != b.template_enabled ||
         a.template_base != b.template_base ||
         a.template_flag != b.template_flag ||
         strcmp(a.template_name, b.template_name) != 0 ||
         memcmp(a.template_gpio, b.template_gpio, sizeof(a.template_gpio)) != 0;
}

bool mqttConfigDiffers(const StoredConfig &a, const StoredConfig &b) {
  return a.mqtt_port != b.mqtt_port ||
         a.mqtt_keepalive != b.mqtt_keepalive ||
         strcmp(a.mqtt_host, b.mqtt_host) != 0 ||
         strcmp(a.mqtt_topic, b.mqtt_topic) != 0;
}

bool energyConfigDiffers(const StoredConfig &a, const StoredConfig &b) {
  return a.energy_total_offset_kwh != b.energy_total_offset_kwh ||
         a.energy_mqtt_interval != b.energy_mqtt_interval ||
         a.energy_mqtt_change_percent_x10 != b.energy_mqtt_change_percent_x10;
}

bool lightConfigDiffers(const StoredConfig &a, const StoredConfig &b) {
  return a.light_power != b.light_power ||
         a.light_dimmer != b.light_dimmer ||
         a.light_ct != b.light_ct ||
         a.light_on_dimmer != b.light_on_dimmer ||
         a.shelly_dimmer_edge != b.shelly_dimmer_edge ||
         a.shelly_dimmer_range_min != b.shelly_dimmer_range_min ||
         a.shelly_dimmer_range_max != b.shelly_dimmer_range_max;
}

bool ledConfigDiffers(const StoredConfig &a, const StoredConfig &b) {
  return memcmp(a.led_attach, b.led_attach, sizeof(a.led_attach)) != 0;
}

bool relayEnforcementConfigDiffers(const StoredConfig &a, const StoredConfig &b) {
  return memcmp(a.relay_on_boot, b.relay_on_boot, sizeof(a.relay_on_boot)) != 0 ||
         memcmp(a.relay_time_enabled, b.relay_time_enabled, sizeof(a.relay_time_enabled)) != 0 ||
         memcmp(a.relay_time_seconds, b.relay_time_seconds, sizeof(a.relay_time_seconds)) != 0;
}

bool inputConfigDiffers(const StoredConfig &a, const StoredConfig &b) {
  return a.button_hold_ms != b.button_hold_ms ||
         a.button_debounce_ms != b.button_debounce_ms ||
         memcmp(a.button_press_action, b.button_press_action, sizeof(a.button_press_action)) != 0 ||
         memcmp(a.button_hold_action, b.button_hold_action, sizeof(a.button_hold_action)) != 0 ||
         memcmp(a.button_press_target, b.button_press_target, sizeof(a.button_press_target)) != 0 ||
         memcmp(a.button_press_payload, b.button_press_payload, sizeof(a.button_press_payload)) != 0 ||
         memcmp(a.button_hold_target, b.button_hold_target, sizeof(a.button_hold_target)) != 0 ||
         memcmp(a.button_hold_payload, b.button_hold_payload, sizeof(a.button_hold_payload)) != 0 ||
         memcmp(a.input_mode, b.input_mode, sizeof(a.input_mode)) != 0 ||
         memcmp(a.input_relay, b.input_relay, sizeof(a.input_relay)) != 0 ||
         memcmp(a.input_on_level, b.input_on_level, sizeof(a.input_on_level)) != 0 ||
         memcmp(a.button_press_relay, b.button_press_relay, sizeof(a.button_press_relay)) != 0 ||
         memcmp(a.button_hold_relay, b.button_hold_relay, sizeof(a.button_hold_relay)) != 0;
}

void appendSettingsActionJson(String &out, uint8_t button, bool hold) {
  const uint8_t action = hold ? config.button_hold_action[button] : config.button_press_action[button];
  const uint8_t configured_relay = hold ? config.button_hold_relay[button] : config.button_press_relay[button];
  const char *target = hold ? config.button_hold_target[button] : config.button_press_target[button];
  const char *payload = hold ? config.button_hold_payload[button] : config.button_press_payload[button];
  out += F("{\"action\":\"");
  out += settingsActionName(action);
  out += F("\"");
  uint8_t relay = 0;
  if (relayAvailable(configured_relay)) {
    relay = configured_relay;
    out += F(",\"relay\":");
    out += relay + 1;
  } else if (buttonRelayTarget(button, hold, relay)) {
    out += F(",\"relay\":");
    out += relay + 1;
  }
  out += F(",\"target\":\"");
  out += settingsJsonEscape(target);
  out += F("\",\"payload\":\"");
  out += settingsJsonEscape(payload);
  out += F("\"}");
}

void appendSettingsExportJson(String &out) {
  out += F("{\"format\":\"mymota-settings\",\"format_version\":");
  out += kSettingsFormatVersion;
  out += F(",\"firmware\":{\"name\":\"myMota\",\"version\":\"");
  out += F(MYMOTA_VERSION);
  out += F("\",\"target\":\"");
  out += F(MYMOTA_TARGET);
  out += F("\",\"chip\":\"");
  out += chipIdHex();
  out += F("\"},\"template\":{\"enabled\":");
  out += config.template_enabled ? F("true") : F("false");
  if (config.template_enabled) {
    const String tpl = currentTemplateJson();
    out += F(",\"json\":\"");
    out += settingsJsonEscape(tpl.c_str());
    out += F("\"");
  }
  out += F("},\"mqtt\":{\"host\":\"");
  out += settingsJsonEscape(config.mqtt_host);
  out += F("\",\"port\":");
  out += config.mqtt_port;
  out += F(",\"topic\":\"");
  out += settingsJsonEscape(config.mqtt_topic);
  out += F("\",\"keepalive\":");
  out += config.mqtt_keepalive;
  out += F("},\"energy\":{\"total_offset_kwh\":");
  out += String(config.energy_total_offset_kwh, 4);
  out += F(",\"report_interval\":");
  out += config.energy_mqtt_interval;
  out += F(",\"report_change_percent\":");
  out += String(energyMqttChangePercent(), 1);
  out += F("},\"light\":{\"power\":");
  out += config.light_power ? F("true") : F("false");
  out += F(",\"dimmer\":");
  out += config.light_dimmer;
  out += F(",\"ct\":");
  out += config.light_ct;
  out += F(",\"on_dimmer\":");
  out += config.light_on_dimmer;
  if (runtime_template.shelly_dimmer) {
    out += F(",\"shelly_dimmer\":{\"edge\":\"");
    out += shellyDimmerEdgeName(config.shelly_dimmer_edge);
    out += F("\",\"range_min\":");
    out += String(config.shelly_dimmer_range_min);
    out += F(",\"range_max\":");
    out += String(config.shelly_dimmer_range_max);
    out += F("}");
  }
  out += F("},\"leds\":[");
  for (uint8_t i = 0; i < kMaxLedOutputs; i++) {
    if (i) out += ',';
    out += F("{\"attach\":\"");
    out += settingsLedAttachmentName(config.led_attach[i]);
    out += F("\"}");
  }
  out += F("],\"relay_enforcement\":[");
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    if (i) out += ',';
    out += F("{\"on_boot\":");
    out += config.relay_on_boot[i] ? F("true") : F("false");
    out += F(",\"time_based\":");
    out += config.relay_time_enabled[i] ? F("true") : F("false");
    out += F(",\"seconds\":");
    out += config.relay_time_seconds[i];
    out += F("}");
  }
  out += F("],\"inputs\":{\"hold_ms\":");
  out += config.button_hold_ms;
  out += F(",\"debounce_ms\":");
  out += config.button_debounce_ms;
  out += F(",\"items\":[");
  for (uint8_t i = 0; i < kMaxButtons; i++) {
    if (i) out += ',';
    const uint8_t mode = effectiveInputMode(i);
    const uint8_t on_level = effectiveInputOnLevel(i);
    uint8_t relay = 0;
    out += F("{\"mode\":\"");
    out += settingsInputModeName(mode);
    out += F("\",\"on_level\":\"");
    out += on_level == kInputOnLevelHigh ? F("high") : F("low");
    out += F("\"");
    if (inputRelayTarget(i, relay)) {
      out += F(",\"relay\":");
      out += relay + 1;
    }
    out += F(",\"press\":");
    appendSettingsActionJson(out, i, false);
    out += F(",\"hold\":");
    appendSettingsActionJson(out, i, true);
    out += F("}");
  }
  out += F("]}}");
}

bool importSettingsTemplate(JsonObjectConst root, StoredConfig &target, SettingsImportStats &stats) {
  JsonVariantConst template_value = root["template"];
  if (template_value.isNull()) return false;
  JsonObjectConst tpl = template_value.as<JsonObjectConst>();
  if (tpl.isNull()) {
    recordSettingsSkipped(stats, F("template"));
    return false;
  }

  JsonVariantConst enabled_value = tpl["enabled"];
  if (!enabled_value.isNull() && !enabled_value.is<bool>()) {
    recordSettingsSkipped(stats, F("template.enabled"));
    return false;
  }
  const bool enabled = enabled_value.isNull() ? true : enabled_value.as<bool>();
  if (!enabled) {
    clearTemplateConfig(target);
    recordSettingsApplied(stats);
    return true;
  }

  String template_json;
  if (!settingsReadString(tpl["json"], template_json, kTemplateJsonMaxLen)) {
    recordSettingsSkipped(stats, F("template.json"));
    return false;
  }

  StoredConfig candidate = target;
  String error;
  if (!parseTemplateJson(template_json, candidate, error)) {
    recordSettingsSkipped(stats, F("template.json"));
    return false;
  }
  target = candidate;
  recordSettingsApplied(stats);
  return true;
}

void importSettingsMqtt(JsonObjectConst root, StoredConfig &target, SettingsImportStats &stats) {
  JsonVariantConst mqtt_value = root["mqtt"];
  if (mqtt_value.isNull()) return;
  JsonObjectConst mqtt = mqtt_value.as<JsonObjectConst>();
  if (mqtt.isNull()) {
    recordSettingsSkipped(stats, F("mqtt"));
    return;
  }

  if (mqtt.containsKey("host")) {
    String host;
    if (settingsReadString(mqtt["host"], host, kMqttHostMaxLen) && isValidMqttHost(host)) {
      strlcpy(target.mqtt_host, host.c_str(), sizeof(target.mqtt_host));
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("mqtt.host"));
    }
  }
  if (mqtt.containsKey("port")) {
    uint16_t port = 0;
    if (settingsReadUint16(mqtt["port"], 1, 65535U, port)) {
      target.mqtt_port = port;
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("mqtt.port"));
    }
  }
  if (mqtt.containsKey("topic")) {
    String topic;
    if (settingsReadString(mqtt["topic"], topic, kMqttTopicMaxLen) && isValidMqttTopic(topic)) {
      strlcpy(target.mqtt_topic, topic.c_str(), sizeof(target.mqtt_topic));
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("mqtt.topic"));
    }
  }
  if (mqtt.containsKey("keepalive")) {
    uint16_t keepalive = 0;
    if (settingsReadUint16(mqtt["keepalive"], 0, kMqttKeepaliveMax, keepalive)) {
      target.mqtt_keepalive = keepalive;
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("mqtt.keepalive"));
    }
  }
}

void importSettingsEnergy(JsonObjectConst root, StoredConfig &target, SettingsImportStats &stats) {
  JsonVariantConst energy_value = root["energy"];
  if (energy_value.isNull()) return;
  JsonObjectConst energy_settings = energy_value.as<JsonObjectConst>();
  if (energy_settings.isNull()) {
    recordSettingsSkipped(stats, F("energy"));
    return;
  }

  if (energy_settings.containsKey("total_offset_kwh")) {
    float offset = 0.0f;
    if (settingsReadFloat(energy_settings["total_offset_kwh"], kEnergyTotalOffsetMinKwh, kEnergyTotalOffsetMaxKwh, offset)) {
      target.energy_total_offset_kwh = offset;
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("energy.total_offset_kwh"));
    }
  }
  if (energy_settings.containsKey("report_interval")) {
    uint16_t interval = 0;
    if (settingsReadUint16(energy_settings["report_interval"], 0, kMqttEnergyIntervalMax, interval)) {
      target.energy_mqtt_interval = interval;
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("energy.report_interval"));
    }
  }
  if (energy_settings.containsKey("report_change_percent")) {
    float percent = 0.0f;
    if (settingsReadFloat(energy_settings["report_change_percent"], 0.0f, kMqttEnergyChangeMaxPercent, percent)) {
      target.energy_mqtt_change_percent_x10 = static_cast<uint16_t>((percent * 10.0f) + 0.5f);
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("energy.report_change_percent"));
    }
  }
}

void importSettingsLight(JsonObjectConst root, StoredConfig &target, const RuntimeTemplate &rt, SettingsImportStats &stats) {
  JsonVariantConst light_value = root["light"];
  if (light_value.isNull()) return;
  JsonObjectConst light_settings = light_value.as<JsonObjectConst>();
  if (light_settings.isNull()) {
    recordSettingsSkipped(stats, F("light"));
    return;
  }
  if (!lightAvailableIn(rt)) {
    recordSettingsSkipped(stats, F("light"));
    return;
  }

  if (light_settings.containsKey("power")) {
    JsonVariantConst power_value = light_settings["power"];
    if (power_value.is<bool>()) {
      target.light_power = power_value.as<bool>() ? 1 : 0;
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("light.power"));
    }
  }
  if (light_settings.containsKey("dimmer")) {
    uint16_t dimmer = kLightDimmerDefault;
    if (settingsReadUint16(light_settings["dimmer"], kLightDimmerOff, kLightDimmerMax, dimmer)) {
      target.light_dimmer = static_cast<uint8_t>(dimmer);
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("light.dimmer"));
    }
  }
  if (light_settings.containsKey("ct")) {
    uint16_t ct = kLightCtDefault;
    if (settingsReadUint16(light_settings["ct"], kLightCtMin, kLightCtMax, ct)) {
      target.light_ct = ct;
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("light.ct"));
    }
  }
  if (light_settings.containsKey("on_dimmer")) {
    uint16_t on_dimmer = kLightPowerOnDimmerDefault;
    if (settingsReadUint16(light_settings["on_dimmer"], kLightDimmerMin, kLightDimmerMax, on_dimmer)) {
      target.light_on_dimmer = static_cast<uint8_t>(on_dimmer);
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("light.on_dimmer"));
    }
  }
  if (light_settings.containsKey("shelly_dimmer")) {
    if (!rt.shelly_dimmer) {
      recordSettingsSkipped(stats, F("light.shelly_dimmer"));
      return;
    }
    JsonObjectConst shelly_settings = light_settings["shelly_dimmer"].as<JsonObjectConst>();
    if (shelly_settings.isNull()) {
      recordSettingsSkipped(stats, F("light.shelly_dimmer"));
      return;
    }

    if (shelly_settings.containsKey("edge")) {
      String edge_name;
      uint8_t edge = kShellyDimmerEdgeDefault;
      if (settingsReadString(shelly_settings["edge"], edge_name, 16) &&
          parseShellyDimmerEdgeName(edge_name, edge)) {
        target.shelly_dimmer_edge = edge;
        recordSettingsApplied(stats);
      } else {
        recordSettingsSkipped(stats, F("light.shelly_dimmer.edge"));
      }
    }

    const bool has_range_min = shelly_settings.containsKey("range_min");
    const bool has_range_max = shelly_settings.containsKey("range_max");
    if (has_range_min || has_range_max) {
      uint16_t range_min = target.shelly_dimmer_range_min;
      uint16_t range_max = target.shelly_dimmer_range_max;
      const bool range_min_ok = !has_range_min ||
        settingsReadUint16(shelly_settings["range_min"], 0, kShellyDimmerRangeMaxLimit - 1, range_min);
      const bool range_max_ok = !has_range_max ||
        settingsReadUint16(shelly_settings["range_max"], 1, kShellyDimmerRangeMaxLimit, range_max);
      if (range_min_ok && range_max_ok && range_min < range_max) {
        target.shelly_dimmer_range_min = static_cast<uint8_t>(range_min);
        target.shelly_dimmer_range_max = static_cast<uint8_t>(range_max);
        if (has_range_min) recordSettingsApplied(stats);
        if (has_range_max) recordSettingsApplied(stats);
      } else {
        if (has_range_min) recordSettingsSkipped(stats, F("light.shelly_dimmer.range_min"));
        if (has_range_max) recordSettingsSkipped(stats, F("light.shelly_dimmer.range_max"));
      }
    }
  }
}

void importSettingsLeds(JsonObjectConst root, StoredConfig &target, const RuntimeTemplate &rt, SettingsImportStats &stats) {
  JsonVariantConst leds_value = root["leds"];
  if (leds_value.isNull()) return;
  JsonArrayConst leds = leds_value.as<JsonArrayConst>();
  if (leds.isNull()) {
    recordSettingsSkipped(stats, F("leds"));
    return;
  }

  const uint8_t count = min(static_cast<size_t>(kMaxLedOutputs), leds.size());
  for (uint8_t i = 0; i < count; i++) {
    JsonObjectConst led = leds[i].as<JsonObjectConst>();
    if (led.isNull()) {
      if (!leds[i].isNull()) recordSettingsSkipped(stats, String(F("leds[")) + String(i) + F("]"));
      continue;
    }
    if (!hasLedOutputIn(rt, i)) continue;
    String attach_name;
    uint8_t attachment = kLedAttachNone;
    if (!settingsReadString(led["attach"], attach_name, 16) ||
        !parseSettingsLedAttachment(attach_name, attachment) ||
        !ledAttachmentAvailableIn(rt, attachment)) {
      recordSettingsSkipped(stats, String(F("leds[")) + String(i) + F("].attach"));
      continue;
    }
    target.led_attach[i] = attachment;
    recordSettingsApplied(stats);
  }
}

void importSettingsRelayEnforcement(JsonObjectConst root, StoredConfig &target, const RuntimeTemplate &rt, SettingsImportStats &stats) {
  JsonVariantConst relays_value = root["relay_enforcement"];
  if (relays_value.isNull()) return;
  JsonArrayConst relays = relays_value.as<JsonArrayConst>();
  if (relays.isNull()) {
    recordSettingsSkipped(stats, F("relay_enforcement"));
    return;
  }

  const uint8_t count = min(static_cast<size_t>(kMaxRelays), relays.size());
  for (uint8_t i = 0; i < count; i++) {
    JsonObjectConst relay = relays[i].as<JsonObjectConst>();
    if (relay.isNull()) {
      if (!relays[i].isNull()) recordSettingsSkipped(stats, String(F("relay_enforcement[")) + String(i) + F("]"));
      continue;
    }
    if (!relayAvailableIn(rt, i)) continue;

    if (relay.containsKey("on_boot")) {
      JsonVariantConst on_boot = relay["on_boot"];
      if (on_boot.is<bool>()) {
        target.relay_on_boot[i] = on_boot.as<bool>() ? 1 : 0;
        recordSettingsApplied(stats);
      } else {
        recordSettingsSkipped(stats, String(F("relay_enforcement[")) + String(i) + F("].on_boot"));
      }
    }

    if (relay.containsKey("time_based")) {
      JsonVariantConst time_based = relay["time_based"];
      if (!time_based.is<bool>()) {
        recordSettingsSkipped(stats, String(F("relay_enforcement[")) + String(i) + F("].time_based"));
      } else if (time_based.as<bool>()) {
        uint16_t seconds = 0;
        if (settingsReadUint16(relay["seconds"], kRelayEnforcementMinSeconds, kRelayEnforcementMaxSeconds, seconds)) {
          target.relay_time_enabled[i] = 1;
          target.relay_time_seconds[i] = seconds;
          recordSettingsApplied(stats);
        } else {
          recordSettingsSkipped(stats, String(F("relay_enforcement[")) + String(i) + F("].seconds"));
        }
      } else {
        target.relay_time_enabled[i] = 0;
        recordSettingsApplied(stats);
      }
    }

    if (relay.containsKey("seconds") && !target.relay_time_enabled[i]) {
      uint16_t seconds = 0;
      if (settingsReadUint16(relay["seconds"], kRelayEnforcementMinSeconds, kRelayEnforcementMaxSeconds, seconds)) {
        target.relay_time_seconds[i] = seconds;
        recordSettingsApplied(stats);
      } else if (!relay["seconds"].isNull()) {
        recordSettingsSkipped(stats, String(F("relay_enforcement[")) + String(i) + F("].seconds"));
      }
    }
  }
}

bool importSettingsRelay(JsonVariantConst value, const RuntimeTemplate &rt, uint8_t &relay) {
  uint16_t relay_number = 0;
  if (!settingsReadUint16(value, 1, kMaxRelays, relay_number)) return false;
  const uint8_t parsed = static_cast<uint8_t>(relay_number - 1);
  if (!relayAvailableIn(rt, parsed)) return false;
  relay = parsed;
  return true;
}

void importSettingsAction(JsonVariantConst value, StoredConfig &target, const RuntimeTemplate &rt,
                          uint8_t button, bool hold, SettingsImportStats &stats, const String &field) {
  JsonObjectConst action_object = value.as<JsonObjectConst>();
  if (action_object.isNull() || !buttonAvailableIn(rt, button)) {
    if (!value.isNull()) recordSettingsSkipped(stats, field);
    return;
  }

  String action_name;
  uint8_t action = kButtonActionNone;
  if (!settingsReadString(action_object["action"], action_name, 24) ||
      !parseSettingsActionName(action_name, action) ||
      !buttonActionAvailableIn(rt, button, action)) {
    recordSettingsSkipped(stats, field + F(".action"));
    return;
  }

  uint8_t *actions = hold ? target.button_hold_action : target.button_press_action;
  uint8_t *relays = hold ? target.button_hold_relay : target.button_press_relay;
  char (*targets)[kButtonActionTargetMaxLen + 1] = hold ? target.button_hold_target : target.button_press_target;
  char (*payloads)[kButtonActionPayloadMaxLen + 1] = hold ? target.button_hold_payload : target.button_press_payload;

  uint8_t relay = relays[button];
  if (action_object.containsKey("relay")) {
    if (!importSettingsRelay(action_object["relay"], rt, relay)) {
      recordSettingsSkipped(stats, field + F(".relay"));
      relay = relays[button];
    }
  }

  String target_text = targets[button];
  String payload_text = payloads[button];
  bool text_ok = true;
  if (action == kButtonActionMqtt) {
    if (action_object.containsKey("target")) {
      text_ok = settingsReadString(action_object["target"], target_text, kButtonActionTargetMaxLen) &&
                isValidMqttPublishTopicTemplate(target_text);
    } else if (target_text.length() == 0) {
      target_text = kDefaultButtonMqttTopic;
    }
    if (text_ok) {
      if (action_object.containsKey("payload")) {
        text_ok = settingsReadString(action_object["payload"], payload_text, kButtonActionPayloadMaxLen, false) &&
                  isValidButtonActionText(payload_text, kButtonActionPayloadMaxLen, false, true);
      } else if (payload_text.length() == 0) {
        payload_text = hold ? kDefaultButtonMqttHoldPayload : kDefaultButtonMqttPressPayload;
      }
    }
  } else if (action == kButtonActionWebhook) {
    text_ok = action_object.containsKey("target") &&
              settingsReadString(action_object["target"], target_text, kButtonActionTargetMaxLen) &&
              isValidWebhookUrlTemplate(target_text);
    if (text_ok && action_object.containsKey("payload")) {
      text_ok = settingsReadString(action_object["payload"], payload_text, kButtonActionPayloadMaxLen, false) &&
                isValidButtonActionText(payload_text, kButtonActionPayloadMaxLen, true, true);
    }
  } else {
    if (action_object.containsKey("target")) {
      text_ok = settingsReadString(action_object["target"], target_text, kButtonActionTargetMaxLen) &&
                isValidButtonActionText(target_text, kButtonActionTargetMaxLen, true);
    }
    if (text_ok && action_object.containsKey("payload")) {
      text_ok = settingsReadString(action_object["payload"], payload_text, kButtonActionPayloadMaxLen, false) &&
                isValidButtonActionText(payload_text, kButtonActionPayloadMaxLen, true, true);
    }
  }

  if (!text_ok) {
    recordSettingsSkipped(stats, field + F(".text"));
    return;
  }

  actions[button] = action;
  relays[button] = relay;
  strlcpy(targets[button], target_text.c_str(), kButtonActionTargetMaxLen + 1);
  strlcpy(payloads[button], payload_text.c_str(), kButtonActionPayloadMaxLen + 1);
  recordSettingsApplied(stats);
}

void importSettingsInputs(JsonObjectConst root, StoredConfig &target, const RuntimeTemplate &rt, SettingsImportStats &stats) {
  JsonVariantConst inputs_value = root["inputs"];
  if (inputs_value.isNull()) return;
  JsonObjectConst inputs = inputs_value.as<JsonObjectConst>();
  if (inputs.isNull()) {
    recordSettingsSkipped(stats, F("inputs"));
    return;
  }

  if (inputs.containsKey("hold_ms")) {
    uint16_t hold_ms = kButtonHoldDefaultMs;
    if (settingsReadUint16(inputs["hold_ms"], kButtonHoldMinMs, kButtonHoldMaxMs, hold_ms)) {
      target.button_hold_ms = hold_ms;
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("inputs.hold_ms"));
    }
  }
  if (inputs.containsKey("debounce_ms")) {
    uint16_t debounce_ms = kButtonDebounceDefaultMs;
    if (settingsReadUint16(inputs["debounce_ms"], kButtonDebounceMinMs, kButtonDebounceMaxMs, debounce_ms)) {
      target.button_debounce_ms = debounce_ms;
      recordSettingsApplied(stats);
    } else {
      recordSettingsSkipped(stats, F("inputs.debounce_ms"));
    }
  }

  JsonVariantConst items_value = inputs["items"];
  if (items_value.isNull()) return;
  JsonArrayConst items = items_value.as<JsonArrayConst>();
  if (items.isNull()) {
    recordSettingsSkipped(stats, F("inputs.items"));
    return;
  }
  const uint8_t count = min(static_cast<size_t>(kMaxButtons), items.size());
  for (uint8_t i = 0; i < count; i++) {
    JsonObjectConst item = items[i].as<JsonObjectConst>();
    if (item.isNull()) {
      if (!items[i].isNull()) recordSettingsSkipped(stats, String(F("inputs.items[")) + String(i) + F("]"));
      continue;
    }
    if (!buttonAvailableIn(rt, i)) continue;

    if (item.containsKey("mode")) {
      String mode_name;
      uint8_t mode = kInputModeUnset;
      if (settingsReadString(item["mode"], mode_name, 24) && parseSettingsInputMode(mode_name, mode)) {
        target.input_mode[i] = mode;
        recordSettingsApplied(stats);
        if (mode == kInputModeButton) {
          target.input_relay[i] = i;
          target.input_on_level[i] = kInputOnLevelUnset;
        }
      } else {
        recordSettingsSkipped(stats, String(F("inputs.items[")) + String(i) + F("].mode"));
      }
    }

    if (target.input_mode[i] == kInputModeSwitch) {
      JsonVariantConst relay_value = item.containsKey("relay") ? item["relay"] : item["target_relay"];
      if (!relay_value.isNull()) {
        uint8_t relay = 0;
        if (importSettingsRelay(relay_value, rt, relay)) {
          target.input_relay[i] = relay;
          recordSettingsApplied(stats);
        } else {
          recordSettingsSkipped(stats, String(F("inputs.items[")) + String(i) + F("].relay"));
        }
      }
      if (item.containsKey("on_level")) {
        String on_level;
        if (settingsReadString(item["on_level"], on_level, 8) && (on_level == F("high") || on_level == F("low"))) {
          target.input_on_level[i] = on_level == F("high") ? kInputOnLevelHigh : kInputOnLevelLow;
          recordSettingsApplied(stats);
        } else {
          recordSettingsSkipped(stats, String(F("inputs.items[")) + String(i) + F("].on_level"));
        }
      } else if (item.containsKey("reverse")) {
        JsonVariantConst reverse = item["reverse"];
        if (reverse.is<bool>()) {
          target.input_on_level[i] = reverse.as<bool>() ? kInputOnLevelLow : kInputOnLevelHigh;
          recordSettingsApplied(stats);
        } else {
          recordSettingsSkipped(stats, String(F("inputs.items[")) + String(i) + F("].reverse"));
        }
      }
    }

    importSettingsAction(item["press"], target, rt, i, false, stats, String(F("inputs.items[")) + String(i) + F("].press"));
    importSettingsAction(item["hold"], target, rt, i, true, stats, String(F("inputs.items[")) + String(i) + F("].hold"));
  }
}

void appendApiSettingsJson(String &out) {
  out += F("{\"format\":\"mymota-api-settings\",\"api_version\":");
  out += kApiSettingsVersion;
  out += F(",\"inputs\":[");
  bool first = true;
  for (uint8_t i = 0; i < runtime_template.button_count && i < kMaxButtons; i++) {
    if (!first) out += ',';
    first = false;
    if (!hasPin(runtime_template.buttons[i])) {
      out += F("null");
      continue;
    }
    out += F("{\"input\":");
    out += i + 1;
    out += F(",\"mode\":\"");
    out += settingsInputModeName(effectiveInputMode(i));
    out += F("\",\"press\":{\"action\":\"");
    out += settingsActionName(config.button_press_action[i]);
    out += F("\",\"mqtt_topic\":\"");
    out += settingsJsonEscape(config.button_press_target[i]);
    out += F("\",\"mqtt_payload\":\"");
    out += settingsJsonEscape(config.button_press_payload[i]);
    out += F("\"}}");
  }
  out += F("]}");
}

JsonVariantConst apiSettingValue(JsonObjectConst object, const char *primary, const char *fallback) {
  JsonVariantConst value = object[primary];
  return value.isNull() ? object[fallback] : value;
}

bool applyApiInputPressMqttValues(uint16_t input_number, bool has_topic, const String &topic_value,
                                  bool has_payload, const String &payload_value, StoredConfig &target,
                                  SettingsImportStats &stats, const String &field) {
  if (input_number < 1 || input_number > kMaxButtons) {
    recordSettingsSkipped(stats, field + F(".input"));
    return false;
  }
  const uint8_t input = static_cast<uint8_t>(input_number - 1);
  if (!buttonAvailableIn(runtime_template, input)) {
    recordSettingsSkipped(stats, field + F(".input"));
    return false;
  }
  if (!has_topic && !has_payload) {
    recordSettingsSkipped(stats, field + F(".press"));
    return false;
  }

  String topic = target.button_press_target[input];
  topic.trim();
  if (topic.length() == 0 || !isValidMqttPublishTopicTemplate(topic)) {
    topic = kDefaultButtonMqttTopic;
  }
  String payload = target.button_press_payload[input];
  if (payload.length() == 0 || !isValidButtonActionText(payload, kButtonActionPayloadMaxLen, false, true)) {
    payload = kDefaultButtonMqttPressPayload;
  }

  if (has_topic) {
    topic = topic_value;
    topic.trim();
  }
  if (has_topic && !isValidMqttPublishTopicTemplate(topic)) {
    recordSettingsSkipped(stats, field + F(".press.mqtt_topic"));
    return false;
  }
  if (has_payload) {
    payload = payload_value;
  }
  if (has_payload && !isValidButtonActionText(payload, kButtonActionPayloadMaxLen, false, true)) {
    recordSettingsSkipped(stats, field + F(".press.mqtt_payload"));
    return false;
  }

  target.input_mode[input] = kInputModeButton;
  target.input_relay[input] = input;
  target.input_on_level[input] = kInputOnLevelUnset;
  target.button_press_action[input] = kButtonActionMqtt;
  strlcpy(target.button_press_target[input], topic.c_str(), sizeof(target.button_press_target[input]));
  strlcpy(target.button_press_payload[input], payload.c_str(), sizeof(target.button_press_payload[input]));
  if (has_topic) recordSettingsApplied(stats);
  if (has_payload) recordSettingsApplied(stats);
  return true;
}

bool applyApiInputPressMqttSetting(JsonObjectConst item, StoredConfig &target, SettingsImportStats &stats, uint8_t item_index) {
  const String field = String(F("inputs[")) + String(item_index) + F("]");
  JsonVariantConst input_value = apiSettingValue(item, "input", "id");
  uint16_t input_number = 0;
  if (!settingsReadUint16(input_value, 1, kMaxButtons, input_number)) {
    recordSettingsSkipped(stats, field + F(".input"));
    return false;
  }

  JsonObjectConst press = item["press"].as<JsonObjectConst>();
  if (press.isNull()) {
    recordSettingsSkipped(stats, field + F(".press"));
    return false;
  }

  JsonVariantConst topic_json = apiSettingValue(press, "mqtt_topic", "topic");
  JsonVariantConst payload_json = apiSettingValue(press, "mqtt_payload", "payload");
  const bool has_topic = !topic_json.isNull();
  const bool has_payload = !payload_json.isNull();

  String topic;
  String payload;
  if (has_topic && !settingsReadString(topic_json, topic, kButtonActionTargetMaxLen)) {
    recordSettingsSkipped(stats, field + F(".press.mqtt_topic"));
    return false;
  }
  if (has_payload && !settingsReadString(payload_json, payload, kButtonActionPayloadMaxLen, false)) {
    recordSettingsSkipped(stats, field + F(".press.mqtt_payload"));
    return false;
  }

  return applyApiInputPressMqttValues(input_number, has_topic, topic, has_payload, payload, target, stats, field);
}

void applyApiInputSettings(JsonObjectConst root, StoredConfig &target, SettingsImportStats &stats) {
  JsonVariantConst inputs_value = root["inputs"];
  if (inputs_value.isNull()) return;
  JsonArrayConst inputs = inputs_value.as<JsonArrayConst>();
  if (inputs.isNull()) {
    recordSettingsSkipped(stats, F("inputs"));
    return;
  }
  const uint8_t count = min(static_cast<size_t>(kMaxButtons), inputs.size());
  for (uint8_t i = 0; i < count; i++) {
    JsonObjectConst item = inputs[i].as<JsonObjectConst>();
    if (item.isNull()) {
      if (!inputs[i].isNull()) recordSettingsSkipped(stats, String(F("inputs[")) + String(i) + F("]"));
      continue;
    }
    applyApiInputPressMqttSetting(item, target, stats, i);
  }
}

bool apiSettingsGetArg(const String &primary, const String &fallback, String &out) {
  if (server.hasArg(primary)) {
    out = server.arg(primary);
    return true;
  }
  if (fallback.length() && server.hasArg(fallback)) {
    out = server.arg(fallback);
    return true;
  }
  return false;
}

bool apiSettingsIndexedArg(uint8_t input_number, const char *primary_suffix, const char *fallback_suffix, String &out) {
  String primary = F("input");
  primary += input_number;
  primary += primary_suffix;
  String fallback = F("input");
  fallback += input_number;
  fallback += fallback_suffix;
  return apiSettingsGetArg(primary, fallback, out);
}

bool applyApiSettingsGetArgs(StoredConfig &target, SettingsImportStats &stats) {
  bool saw_setting_arg = false;

  if (server.hasArg(F("input")) || server.hasArg(F("id"))) {
    saw_setting_arg = true;
    const String input_text = server.hasArg(F("input")) ? server.arg(F("input")) : server.arg(F("id"));
    uint16_t input_number = 0;
    if (!parseUint16Input(input_text, 1, kMaxButtons, input_number)) {
      recordSettingsSkipped(stats, F("query.input"));
    } else {
      String topic;
      String payload;
      const bool has_topic = apiSettingsGetArg(F("mqtt_topic"), F("topic"), topic);
      const bool has_payload = apiSettingsGetArg(F("mqtt_payload"), F("payload"), payload);
      applyApiInputPressMqttValues(input_number, has_topic, topic, has_payload, payload, target, stats, F("query"));
    }
  }

  for (uint8_t input_number = 1; input_number <= kMaxButtons; input_number++) {
    String topic;
    String payload;
    const bool has_topic = apiSettingsIndexedArg(input_number, "_mqtt_topic", "_topic", topic);
    const bool has_payload = apiSettingsIndexedArg(input_number, "_mqtt_payload", "_payload", payload);
    if (!has_topic && !has_payload) continue;
    saw_setting_arg = true;
    applyApiInputPressMqttValues(input_number, has_topic, topic, has_payload, payload, target, stats,
                                 String(F("query.input")) + String(input_number));
  }

  return saw_setting_arg;
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
    case kMqttConnectSubscribeFailed: return F("subscribe_failed");
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

bool mqttReadBytesUntil(uint8_t *buffer, uint32_t length, uint32_t deadline_ms) {
  for (uint32_t i = 0; i < length; i++) {
    if (!mqttReadByteUntil(buffer[i], deadline_ms)) return false;
  }
  return true;
}

bool mqttSkipBytesUntil(uint32_t length, uint32_t deadline_ms) {
  uint8_t ignored = 0;
  for (uint32_t i = 0; i < length; i++) {
    if (!mqttReadByteUntil(ignored, deadline_ms)) return false;
  }
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

String mqttCommandTopicFilter() {
  String topic;
  topic.reserve(strlen(config.mqtt_topic) + 8);
  topic += F("cmnd/");
  topic += config.mqtt_topic;
  topic += F("/#");
  return topic;
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
    last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
  }

  if (light.present) {
    mqtt_pending_light_mask |= kMqttLightPendingAll;
  }
}

bool mqttReadSuback(uint16_t packet_id, uint32_t deadline_ms) {
  uint8_t packet_type = 0;
  uint32_t remaining = 0;
  if (!mqttReadByteUntil(packet_type, deadline_ms)) return false;
  if (packet_type != kMqttPacketSuback) return false;
  if (!mqttReadRemainingLengthUntil(remaining, kMqttSubackMaxRemainingLength, deadline_ms)) return false;
  if (remaining < 3) return false;

  uint8_t id_bytes[2];
  if (!mqttReadBytesUntil(id_bytes, sizeof(id_bytes), deadline_ms)) return false;
  remaining -= sizeof(id_bytes);
  const uint16_t received_id = (static_cast<uint16_t>(id_bytes[0]) << 8) | id_bytes[1];
  if (received_id != packet_id) {
    if (remaining) mqttSkipBytesUntil(remaining, deadline_ms);
    return false;
  }

  uint8_t return_code = 0x80;
  if (!mqttReadByteUntil(return_code, deadline_ms)) return false;
  remaining--;
  if (remaining && !mqttSkipBytesUntil(remaining, deadline_ms)) return false;
  return return_code != 0x80;
}

bool mqttSubscribeCommandTopic() {
  const String filter = mqttCommandTopicFilter();
  if (filter.length() == 0 || filter.length() > kMqttCommandTopicMaxLen) return false;

  const uint32_t remaining_length = 2U + 2U + filter.length() + 1U;
  const bool ok = mqttWriteByte(kMqttPacketSubscribe) &&
                  mqttWriteRemainingLength(remaining_length) &&
                  mqttWriteByte(static_cast<uint8_t>(kMqttCommandPacketId >> 8)) &&
                  mqttWriteByte(static_cast<uint8_t>(kMqttCommandPacketId & 0xffU)) &&
                  mqttWriteString(filter.c_str()) &&
                  mqttWriteByte(0x00);
  if (!ok) return false;
  last_mqtt_io = millis();
  return mqttReadSuback(kMqttCommandPacketId, millis() + kMqttConnackTimeoutMs);
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
  if (!mqttSubscribeCommandTopic()) {
    mqttStop();
    recordMqttConnectResult(kMqttConnectSubscribeFailed, started);
    return false;
  }
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

bool mqttPublishCommandResult(const String &payload) {
  if (payload.length() == 0) return true;
  String topic;
  topic.reserve(strlen(config.mqtt_topic) + 14);
  topic += F("stat/");
  topic += config.mqtt_topic;
  topic += F("/RESULT");
  return mqttPublish(topic.c_str(), payload.c_str());
}

void scheduleMqttLightPublish(uint8_t mask) {
  if (!light.present || !mqttConfigured()) return;
  mqtt_pending_light_mask |= (mask & kMqttLightPendingAll);
}

bool mqttPublishLightState(uint8_t mask) {
  if (!light.present) return true;
  mask &= kMqttLightPendingAll;
  if (!mask) return true;

  String payload;
  payload.reserve(32);
  payload += '{';
  bool needs_comma = false;
  if (mask & kMqttLightPendingDimmer) {
    payload += F("\"Dimmer\":");
    payload += light.dimmer;
    needs_comma = true;
  }
  if (mask & kMqttLightPendingCt) {
    if (needs_comma) payload += ',';
    payload += F("\"CT\":");
    payload += light.ct;
  }
  payload += '}';

  const bool ok = mqttPublishCommandResult(payload);
  if (ok) {
    last_mqtt_state_publish = millis();
  }
  return ok;
}

bool mqttCommandFromTopic(const char *topic, size_t topic_len, const char *&command, size_t &command_len) {
  constexpr size_t prefix_len = 5;
  if (!topic || topic_len <= prefix_len) return false;
  if (strncmp(topic, "cmnd/", prefix_len) != 0) return false;

  const size_t configured_len = strlen(config.mqtt_topic);
  if (configured_len == 0) return false;
  if (topic_len <= prefix_len + configured_len + 1) return false;
  if (memcmp(topic + prefix_len, config.mqtt_topic, configured_len) != 0) return false;
  if (topic[prefix_len + configured_len] != '/') return false;

  command = topic + prefix_len + configured_len + 1;
  command_len = topic_len - prefix_len - configured_len - 1;
  return command_len > 0;
}

bool mqttSendPuback(uint16_t packet_id) {
  const bool ok = mqttWriteByte(kMqttPacketPuback) &&
                  mqttWriteByte(0x02) &&
                  mqttWriteByte(static_cast<uint8_t>(packet_id >> 8)) &&
                  mqttWriteByte(static_cast<uint8_t>(packet_id & 0xffU));
  if (ok) {
    last_mqtt_io = millis();
  }
  return ok;
}

bool mqttProcessPublish(uint8_t packet_type, uint32_t remaining, uint32_t deadline) {
  const uint8_t qos = (packet_type >> 1) & 0x03U;
  if (qos == 3 || remaining < 2) return false;
  if (remaining > kMqttInboundMaxRemainingLength) {
    return mqttSkipBytesUntil(remaining, deadline);
  }

  uint8_t topic_len_bytes[2];
  if (!mqttReadBytesUntil(topic_len_bytes, sizeof(topic_len_bytes), deadline)) return false;
  remaining -= sizeof(topic_len_bytes);
  const uint16_t topic_len = (static_cast<uint16_t>(topic_len_bytes[0]) << 8) | topic_len_bytes[1];
  if (topic_len == 0 || topic_len > remaining) return false;
  if (topic_len > kMqttInboundTopicMaxLen) {
    return mqttSkipBytesUntil(remaining, deadline);
  }

  char topic[kMqttInboundTopicMaxLen + 1];
  if (!mqttReadBytesUntil(reinterpret_cast<uint8_t *>(topic), topic_len, deadline)) return false;
  topic[topic_len] = '\0';
  remaining -= topic_len;

  uint16_t packet_id = 0;
  if (qos > 0) {
    if (remaining < 2) return false;
    uint8_t id_bytes[2];
    if (!mqttReadBytesUntil(id_bytes, sizeof(id_bytes), deadline)) return false;
    remaining -= sizeof(id_bytes);
    packet_id = (static_cast<uint16_t>(id_bytes[0]) << 8) | id_bytes[1];
  }

  if (qos > 1 || remaining > kMqttInboundPayloadMaxLen) {
    if (!mqttSkipBytesUntil(remaining, deadline)) return false;
    if (qos == 1 && !mqttSendPuback(packet_id)) return false;
    return true;
  }

  char payload[kMqttInboundPayloadMaxLen + 1];
  if (remaining && !mqttReadBytesUntil(reinterpret_cast<uint8_t *>(payload), remaining, deadline)) return false;
  payload[remaining] = '\0';

  if (qos == 1 && !mqttSendPuback(packet_id)) return false;

  const char *command = nullptr;
  size_t command_len = 0;
  if (!mqttCommandFromTopic(topic, topic_len, command, command_len)) return true;

  String response;
  String error;
  if (!executeDeviceCommand(command, command_len, payload, remaining, response, error)) {
    return true;
  }
  return mqttPublishCommandResult(response);
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

const __FlashStringHelper *mqttEnergyReportReasonName(uint8_t reason) {
  switch (reason) {
    case kMqttEnergyReportReasonInitial: return F("initial");
    case kMqttEnergyReportReasonInterval: return F("interval");
    case kMqttEnergyReportReasonPowerChange: return F("power change %");
    case kMqttEnergyReportReasonIntervalPowerChange: return F("interval + power change %");
    default: return F("none");
  }
}

uint8_t mqttEnergyReportReason(uint32_t now) {
  if (last_mqtt_energy_publish == 0 || isnan(last_mqtt_energy_power)) {
    return kMqttEnergyReportReasonInitial;
  }

  bool interval_due = false;
  if (config.energy_mqtt_interval > 0) {
    const uint32_t interval_ms = static_cast<uint32_t>(config.energy_mqtt_interval) * 1000UL;
    interval_due = now - last_mqtt_energy_publish >= interval_ms;
  }
  const bool power_change_due = mqttEnergyPowerChangedEnough();
  if (interval_due && power_change_due) return kMqttEnergyReportReasonIntervalPowerChange;
  if (interval_due) return kMqttEnergyReportReasonInterval;
  if (power_change_due) return kMqttEnergyReportReasonPowerChange;
  return kMqttEnergyReportReasonNone;
}

bool mqttPublishEnergyStatus(uint8_t reason) {
  if (!energy.present) return true;

  const String topic = mqttEnergyTopic();
  String payload;
  payload.reserve(260);
  payload += F("{\"StatusSNS\":{\"ENERGY\":{\"Total\":");
  payload += String(reportedEnergyTotalKwh(), 4);
  payload += F(",\"Power\":");
  payload += String(energy.power, 2);
  payload += F(",\"Voltage\":");
  payload += String(energy.voltage, 1);
  payload += F(",\"Current\":");
  payload += String(energy.current, 3);
  if (energy.channel_count > 1) {
    for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
      payload += F(",\"Power");
      payload += String(i + 1);
      payload += F("\":");
      payload += String(energy.channel[i].power, 2);
      payload += F(",\"Current");
      payload += String(i + 1);
      payload += F("\":");
      payload += String(energy.channel[i].current, 3);
    }
  }
  payload += F("}}}");

  const bool ok = mqttPublish(topic.c_str(), payload.c_str());
  if (ok) {
    last_mqtt_energy_publish = millis();
    last_mqtt_energy_power = energy.power;
    last_mqtt_energy_report_reason = reason;
  }
  return ok;
}

void maintainMqttEnergyReports(uint32_t now) {
  if (!mqttEnergyReportingEnabled()) {
    last_mqtt_energy_publish = 0;
    last_mqtt_energy_power = NAN;
    last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
    return;
  }
  if (!mqttEnergyReportReady()) return;

  const uint8_t reason = mqttEnergyReportReason(now);
  if (reason != kMqttEnergyReportReasonNone) {
    mqttPublishEnergyStatus(reason);
  }
}

bool mqttProcessInboundPacket() {
  uint8_t packet_type = 0;
  uint32_t remaining = 0;
  const uint32_t deadline = millis() + kMqttInboundReadTimeoutMs;

  if (!mqttReadByteUntil(packet_type, deadline)) return false;
  if (!mqttReadRemainingLengthUntil(remaining, kMqttInboundMaxRemainingLength, deadline)) return false;

  if (packet_type == kMqttPacketPingresp) {
    if (remaining != 0) return false;
    last_mqtt_ping = 0;
    mqtt_ping_pending = false;
    return true;
  }

  if ((packet_type & 0xf0U) == kMqttPacketPublish) {
    return mqttProcessPublish(packet_type, remaining, deadline);
  }

  return mqttSkipBytesUntil(remaining, deadline);
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
    last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
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

  if (mqtt_pending_light_mask && light.present) {
    const uint8_t mask = mqtt_pending_light_mask;
    if (!mqttPublishLightState(mask)) return;
    mqtt_pending_light_mask &= ~mask;
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

  if (config.mqtt_keepalive > 0 && (runtime_template.relay_count > 0 || light.present)) {
    const uint32_t interval_ms = static_cast<uint32_t>(config.mqtt_keepalive) * 1000UL;
    if (now - last_mqtt_state_publish >= interval_ms) {
      if (runtime_template.relay_count > 0 && !mqttPublishAllRelayStates()) return;
      if (light.present && !mqttPublishLightState(kMqttLightPendingAll)) return;
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

const __FlashStringHelper *energyDriverName() {
  if (energy.driver == kEnergyDriverAde7953) return F("ADE7953");
  if (energy.driver == kEnergyDriverShellyDimmer) return F("Shelly Dimmer");
  if (energy.driver == kEnergyDriverPulse) return energy.hjl ? F("HJL/BL0937") : F("HLW8012");
  return F("none");
}

uint32_t absSigned32(uint32_t value) {
  if (value & 0x80000000UL) return (~value) + 1;
  return value;
}

void updateEnergyAggregateFromChannels() {
  float power = 0.0f;
  float current = 0.0f;
  float voltage = 0.0f;
  uint8_t voltage_count = 0;
  for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
    power += energy.channel[i].power;
    current += energy.channel[i].current;
    if (energy.channel[i].voltage > 0.0f) {
      voltage += energy.channel[i].voltage;
      voltage_count++;
    }
  }
  energy.power = power;
  energy.current = current;
  energy.voltage = voltage_count ? voltage / static_cast<float>(voltage_count) : 0.0f;
}

uint8_t ade7953RegSize(uint16_t reg) {
  switch ((reg >> 8) & 0x0f) {
    case 0x03: return 4;
    case 0x02: return 3;
    case 0x01: return 2;
    case 0x00:
    case 0x07:
    case 0x08:
      return 1;
  }
  return 0;
}

bool ade7953Write(uint16_t reg, uint32_t value) {
  uint8_t size = ade7953RegSize(reg);
  if (!size) return false;
  Wire.beginTransmission(kAde7953Address);
  Wire.write(static_cast<uint8_t>((reg >> 8) & 0xff));
  Wire.write(static_cast<uint8_t>(reg & 0xff));
  while (size--) {
    Wire.write(static_cast<uint8_t>((value >> (8 * size)) & 0xff));
  }
  const bool ok = Wire.endTransmission() == 0;
  delayMicroseconds(5);
  if (!ok) energy.i2c_error_count++;
  return ok;
}

bool ade7953Read(uint16_t reg, uint32_t &value) {
  const uint8_t size = ade7953RegSize(reg);
  if (!size) return false;
  Wire.beginTransmission(kAde7953Address);
  Wire.write(static_cast<uint8_t>((reg >> 8) & 0xff));
  Wire.write(static_cast<uint8_t>(reg & 0xff));
  if (Wire.endTransmission(false) != 0) {
    energy.i2c_error_count++;
    return false;
  }
  if (Wire.requestFrom(kAde7953Address, size) != size) {
    energy.i2c_error_count++;
    return false;
  }
  value = 0;
  for (uint8_t i = 0; i < size; i++) {
    value = (value << 8) | Wire.read();
  }
  return true;
}

bool ade7953SetCalibration(uint8_t channel) {
  if (channel == 0) {
    return ade7953Write(kAde7953AVGain, kAde7953GainDefault) &&
           ade7953Write(kAde7953AIGain, kAde7953GainDefault) &&
           ade7953Write(kAde7953AWGain, kAde7953GainDefault) &&
           ade7953Write(kAde7953AVAGain, kAde7953GainDefault) &&
           ade7953Write(kAde7953AVarGain, kAde7953GainDefault) &&
           ade7953Write(kAde7953PhcalA, 0);
  }
  return ade7953Write(kAde7953BIGain, kAde7953GainDefault) &&
         ade7953Write(kAde7953BWGain, kAde7953GainDefault) &&
         ade7953Write(kAde7953BVAGain, kAde7953GainDefault) &&
         ade7953Write(kAde7953BVarGain, kAde7953GainDefault) &&
         ade7953Write(kAde7953PhcalB, 0);
}

bool ade7953InitRegisters() {
  return ade7953Write(kAde7953Config, 0x0004) &&
         ade7953Write(kAde7953Unlock120, 0x00ad) &&
         ade7953Write(kAde7953Reserved120, 0x0030) &&
         ade7953Write(kAde7953DisNoLoad, 0x07) &&
         ade7953Write(kAde7953ApNoLoad, kAde7953NoLoadThreshold) &&
         ade7953Write(kAde7953VarNoLoad, kAde7953NoLoadThreshold) &&
         ade7953Write(kAde7953DisNoLoad, 0x00) &&
         ade7953SetCalibration(0) &&
         ade7953SetCalibration(1);
}

bool ade7953Probe() {
  Wire.beginTransmission(kAde7953Address);
  return Wire.endTransmission() == 0;
}

bool setupAde7953EnergyMonitor() {
  if (runtime_template.ade7953_irq_pin == kInvalidPin ||
      runtime_template.ade7953_model != kAde7953ModelShelly25 ||
      !digitalPinSupported(runtime_template.i2c_scl_pin) ||
      !digitalPinSupported(runtime_template.i2c_sda_pin)) {
    return false;
  }

  if (digitalPinSupported(runtime_template.ade7953_irq_pin)) {
    pinMode(runtime_template.ade7953_irq_pin, INPUT);
  }

  Wire.begin(runtime_template.i2c_sda_pin, runtime_template.i2c_scl_pin);
  Wire.setClock(100000);
  Wire.setClockStretchLimit(1500);
  delay(100);
  if (!ade7953Probe()) {
    energy.i2c_error_count++;
    return false;
  }
  if (!ade7953InitRegisters()) {
    return false;
  }

  energy.present = true;
  energy.driver = kEnergyDriverAde7953;
  energy.channel_count = 2;
  energy.ade7953_model = runtime_template.ade7953_model;
  energy.ade7953_skip_reads = kAde7953SkipInitialReads;
  energy.last_success_ms = millis();
  return true;
}

bool ade7953ReadHardwareChannel(uint8_t hardware_channel, EnergyChannelState &channel) {
  uint32_t value = 0;
  const uint16_t irms_reg = hardware_channel == 0 ? kAde7953IrmsA : kAde7953IrmsB;
  const uint16_t energy_reg = hardware_channel == 0 ? kAde7953AEnergyA : kAde7953AEnergyB;
  const uint16_t apparent_reg = hardware_channel == 0 ? kAde7953ApEnergyA : kAde7953ApEnergyB;

  if (!ade7953Read(irms_reg, value)) return false;
  channel.current_raw = value;
  if (!ade7953Read(energy_reg, value)) return false;
  channel.active_power_raw = absSigned32(value);
  if (!ade7953Read(apparent_reg, value)) return false;
  channel.apparent_power_raw = absSigned32(value);
  if (!ade7953Read(kAde7953Vrms, value)) return false;
  channel.voltage_raw = value;

  channel.voltage = static_cast<float>(channel.voltage_raw) / static_cast<float>(kAde7953VoltageCal);
  channel.current = channel.active_power_raw ? static_cast<float>(channel.current_raw) / (static_cast<float>(kAde7953CurrentCal) * 10.0f) : 0.0f;
  const float sample_correction = energy.ade7953_sample_ms ? static_cast<float>(energy.ade7953_sample_ms) / 1000.0f : 1.0f;
  channel.power = static_cast<float>(channel.active_power_raw) /
                  (((static_cast<float>(kAde7953PowerCal) / 10.0f) / kAde7953PowerCorrection) * sample_correction);
  return true;
}

bool readAde7953Energy() {
  uint32_t value = 0;
  if (!ade7953Read(kAde7953AccMode, value)) return false;
  energy.ade7953_acc_mode = value;

  EnergyChannelState next[kEnergyMaxChannels]{};
  if (!ade7953ReadHardwareChannel(1, next[0])) return false;  // Shelly 2.5 relay 1 is ADE7953 channel B
  if (!ade7953ReadHardwareChannel(0, next[1])) return false;  // Shelly 2.5 relay 2 is ADE7953 channel A

  if (energy.ade7953_skip_reads) {
    energy.ade7953_skip_reads--;
    return true;
  }

  for (uint8_t i = 0; i < kEnergyMaxChannels; i++) {
    energy.channel[i] = next[i];
  }
  updateEnergyAggregateFromChannels();
  energy.last_success_ms = millis();
  return true;
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
  energy.last_success_ms = energy.last_update_ms;

  if (runtime_template.shelly_dimmer) {
    energy.present = true;
    energy.driver = kEnergyDriverShellyDimmer;
    energy.channel_count = 1;
    return;
  }

  if (setupAde7953EnergyMonitor()) {
    return;
  }

  energy.present = interruptPinSupported(energy.cf_pin);
  if (!energy.present) return;
  energy.driver = kEnergyDriverPulse;
  energy.channel_count = 1;

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

uint16_t shellyDimmerTargetBrightness() {
  if (!light.present || !light.power) return 0;
  const uint16_t dimmer = light.dimmer > kLightDimmerMax ? kLightDimmerMax : light.dimmer;
  if (dimmer == 0) return 0;
  uint16_t range_min = config.shelly_dimmer_range_min;
  uint16_t range_max = config.shelly_dimmer_range_max;
  if (range_max == 0 || range_min >= range_max) {
    range_min = kShellyDimmerRangeMinDefault;
    range_max = kShellyDimmerRangeMaxDefault;
  }
  return static_cast<uint16_t>(
    (range_min * 10U) +
    (((static_cast<uint32_t>(range_max - range_min) * 10U * dimmer) + 50U) / 100U)
  );
}

uint16_t shellyDimmerChecksum(const uint8_t *buf, uint8_t len) {
  uint16_t sum = 0;
  for (uint8_t i = 0; i < len; i++) sum += buf[i];
  return sum;
}

uint16_t readLe16(const uint8_t *buf) {
  return static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
}

uint32_t readLe32(const uint8_t *buf) {
  return static_cast<uint32_t>(buf[0]) |
         (static_cast<uint32_t>(buf[1]) << 8) |
         (static_cast<uint32_t>(buf[2]) << 16) |
         (static_cast<uint32_t>(buf[3]) << 24);
}

void shellyDimmerResetRx() {
  shelly_dimmer.byte_count = 0;
  shelly_dimmer.expected_frame_len = 0;
}

void shellyDimmerUpdateEnergy(float wattage, float voltage, float current) {
  if (energy.driver != kEnergyDriverShellyDimmer) return;
  energy.channel[0].power = wattage;
  energy.channel[0].voltage = voltage;
  energy.channel[0].current = current;
  energy.channel[0].active_power_raw = shelly_dimmer.wattage_raw;
  energy.channel[0].voltage_raw = shelly_dimmer.voltage_raw;
  energy.channel[0].current_raw = shelly_dimmer.current_raw;
  updateEnergyAggregateFromChannels();
  energy.last_success_ms = millis();
}

void shellyDimmerProcessPacket(const uint8_t *frame, uint8_t frame_len) {
  if (frame_len < 7 || frame[0] != kShellyDimmerStartByte) return;
  const uint8_t cmd = frame[2];
  const uint8_t len = frame[3];
  const uint8_t *payload = frame + 4;
  shelly_dimmer.last_rx_ms = millis();

  if (cmd == kShellyDimmerPollCmd && len >= 17) {
    const uint8_t hw_version_raw = payload[0];
    const uint16_t brightness = readLe16(payload + 2);
    const uint32_t wattage_raw = readLe32(payload + 4);
    const uint32_t voltage_raw = readLe32(payload + 8);
    const uint32_t current_raw = readLe32(payload + 12);

    shelly_dimmer.hw_version = hw_version_raw == 0 ? 1 : (hw_version_raw == 1 ? 2 : hw_version_raw);
    shelly_dimmer.actual_brightness = brightness;
    shelly_dimmer.wattage_raw = wattage_raw;
    shelly_dimmer.voltage_raw = voltage_raw;
    shelly_dimmer.current_raw = current_raw;
    shelly_dimmer.fade_rate = payload[16];

    const float wattage = wattage_raw > 0 ? 880373.0f / static_cast<float>(wattage_raw) : 0.0f;
    const float voltage = voltage_raw > 0 ? 347800.0f / static_cast<float>(voltage_raw) : 0.0f;
    const float current = current_raw > 0 ? 1448.0f / static_cast<float>(current_raw) : 0.0f;
    shellyDimmerUpdateEnergy(wattage, voltage, current);
  } else if (cmd == kShellyDimmerVersionCmd && len >= 2) {
    shelly_dimmer.version_minor = payload[0];
    shelly_dimmer.version_major = payload[1];
  } else if (cmd == kShellyDimmerSwitchCmd && len >= 1 && payload[0] == 0x01) {
    shelly_dimmer.actual_brightness = shelly_dimmer.requested_brightness;
  }
}

bool shellyDimmerSerialInput() {
  bool got_packet = false;
  while (Serial.available()) {
    const int value = Serial.read();
    if (value < 0) break;
    const uint8_t byte = static_cast<uint8_t>(value);

    if (shelly_dimmer.byte_count == 0 && byte != kShellyDimmerStartByte) {
      continue;
    }
    if (shelly_dimmer.byte_count >= kShellyDimmerBufferSize) {
      shelly_dimmer.error_count++;
      shellyDimmerResetRx();
      continue;
    }

    shelly_dimmer.buffer[shelly_dimmer.byte_count++] = byte;
    if (shelly_dimmer.byte_count == 4) {
      const uint8_t payload_len = shelly_dimmer.buffer[3];
      shelly_dimmer.expected_frame_len = static_cast<uint8_t>(4U + payload_len + 3U);
      if (shelly_dimmer.expected_frame_len > kShellyDimmerBufferSize) {
        shelly_dimmer.error_count++;
        shellyDimmerResetRx();
      }
      continue;
    }

    if (shelly_dimmer.expected_frame_len == 0 ||
        shelly_dimmer.byte_count < shelly_dimmer.expected_frame_len) {
      continue;
    }

    const uint8_t payload_len = shelly_dimmer.buffer[3];
    const uint8_t checksum_pos = static_cast<uint8_t>(4U + payload_len);
    const uint16_t received_checksum =
      (static_cast<uint16_t>(shelly_dimmer.buffer[checksum_pos]) << 8) |
      shelly_dimmer.buffer[checksum_pos + 1];
    const uint16_t calculated_checksum =
      shellyDimmerChecksum(shelly_dimmer.buffer + 1, static_cast<uint8_t>(3U + payload_len));
    if (shelly_dimmer.buffer[shelly_dimmer.expected_frame_len - 1] == kShellyDimmerEndByte &&
        received_checksum == calculated_checksum) {
      shellyDimmerProcessPacket(shelly_dimmer.buffer, shelly_dimmer.expected_frame_len);
      got_packet = true;
    } else {
      shelly_dimmer.error_count++;
    }
    shellyDimmerResetRx();
  }
  return got_packet;
}

bool shellyDimmerWaitForResponse(uint32_t timeout_ms) {
  const uint32_t deadline = millis() + timeout_ms;
  while (static_cast<int32_t>(millis() - deadline) < 0) {
    if (shellyDimmerSerialInput()) return true;
    delay(1);
  }
  return false;
}

bool shellyDimmerSendCmd(uint8_t cmd, const uint8_t *payload, uint8_t len) {
  if (!shelly_dimmer.present || !shelly_dimmer.serial_claimed || len > kShellyDimmerMaxPayloadSize) {
    return false;
  }

  uint8_t frame[4 + kShellyDimmerMaxPayloadSize + 3];
  uint8_t pos = 0;
  frame[pos++] = kShellyDimmerStartByte;
  frame[pos++] = shelly_dimmer.counter++;
  frame[pos++] = cmd;
  frame[pos++] = len;
  if (payload && len) {
    memcpy(frame + pos, payload, len);
    pos += len;
  }
  const uint16_t checksum = shellyDimmerChecksum(frame + 1, static_cast<uint8_t>(3U + len));
  frame[pos++] = checksum >> 8;
  frame[pos++] = checksum & 0xff;
  frame[pos++] = kShellyDimmerEndByte;

  shellyDimmerSerialInput();
  Serial.write(frame, pos);
  Serial.flush();
  shelly_dimmer.last_command_ms = millis();
  if (shellyDimmerWaitForResponse(kShellyDimmerResponseTimeoutMs)) {
    return true;
  }
  shelly_dimmer.timeout_count++;
  return false;
}

bool shellyDimmerSendVersion() {
  return shellyDimmerSendCmd(kShellyDimmerVersionCmd, nullptr, 0);
}

bool shellyDimmerSendSettings() {
  uint8_t payload[10];
  const uint16_t brightness = shellyDimmerTargetBrightness();
  const uint16_t edge_mode = shellyDimmerSettingsEdgePayload();
  const uint16_t fade_rate = 0;
  payload[0] = brightness & 0xff;
  payload[1] = brightness >> 8;
  payload[2] = edge_mode & 0xff;
  payload[3] = edge_mode >> 8;
  payload[4] = fade_rate & 0xff;
  payload[5] = fade_rate >> 8;
  payload[6] = kShellyDimmerDefaultWarmupBrightness & 0xff;
  payload[7] = kShellyDimmerDefaultWarmupBrightness >> 8;
  payload[8] = kShellyDimmerDefaultWarmupMs & 0xff;
  payload[9] = kShellyDimmerDefaultWarmupMs >> 8;
  return shellyDimmerSendCmd(kShellyDimmerSettingsCmd, payload, sizeof(payload));
}

bool shellyDimmerSendBrightness(uint16_t brightness) {
  uint8_t payload[2];
  payload[0] = brightness & 0xff;
  payload[1] = brightness >> 8;
  return shellyDimmerSendCmd(kShellyDimmerSwitchCmd, payload, sizeof(payload));
}

void shellyDimmerResetToAppMode() {
  pinMode(runtime_template.shelly_dimmer_reset_pin, OUTPUT);
  digitalWrite(runtime_template.shelly_dimmer_reset_pin, LOW);
  pinMode(runtime_template.shelly_dimmer_boot0_pin, OUTPUT);
  digitalWrite(runtime_template.shelly_dimmer_boot0_pin, LOW);
  delay(50);
  while (Serial.available()) Serial.read();
  digitalWrite(runtime_template.shelly_dimmer_reset_pin, HIGH);
  delay(50);
}

void setupShellyDimmerRuntime() {
  memset(&shelly_dimmer, 0, sizeof(shelly_dimmer));
  shelly_dimmer.requested_brightness = 0xffffU;
  if (!runtime_template.shelly_dimmer) return;

  shelly_dimmer.present = true;
  shelly_dimmer.counter = 1;
  Serial.begin(115200);
  shelly_dimmer.serial_claimed = true;
  shellyDimmerResetToAppMode();
  shellyDimmerSendVersion();
  shellyDimmerSendSettings();
}

void shellyDimmerApplyLightState(bool force) {
  if (!shelly_dimmer.present) return;
  const uint16_t target = shellyDimmerTargetBrightness();
  const bool target_changed = shelly_dimmer.requested_brightness != target;
  const uint32_t now = millis();
  shelly_dimmer.requested_brightness = target;
  if (force || target_changed ||
      (target != shelly_dimmer.actual_brightness && now - shelly_dimmer.last_command_ms >= kShellyDimmerRetryMs)) {
    shellyDimmerSendBrightness(target);
  }
}

void shellyDimmerPoll() {
  if (!shelly_dimmer.present) return;
  shellyDimmerSendCmd(kShellyDimmerPollCmd, nullptr, 0);
}

void maintainShellyDimmer() {
  if (!shelly_dimmer.present) return;
  shellyDimmerSerialInput();
  const uint32_t now = millis();
  if (now - shelly_dimmer.last_poll_ms >= kShellyDimmerPollMs) {
    shelly_dimmer.last_poll_ms = now;
    shellyDimmerPoll();
  }
  shellyDimmerApplyLightState(false);
}

uint8_t sanitizeLightDimmerValue(uint16_t value) {
  if (value < kLightDimmerMin) return kLightDimmerMin;
  if (value > kLightDimmerMax) return kLightDimmerMax;
  return static_cast<uint8_t>(value);
}

uint16_t sanitizeLightCtValue(uint16_t value) {
  if (value < kLightCtMin) return kLightCtMin;
  if (value > kLightCtMax) return kLightCtMax;
  return value;
}

void loadLightStateFromConfig() {
  light.power = config.light_power != 0;
  light.dimmer = light.power ? sanitizeLightDimmerValue(config.light_dimmer) : kLightDimmerOff;
  light.ct = sanitizeLightCtValue(config.light_ct);
  light.config_dirty = false;
  light.config_save_at = 0;
}

uint16_t lightBrightnessDuty() {
  if (!light.present || !light.power) return 0;
  return static_cast<uint16_t>(((static_cast<uint32_t>(light.dimmer) * kLightPwmRange) + 50U) / 100U);
}

uint16_t lightPwmDuty(uint8_t index) {
  const uint16_t brightness = lightBrightnessDuty();
  if (brightness == 0 || runtime_template.pwm_count <= 1) return brightness;

  const uint16_t ct = sanitizeLightCtValue(light.ct);
  const uint16_t ct_range = kLightCtMax - kLightCtMin;
  const uint16_t warm = static_cast<uint16_t>(
    ((static_cast<uint32_t>(ct - kLightCtMin) * brightness) + (ct_range / 2U)) / ct_range
  );
  return index == 0 ? static_cast<uint16_t>(brightness - warm) : warm;
}

void writeLightPwm(uint8_t index, uint16_t duty) {
  if (index >= kMaxLightPwms || !hasPin(runtime_template.light_pwm[index])) return;
  if (duty > kLightPwmRange) duty = kLightPwmRange;
  if (runtime_template.light_pwm[index].inverted) {
    duty = kLightPwmRange - duty;
  }
  analogWrite(runtime_template.light_pwm[index].pin, duty);
}

void updateLightOutputs() {
  if (!light.present) return;
  if (runtime_template.shelly_dimmer) {
    shellyDimmerApplyLightState(false);
    return;
  }
  for (uint8_t i = 0; i < runtime_template.pwm_count && i < kMaxLightPwms; i++) {
    if (!hasPin(runtime_template.light_pwm[i])) continue;
    writeLightPwm(i, lightPwmDuty(i));
  }
}

void scheduleLightConfigPersist() {
  if (!light.present) return;
  config.light_power = light.power ? 1 : 0;
  config.light_dimmer = light.dimmer;
  config.light_ct = light.ct;
  light.config_dirty = true;
  light.config_save_at = millis() + kLightPersistDelayMs;
}

bool persistLightConfig(bool force = false) {
  if (!light.config_dirty) return true;
  if (!force && static_cast<int32_t>(millis() - light.config_save_at) < 0) return true;
  config.light_power = light.power ? 1 : 0;
  config.light_dimmer = light.dimmer;
  config.light_ct = light.ct;
  if (!commitConfig()) return false;
  light.config_dirty = false;
  return true;
}

void setLightPower(bool on, bool persist = true) {
  if (!light.present) return;
  const bool changed = light.power != on;
  const uint8_t on_dimmer = sanitizeLightDimmerValue(config.light_on_dimmer);
  const uint8_t target_dimmer = on ? on_dimmer : kLightDimmerOff;
  const bool dimmer_changed = light.dimmer != target_dimmer;
  light.power = on;
  light.dimmer = target_dimmer;
  updateLightOutputs();
  if (changed || dimmer_changed) scheduleMqttLightPublish(kMqttLightPendingDimmer);
  if (persist && (changed || dimmer_changed)) scheduleLightConfigPersist();
}

void toggleLightPower(bool persist) {
  setLightPower(!light.power, persist);
}

void setLightDimmer(uint16_t dimmer, bool persist = true) {
  if (!light.present) return;
  if (dimmer == 0) {
    setLightPower(false, persist);
    return;
  }
  const uint8_t sanitized = sanitizeLightDimmerValue(dimmer);
  const bool changed = light.dimmer != sanitized || !light.power;
  light.power = true;
  light.dimmer = sanitized;
  updateLightOutputs();
  if (changed) scheduleMqttLightPublish(kMqttLightPendingDimmer);
  if (persist && changed) scheduleLightConfigPersist();
}

void setLightCt(uint16_t ct, bool persist = true) {
  if (!light.present) return;
  const uint16_t sanitized = sanitizeLightCtValue(ct);
  const bool changed = light.ct != sanitized;
  light.ct = sanitized;
  updateLightOutputs();
  if (changed) scheduleMqttLightPublish(kMqttLightPendingCt);
  if (persist && changed) scheduleLightConfigPersist();
}

bool deferButtonPressForRotary(uint8_t button) {
  return button == 0 && light.present && rotary_encoder.present;
}

const int8_t kRotaryStatePos[16] = {0, 1, -1, 2, -1, 0, -2, 1, 1, -2, 0, -1, 2, -1, 1, 0};

void IRAM_ATTR rotaryInterrupt() {
  if (!rotary_encoder.present) return;
  uint8_t state = rotary_encoder.state & 3U;
  if (digitalRead(rotary_encoder.a.pin)) state |= 4U;
  if (digitalRead(rotary_encoder.b.pin)) state |= 8U;
  rotary_encoder.position += kRotaryStatePos[state];
  rotary_encoder.state = state >> 2;
}

void setupLightRuntime() {
  memset(&light, 0, sizeof(light));
  light.present = lightAvailableIn(runtime_template);
  loadLightStateFromConfig();
  if (!light.present) return;

  if (runtime_template.shelly_dimmer) {
    setupShellyDimmerRuntime();
    updateLightOutputs();
    return;
  }

  analogWriteRange(kLightPwmRange);
  analogWriteFreq(kLightPwmFrequency);
  for (uint8_t i = 0; i < runtime_template.pwm_count && i < kMaxLightPwms; i++) {
    if (!hasPin(runtime_template.light_pwm[i])) continue;
    pinMode(runtime_template.light_pwm[i].pin, OUTPUT);
    writeLightPwm(i, 0);
  }
  updateLightOutputs();
}

void setupRotaryEncoder() {
  memset(&rotary_encoder, 0, sizeof(rotary_encoder));
  resetPinAssignment(rotary_encoder.a);
  resetPinAssignment(rotary_encoder.b);
  if (runtime_template.rotary_count == 0) return;
  if (!hasPin(runtime_template.rotary_a[0]) || !hasPin(runtime_template.rotary_b[0])) return;
  if (!interruptPinSupported(runtime_template.rotary_a[0].pin) ||
      !interruptPinSupported(runtime_template.rotary_b[0].pin)) {
    return;
  }

  rotary_encoder.a = runtime_template.rotary_a[0];
  rotary_encoder.b = runtime_template.rotary_b[0];
  pinMode(rotary_encoder.a.pin, rotary_encoder.a.no_pullup ? INPUT : INPUT_PULLUP);
  pinMode(rotary_encoder.b.pin, rotary_encoder.b.no_pullup ? INPUT : INPUT_PULLUP);
  rotary_encoder.state = (digitalRead(rotary_encoder.a.pin) ? 1U : 0U) |
                         (digitalRead(rotary_encoder.b.pin) ? 2U : 0U);
  rotary_encoder.position = 0;
  rotary_encoder.changed_while_pressed = false;
  rotary_encoder.present = true;
  attachInterrupt(digitalPinToInterrupt(rotary_encoder.a.pin), rotaryInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotary_encoder.b.pin), rotaryInterrupt, CHANGE);
}

void maintainRotary() {
  if (!rotary_encoder.present || !light.present) return;
  const uint32_t now = millis();
  if (now - last_rotary_handler < kRotaryHandlerMs) return;
  last_rotary_handler = now;

  noInterrupts();
  const int16_t delta = rotary_encoder.position;
  rotary_encoder.position = 0;
  interrupts();
  if (delta == 0) return;

  const bool button_pressed = runtime_template.button_count > 0 && button_state[0].stable_pressed;
  if (button_pressed) {
    const int32_t next_ct = static_cast<int32_t>(light.ct) +
      (static_cast<int32_t>(delta) * (350 / (kRotaryMaxSteps * kRotaryMiDeskStepScale)));
    setLightCt(static_cast<uint16_t>(next_ct < 0 ? 0 : next_ct), true);
    rotary_encoder.changed_while_pressed = true;
    rotary_suppress_button[0] = true;
  } else {
    if (!light.power && delta > 0) {
      setLightPower(true, true);
      return;
    }
    const int32_t next_dimmer = static_cast<int32_t>(light.dimmer) +
      (static_cast<int32_t>(delta) * (100 / (kRotaryMaxSteps * kRotaryMiDeskStepScale)));
    setLightDimmer(static_cast<uint16_t>(next_dimmer < 0 ? 0 : next_dimmer), true);
  }
}

void maintainLight() {
  maintainShellyDimmer();
  persistLightConfig(false);
}

bool relayTimeEnforcementActive(uint8_t relay) {
  return relayAvailable(relay) &&
         config.relay_time_enabled[relay] &&
         config.relay_time_seconds[relay] >= kRelayEnforcementMinSeconds;
}

void cancelRelayEnforcement(uint8_t relay) {
  if (relay >= kMaxRelays) return;
  relay_enforcement_pending[relay] = false;
  relay_enforcement_due[relay] = 0;
}

void scheduleRelayEnforcement(uint8_t relay) {
  if (relay >= kMaxRelays || !relayTimeEnforcementActive(relay)) {
    cancelRelayEnforcement(relay);
    return;
  }
  relay_enforcement_due[relay] = millis() + (static_cast<uint32_t>(config.relay_time_seconds[relay]) * 1000UL);
  relay_enforcement_pending[relay] = true;
}

void refreshRelayEnforcementRuntime(bool schedule_off_relays) {
  for (uint8_t i = 0; i < kMaxRelays; i++) {
    if (!relayTimeEnforcementActive(i) || relay_state[i]) {
      cancelRelayEnforcement(i);
    } else if (schedule_off_relays) {
      scheduleRelayEnforcement(i);
    }
  }
}

void setRelay(uint8_t relay, bool on) {
  if (relay >= kMaxRelays || !hasPin(runtime_template.relays[relay])) return;
  const bool changed = relay_state[relay] != on;
  const bool was_on = relay_state[relay];
  relay_state[relay] = on;
  writeAssignedPin(runtime_template.relays[relay], on);
  if (on) {
    cancelRelayEnforcement(relay);
  } else if (changed) {
    scheduleRelayEnforcement(relay);
  }
  if (changed) {
    scheduleMqttRelayPublish(relay);
    if (energy.present && was_on && !on) {
      energy_persist_requested = true;
    }
    updateDeviceLeds(true);
  }
}

void toggleRelay(uint8_t relay) {
  if (relay >= kMaxRelays) return;
  setRelay(relay, !relay_state[relay]);
}

void applyRelayOnBootEnforcement() {
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relayAvailable(i) || !config.relay_on_boot[i]) continue;
    setRelay(i, true);
  }
}

void maintainRelayEnforcement() {
  const uint32_t now = millis();
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relay_enforcement_pending[i]) continue;
    if (!relayTimeEnforcementActive(i) || relay_state[i]) {
      cancelRelayEnforcement(i);
      continue;
    }
    if (static_cast<int32_t>(now - relay_enforcement_due[i]) >= 0) {
      setRelay(i, true);
    }
  }
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
    if (buttonRelayTarget(button, hold, relay)) {
      toggleRelay(relay);
      return true;
    }
    if (light.present) {
      toggleLightPower();
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
    const bool raw = readInputActive(i);
    if (raw != button_state[i].raw_pressed) {
      button_state[i].raw_pressed = raw;
      button_state[i].changed_at = now;
    }
    if ((now - button_state[i].changed_at) >= config.button_debounce_ms && raw != button_state[i].stable_pressed) {
      button_state[i].stable_pressed = raw;
      bool led_handled = false;
      if (effectiveInputMode(i) == kInputModeSwitch) {
        uint8_t relay = 0;
        if (inputRelayTarget(i, relay)) {
          setRelay(relay, raw);
        } else if (light.present) {
          setLightPower(raw, false);
        }
        button_state[i].hold_emitted = false;
      } else {
        if (raw) {
          button_state[i].pressed_at = now;
          button_state[i].hold_emitted = false;
          rotary_suppress_button[i] = false;
          if (config.button_hold_action[i] == kButtonActionNone && !deferButtonPressForRotary(i)) {
            led_handled = runButtonAction(i, config.button_press_action[i], false);
            button_state[i].hold_emitted = true;
          }
        } else {
          if (rotary_suppress_button[i]) {
            rotary_suppress_button[i] = false;
          } else if (!button_state[i].hold_emitted) {
            led_handled = runButtonAction(i, config.button_press_action[i], false);
          }
          button_state[i].hold_emitted = false;
        }
      }
      if (!led_handled) updateDeviceLeds(true);
    }
    if (effectiveInputMode(i) == kInputModeButton &&
        button_state[i].stable_pressed &&
        !button_state[i].hold_emitted &&
        now - button_state[i].pressed_at >= config.button_hold_ms) {
      if (rotary_suppress_button[i]) {
        continue;
      }
      if (config.button_hold_action[i] == kButtonActionNone && deferButtonPressForRotary(i)) {
        continue;
      }
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
  if (energy.driver == kEnergyDriverAde7953) {
    if (now - energy.last_update_ms >= kAde7953UpdateMs) {
      energy.ade7953_sample_ms = now - energy.last_update_ms;
      energy.last_update_ms = now;
      if (!readAde7953Energy() && now - energy.last_success_ms >= kAde7953StaleMs) {
        for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
          energy.channel[i].current = 0.0f;
          energy.channel[i].power = 0.0f;
        }
        updateEnergyAggregateFromChannels();
      }
    }
    if (now - energy.last_integrate_ms >= kEnergyIntegrateMs) {
      const uint32_t elapsed = now - energy.last_integrate_ms;
      energy.last_integrate_ms = now;
      energy.total_kwh += (energy.power * static_cast<float>(elapsed)) / 3600000000.0f;
    }
    persistEnergyTotal(false);
    return;
  }

  if (energy.driver == kEnergyDriverShellyDimmer) {
    if (now - energy.last_success_ms >= kShellyDimmerStaleMs) {
      energy.channel[0].power = 0.0f;
      energy.channel[0].current = 0.0f;
      updateEnergyAggregateFromChannels();
    }
    if (now - energy.last_integrate_ms >= kEnergyIntegrateMs) {
      const uint32_t elapsed = now - energy.last_integrate_ms;
      energy.last_integrate_ms = now;
      energy.total_kwh += (energy.power * static_cast<float>(elapsed)) / 3600000000.0f;
    }
    persistEnergyTotal(false);
    return;
  }

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
  setupLightRuntime();
  memset(relay_enforcement_pending, 0, sizeof(relay_enforcement_pending));
  memset(relay_enforcement_due, 0, sizeof(relay_enforcement_due));

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
    const bool active = readInputActive(i);
    button_state[i] = {active, active, false, millis(), millis()};
    if (effectiveInputMode(i) == kInputModeSwitch) {
      uint8_t relay = 0;
      if (inputRelayTarget(i, relay)) {
        setRelay(relay, active);
      } else if (light.present) {
        setLightPower(active, false);
      }
    }
  }
  applyRelayOnBootEnforcement();

  setupRotaryEncoder();
  setupEnergyMonitor();
  maintainAdc();
  updateDeviceLeds(true);
}

void maintainDevice() {
  maintainRotary();
  maintainButtons();
  maintainRelayEnforcement();
  maintainLight();
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
  page += F(".brand{font-size:28px;font-weight:700;letter-spacing:0;color:inherit;text-decoration:none}.brand span{color:#7dd3aa}.sub{color:#c7d0dc;font-size:13px}.meta{display:flex;align-items:center;gap:8px}");
  page += F(".spin{width:13px;height:13px;border:2px solid rgba(255,255,255,.35);border-top-color:#7dd3aa;border-radius:50%;opacity:.55}.spin.active{opacity:1;animation:rot .7s linear infinite}@keyframes rot{to{transform:rotate(360deg)}}main{max-width:1080px;margin:18px auto 28px;padding:0 14px}");
  page += F(".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:14px;box-shadow:0 1px 2px rgba(0,0,0,.04)}.wide{grid-column:1/-1}");
  page += F(".panel h2{font-size:17px;margin:0 0 12px}.panel-title{display:flex;align-items:center;justify-content:space-between;gap:12px;margin:0 0 12px}.panel-title h2{margin:0}.kv{display:grid;grid-template-columns:minmax(110px,42%) 1fr;gap:8px 12px}.kv span,.hint{color:var(--muted)}.kv div{min-width:0}");
  page += F("code{background:#eef2f6;border:1px solid #dce3ea;border-radius:4px;padding:1px 4px;word-break:break-word}.pill{display:inline-block;border-radius:999px;padding:2px 8px;background:#eef2f6;color:#364152}.pill.ok{background:var(--ok);color:#fff}.pill.bad{background:var(--bad);color:#fff}.panel h2 .pill{font-size:13px;font-weight:400;vertical-align:1px}.ok{color:var(--ok)}.bad{color:var(--bad)}.muted{color:var(--muted)}");
  page += F(".note{background:#eef2f6;border:1px solid #dce3ea;border-radius:6px;padding:10px;margin:10px 0}.note p{margin:0 0 7px}.tokens{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:8px}.tokens div{display:flex;flex-direction:column;gap:3px}.help{position:relative;margin-left:auto}.help-q{display:inline-flex;align-items:center;justify-content:center;width:24px;height:24px;border:1px solid var(--line);border-radius:50%;background:#eef2f6;color:var(--accent2);font-size:14px;font-weight:700;cursor:help}.help-box{display:none;position:absolute;right:0;top:30px;z-index:30;width:520px;max-width:calc(100vw - 48px);background:var(--panel);border:1px solid var(--line);border-radius:8px;padding:12px;box-shadow:0 8px 24px rgba(0,0,0,.18);color:var(--text);font-size:14px;font-weight:400;line-height:1.4}.help:hover .help-box,.help:focus-within .help-box{display:block}.help-box p{margin:0 0 8px}.button-block{border-top:1px solid var(--line);margin-top:12px;padding-top:12px}.action-extra,.mode-extra{display:none}.action-extra.show,.mode-extra.show{display:block}.hidden{display:none}");
  page += F("form{margin:0}.row{margin:10px 0}label{display:block;font-weight:600;color:#344054}input,button,select,textarea{font:inherit}input,select,textarea{width:100%;margin-top:4px;padding:9px;border:1px solid #b9c4d0;border-radius:6px;background:#fff}input[type=checkbox]{width:auto;margin:0 6px 0 0;padding:0;vertical-align:-1px}textarea{min-height:92px;resize:vertical}");
  page += F("button,.btn{display:inline-block;margin:4px 4px 0 0;padding:8px 12px;border:1px solid var(--accent);border-radius:6px;background:var(--accent);color:#fff;text-decoration:none;cursor:pointer}.secondary{background:#fff;color:var(--accent2);border-color:#9eb7cf}.danger{background:#fff;color:var(--bad);border-color:#d4aaa7}.inline{display:inline}.actions{display:flex;flex-wrap:wrap;gap:6px}.inline button{margin:0 4px 0 0}.list{margin:0;padding-left:18px}@media(max-width:520px){.kv{grid-template-columns:1fr}.brand{font-size:24px}}</style></head><body>");
  page += F("<header class='top'><div class='topin'><div><a class='brand' href='/'>my<span>Mota</span></a><div class='sub'>ESP8266/ESP8285 firmware</div></div><div class='sub meta'><span>");
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
  page += F("function sv(n,v){var a=document.getElementsByName(n),e=a&&a[0];if(e&&document.activeElement!==e&&String(e.value)!=String(v))e.value=v;}");
  page += F("function p(i,v,c){var e=document.getElementById(i);if(e){e.textContent=v;e.className=c;}}");
  page += F("function fmt(v,d,u){return v==null?'n/a':Number(v).toFixed(d)+(u||'');}");
  page += F("function live(){fh().then(function(d){");
  page += F("t('live-heap',d.heap+' bytes');t('live-uptime',d.uptime+'s');t('live-active-phy',d.active_phy);");
  page += F("if(d.perf){t('live-loop-load',d.perf.loop_load+'%');t('live-loop-hz',d.perf.loop_hz+'/s');t('live-loop-max',Number(d.perf.loop_max_us/1000).toFixed(1)+' ms');}");
  page += F("t('live-recovery',d.recovery.fast_boot_count+'/'+d.recovery.limit);");
  page += F("p('live-wifi',d.wifi?'connected':'disconnected',d.wifi?'pill ok':'pill bad');t('live-ssid',d.wifi_ssid||'n/a');t('live-ip',d.ip||'n/a');t('live-rssi',d.rssi==null?'n/a':d.rssi+' dBm');");
  page += F("p('live-mqtt',d.mqtt.enabled?(d.mqtt.connected?'connected':'disconnected'):'not configured',d.mqtt.enabled?(d.mqtt.connected?'pill ok':'pill bad'):'pill');");
  page += F("if(d.mqtt){t('live-mqtt-pending',d.mqtt.pending);t('live-mqtt-result',d.mqtt.last_connect_result);t('live-mqtt-connect-ms',d.mqtt.last_connect_ms+' ms');t('live-mqtt-attempt',d.mqtt.last_attempt_ms_ago==null?'n/a':d.mqtt.last_attempt_ms_ago+' ms ago');}");
  page += F("if(d.light){p('live-light-power',d.light.power?'on':'off',d.light.power?'pill ok':'pill bad');t('live-light-dimmer',d.light.dimmer+'%');t('live-light-ct',d.light.ct+' mired');t('live-light-on-dimmer',d.light.on_dimmer+'%');sv('dimmer',d.light.dimmer);sv('ct',d.light.ct);if(d.light.shelly_dimmer){var sd=d.light.shelly_dimmer;t('live-shelly-edge',sd.edge||'auto');t('live-shelly-range-min',sd.range_min);t('live-shelly-range-max',sd.range_max);sv('shelly_edge',sd.edge||'auto');sv('shelly_range_min',sd.range_min);sv('shelly_range_max',sd.range_max);}}");
  page += F("if(d.power){for(var i=0;i<d.power.length;i++){if(d.power[i]!==null)p('live-relay-'+i,d.power[i]?'on':'off',d.power[i]?'pill ok':'pill bad');}}");
  page += F("if(d.buttons){for(var b=0;b<d.buttons.length;b++){if(d.buttons[b])p('live-button-'+b,d.buttons[b].state||(d.buttons[b].pressed?'pressed':'released'),d.buttons[b].pressed?'pill ok':'pill bad');}}");
  page += F("if(d.leds){for(var l=0;l<d.leds.length;l++){if(d.leds[l])p('live-led-'+l,d.leds[l].on?'on':'off',d.leds[l].on?'pill ok':'pill bad');}}");
  page += F("if(d.energy){t('live-energy-power',fmt(d.energy.power,1,' W'));t('live-energy-voltage',fmt(d.energy.voltage,1,' V'));t('live-energy-current',fmt(d.energy.current,3,' A'));t('live-energy-total',fmt(d.energy.total_kwh,4,' kWh'));t('live-energy-offset',fmt(d.energy.offset_kwh,4,' kWh'));t('live-energy-mqtt-age',d.energy.last_mqtt_report_ms_ago==null?'n/a':d.energy.last_mqtt_report_ms_ago+' ms ago');t('live-energy-mqtt-reason',d.energy.last_mqtt_report_reason||'n/a');if(d.energy.channels){for(var e=0;e<d.energy.channels.length;e++){t('live-energy-ch'+e+'-power',fmt(d.energy.channels[e].power,1,' W'));t('live-energy-ch'+e+'-current',fmt(d.energy.channels[e].current,3,' A'));}}}");
  page += F("t('live-temp',d.temperature_c==null?'n/a':Number(d.temperature_c).toFixed(1)+' C');t('live-adc-raw',d.adc_raw==null?'n/a':d.adc_raw);");
  page += F("}).catch(function(){});}");
  page += F("function ba(s){var k=s.getAttribute('data-key'),v=s.value,b=document.getElementById('extra-'+k);if(!b)return;var t=b.querySelector('.target-input'),p=b.querySelector('.payload-input'),rr=b.querySelector('.relay-row'),tr=b.querySelector('.target-row'),pr=b.querySelector('.payload-row'),tl=b.querySelector('.target-label'),h=b.querySelector('.action-hint');b.className=(v=='1'||v=='2'||v=='3')?'action-extra show':'action-extra';if(rr)rr.className=v=='1'?'row relay-row':'row relay-row hidden';if(tr)tr.className=(v=='2'||v=='3')?'row target-row':'row target-row hidden';if(pr)pr.className=(v=='2')?'row payload-row':'row payload-row hidden';if(v=='1'){if(h)h.textContent='Toggles the configured output.';}else if(v=='2'){if(t&&(!t.value||t.value.indexOf('http://')==0))t.value=t.getAttribute('data-default-topic');if(p&&!p.value)p.value=p.getAttribute('data-default-payload');if(tl)tl.textContent='MQTT topic';if(h)h.textContent='Publishes this topic and payload through the configured MQTT broker.';}else if(v=='3'){if(tl)tl.textContent='Webhook URL';if(h)h.textContent='Executes an HTTP GET request; only http:// URLs are supported.';}}");
  page += F("function im(s){var k=s.getAttribute('data-input'),v=s.value,b=document.getElementById('input-button-'+k),w=document.getElementById('input-switch-'+k);if(b)b.className=v=='0'?'mode-extra show':'mode-extra';if(w)w.className=v=='1'?'mode-extra show':'mode-extra';}");
  page += F("function ts(){var s=document.getElementById('known-template'),t=document.getElementById('template-json');if(!s||!t)return;var v=t.value.trim(),m=0;for(var i=1;i<s.options.length;i++){if(s.options[i].getAttribute('data-json')==v){m=i;break;}}s.selectedIndex=m;}");
  page += F("function tp(s){var o=s.options[s.selectedIndex],t=document.getElementById('template-json');if(o&&t&&o.getAttribute('data-json')){t.value=o.getAttribute('data-json');ts();}}");
  page += F("function sf(i){var t=document.getElementById('settings-json');if(!i.files||!i.files[0]||!t)return;var r=new FileReader();r.onload=function(){t.value=String(r.result||'');};r.readAsText(i.files[0]);}");
  page += F("function lu(i){var e=i.getAttribute('data-live'),s=i.getAttribute('data-suffix')||'';if(e)t(e,i.value+s);}function la(i){lu(i);var fd=new FormData();fd.append(i.name,i.value);fd.append('_inline','1');fetch('/light',{method:'POST',body:fd,cache:'no-store'}).then(function(r){if(!r.ok)return r.text().then(function(x){throw Error(x||r.statusText)});live();}).catch(function(x){alert(x.message||x);});}");
  page += F("function bi(){var a=document.querySelectorAll('.button-action');for(var i=0;i<a.length;i++){a[i].onchange=function(){ba(this)};ba(a[i]);}var m=document.querySelectorAll('.input-mode');for(var j=0;j<m.length;j++){m[j].onchange=function(){im(this)};im(m[j]);}var l=document.querySelectorAll('.light-auto');for(var k=0;k<l.length;k++){l[k].oninput=function(){lu(this)};l[k].onchange=function(){la(this)};}var t=document.getElementById('template-json');if(t){t.oninput=ts;t.onchange=ts;}ts();}bi();");
  page += F("document.addEventListener('click',function(e){var b=e.target;while(b&&b.tagName!='BUTTON'&&b.tagName!='INPUT')b=b.parentNode;if(!b||!b.form)return;var t=(b.type||'').toLowerCase();if(t=='submit'||t=='image')b.form._s=b;},true);");
  page += F("document.addEventListener('submit',function(e){var f=e.target;if(!f||f.getAttribute('data-inline')!='1')return;e.preventDefault();var fd=new FormData(f),b=e.submitter||f._s;if(b&&b.name)fd.append(b.name,b.value);fd.append('_inline','1');fetch(f.getAttribute('action')||location.pathname,{method:(f.method||'POST').toUpperCase(),body:fd,cache:'no-store'}).then(function(r){if(!r.ok)return r.text().then(function(x){throw Error(x||r.statusText)});live();}).catch(function(x){alert(x.message||x);});},true);");
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

  page += F("<span>MQTT</span><div>");
  if (config.mqtt_host[0] == '\0') {
    page += F("<span id='live-mqtt' class='pill'>not configured</span>");
  } else if (mqtt_client.connected()) {
    page += F("<span id='live-mqtt' class='pill ok'>connected</span>");
  } else {
    page += F("<span id='live-mqtt' class='pill bad'>disconnected</span>");
  }
  page += F("</div><span>MQTT broker</span><div>");
  if (config.mqtt_host[0] == '\0') {
    page += F("<span class='muted'>not configured</span>");
  } else {
    page += F("<code>");
    page += htmlEscape(config.mqtt_host);
    page += F(":");
    page += String(config.mqtt_port);
    page += F("</code>");
  }
  page += F("</div><span>MQTT topic</span><div><code>");
  page += htmlEscape(config.mqtt_topic);
  page += F("</code></div><span>MQTT keepalive</span><div><code>");
  if (config.mqtt_keepalive == 0) {
    page += F("disabled");
  } else {
    page += String(config.mqtt_keepalive);
    page += F("s");
  }
  page += F("</code></div><span>MQTT pending</span><div><code id='live-mqtt-pending'>");
  page += String(mqtt_pending_relay_mask);
  page += F("</code></div><span>MQTT last connect</span><div><code id='live-mqtt-result'>");
  page += mqttConnectResultName(last_mqtt_connect_result);
  page += F("</code> in <code id='live-mqtt-connect-ms'>");
  page += String(last_mqtt_connect_duration);
  page += F(" ms</code></div><span>MQTT last attempt</span><div><code id='live-mqtt-attempt'>");
  if (last_mqtt_connect_attempt == 0) {
    page += F("n/a");
  } else {
    page += String(millis() - last_mqtt_connect_attempt);
    page += F(" ms ago");
  }
  page += F("</code></div>");
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
    page += F("</code> inputs <code>");
    page += String(runtime_template.led_count);
    page += F("</code> LEDs <code>");
    page += String(runtime_template.pwm_count);
    page += F("</code> PWM</div>");
    if (light.present) {
      page += F("<span>Light</span><div>");
      if (runtime_template.shelly_dimmer) {
        page += F("Shelly Dimmer STM32 serial TX <code>");
        page += pinName(runtime_template.serial_tx_pin);
        page += F("</code>, RX <code>");
        page += pinName(runtime_template.serial_rx_pin);
        page += F("</code>");
      } else {
        for (uint8_t i = 0; i < runtime_template.pwm_count && i < kMaxLightPwms; i++) {
          if (!hasPin(runtime_template.light_pwm[i])) continue;
          if (i) page += F(", ");
          page += i == 0 ? F("cold ") : F("warm ");
          page += F("<code>");
          page += pinName(runtime_template.light_pwm[i].pin);
          page += F("</code>");
        }
      }
      page += F("</div>");
    }
    if (rotary_encoder.present) {
      page += F("<span>Rotary</span><div>A <code>");
      page += pinName(rotary_encoder.a.pin);
      page += F("</code>, B <code>");
      page += pinName(rotary_encoder.b.pin);
      page += F("</code></div>");
    }
    if (runtime_template.i2c_scl_pin != kInvalidPin || runtime_template.i2c_sda_pin != kInvalidPin) {
      page += F("<span>I2C</span><div>SCL <code>");
      page += pinName(runtime_template.i2c_scl_pin);
      page += F("</code>, SDA <code>");
      page += pinName(runtime_template.i2c_sda_pin);
      page += F("</code></div>");
    }
    if (energy.present) {
      page += F("<span>Energy</span><div><code>");
      page += energyDriverName();
      page += F("</code>");
      if (energy.driver == kEnergyDriverAde7953) {
        page += F(" I2C <code>0x38</code>, IRQ/model <code>");
        page += pinName(runtime_template.ade7953_irq_pin);
        page += F("</code>, channels <code>");
        page += String(energy.channel_count);
        page += F("</code>");
      } else if (energy.driver == kEnergyDriverShellyDimmer) {
        page += F(" MCU v<code>");
        page += String(shelly_dimmer.version_major);
        page += F(".");
        page += String(shelly_dimmer.version_minor);
        page += F("</code>, BOOT0 <code>");
        page += pinName(runtime_template.shelly_dimmer_boot0_pin);
        page += F("</code>, reset <code>");
        page += pinName(runtime_template.shelly_dimmer_reset_pin);
        page += F("</code>");
      } else {
        page += F(" CF <code>");
        page += pinName(energy.cf_pin);
        page += F("</code>, CF1 <code>");
        page += pinName(energy.cf1_pin);
        page += F("</code>, SEL <code>");
        page += pinName(energy.sel_pin);
        page += F("</code>");
      }
      page += F("</div>");
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
  if (!runtime_template.enabled || (runtime_template.relay_count == 0 && !energy.present && !light.present)) return;
  page += F("<section class='panel'><h2>Device</h2>");
  if (light.present) {
    const bool has_ct = lightSupportsColorTemperature();
    page += F("<div class='row'><strong>Light</strong> ");
    page += F("<span id='live-light-power' class='");
    page += light.power ? F("pill ok'>on") : F("pill bad'>off");
    page += F("</span><div class='kv'><span>Dimmer</span><div><code id='live-light-dimmer'>");
    page += String(light.dimmer);
    page += F("%</code></div>");
    if (has_ct) {
      page += F("<span>Color temp</span><div><code id='live-light-ct'>");
      page += String(light.ct);
      page += F(" mired</code></div>");
    }
    if (runtime_template.shelly_dimmer) {
      page += F("<span>Dimmer MCU</span><div><code>");
      page += String(shelly_dimmer.version_major);
      page += F(".");
      page += String(shelly_dimmer.version_minor);
      page += F("</code> hw <code>");
      page += String(shelly_dimmer.hw_version);
      page += F("</code></div><span>Edge mode</span><div><code id='live-shelly-edge'>");
      page += shellyDimmerEdgeName(config.shelly_dimmer_edge);
      page += F("</code></div><span>Hardware range</span><div><code id='live-shelly-range-min'>");
      page += String(config.shelly_dimmer_range_min);
      page += F("</code> to <code id='live-shelly-range-max'>");
      page += String(config.shelly_dimmer_range_max);
      page += F("</code></div>");
    }
    page += F("<span>ON dimmer</span><div><code id='live-light-on-dimmer'>");
    page += String(config.light_on_dimmer);
    page += F("%</code></div></div>");
    page += F("<form class='inline' data-inline='1' method='post' action='/light'><span class='actions'><button name='power' value='toggle'>Toggle</button><button name='power' value='on'>On</button><button class='secondary' name='power' value='off'>Off</button></span></form>");
    page += F("<div class='row'><label>Dimmer<br><input class='light-auto' data-live='live-light-dimmer' data-suffix='%' name='dimmer' type='range' min='");
    page += String(kLightDimmerOff);
    page += F("' max='");
    page += String(kLightDimmerMax);
    page += F("' step='1' value='");
    page += String(light.dimmer);
    page += F("'></label></div>");
    if (has_ct) {
      page += F("<div class='row'><label>Color temperature<br><input class='light-auto' data-live='live-light-ct' data-suffix=' mired' name='ct' type='range' min='");
      page += String(kLightCtMin);
      page += F("' max='");
      page += String(kLightCtMax);
      page += F("' step='1' value='");
      page += String(light.ct);
      page += F("'></label></div>");
    }
    page += F("<div class='row'><label>ON dimmer<br><input class='light-auto' data-live='live-light-on-dimmer' data-suffix='%' name='on_dimmer' type='number' min='");
    page += String(kLightDimmerMin);
    page += F("' max='");
    page += String(kLightDimmerMax);
    page += F("' step='1' value='");
    page += String(config.light_on_dimmer);
    page += F("'></label></div>");
    if (runtime_template.shelly_dimmer) {
      page += F("<div class='button-block'><strong>Shelly dimmer</strong><div class='row'><label>Edge mode<br><select class='light-auto' data-live='live-shelly-edge' name='shelly_edge'><option value='auto'");
      if (config.shelly_dimmer_edge == kShellyDimmerEdgeAuto) page += F(" selected");
      page += F(">Auto</option><option value='trailing'");
      if (config.shelly_dimmer_edge == kShellyDimmerEdgeTrailing) page += F(" selected");
      page += F(">Trailing edge</option><option value='leading'");
      if (config.shelly_dimmer_edge == kShellyDimmerEdgeLeading) page += F(" selected");
      page += F(">Leading edge</option></select></label></div><div class='row'><label>Range minimum<br><input class='light-auto' data-live='live-shelly-range-min' name='shelly_range_min' type='number' min='0' max='");
      page += String(kShellyDimmerRangeMaxLimit - 1);
      page += F("' step='1' value='");
      page += String(config.shelly_dimmer_range_min);
      page += F("'></label></div><div class='row'><label>Range maximum<br><input class='light-auto' data-live='live-shelly-range-max' name='shelly_range_max' type='number' min='1' max='");
      page += String(kShellyDimmerRangeMaxLimit);
      page += F("' step='1' value='");
      page += String(config.shelly_dimmer_range_max);
      page += F("'></label></div></div>");
    }
    page += F("</div>");
  }
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
    page += F("<form class='inline' data-inline='1' method='post' action='/power'><input type='hidden' name='relay' value='");
    page += String(i + 1);
    page += F("'><span class='actions'><button name='state' value='toggle'>Toggle</button><button name='state' value='on'>On</button><button class='secondary' name='state' value='off'>Off</button></span></form></div>");
  }
  if (energy.present) {
    page += F("<div class='button-block'><strong>Energy</strong><div class='kv'><span>Power</span><div><code id='live-energy-power'>");
    page += String(energy.power, 1);
    page += F(" W</code></div><span>Voltage</span><div><code id='live-energy-voltage'>");
    page += String(energy.voltage, 1);
    page += F(" V</code></div><span>Current</span><div><code id='live-energy-current'>");
    page += String(energy.current, 3);
    page += F(" A</code></div><span>Total</span><div><code id='live-energy-total'>");
    page += String(reportedEnergyTotalKwh(), 4);
    page += F(" kWh</code></div><span>Total offset</span><div><code id='live-energy-offset'>");
    page += String(config.energy_total_offset_kwh, 4);
    page += F(" kWh</code></div><span>Last MQTT report</span><div><code id='live-energy-mqtt-age'>");
    if (last_mqtt_energy_publish == 0) {
      page += F("n/a");
    } else {
      page += String(millis() - last_mqtt_energy_publish);
      page += F(" ms ago");
    }
    page += F("</code></div><span>MQTT report reason</span><div><code id='live-energy-mqtt-reason'>");
    page += mqttEnergyReportReasonName(last_mqtt_energy_report_reason);
    page += F("</code></div></div>");
    if (energy.channel_count > 1) {
      page += F("<div class='kv'>");
      for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
        page += F("<span>Channel ");
        page += String(i + 1);
        page += F("</span><div><code id='live-energy-ch");
        page += String(i);
        page += F("-power'>");
        page += String(energy.channel[i].power, 1);
        page += F(" W</code> <code id='live-energy-ch");
        page += String(i);
        page += F("-current'>");
        page += String(energy.channel[i].current, 3);
        page += F(" A</code></div>");
      }
      page += F("</div>");
    }
    page += F("<form data-inline='1' method='post' action='/energy'><div class='row'><label>Total kWh offset<br><input name='total_offset_kwh' type='number' min='");
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
    page += F("'></label></div><button type='submit'>Save energy</button></form></div>");
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

  page += F("<section class='panel'><h2>LEDs</h2><form data-inline='1' method='post' action='/leds'>");
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
      appendLedAttachmentOption(page, kLedAttachButtonBase + button, String(F("Input ")) + String(button + 1), selected);
    }
    page += F("</select></label></div>");
  }
  page += F("<button type='submit'>Save LEDs</button></form></section>");
}

void appendRelayEnforcementSettings(String &page) {
  if (!runtime_template.enabled || !hasConfigurableRelays()) return;

  page += F("<section class='panel'><h2>Relay Enforcement</h2><form data-inline='1' method='post' action='/relay-enforcement'>");
  page += F("<p class='hint'>Keep selected relays on at startup, or turn them back on after they are switched off.</p>");
  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relayAvailable(i)) continue;
    page += F("<div class='button-block'><strong>Relay ");
    page += String(i + 1);
    page += F("</strong> <span class='hint'>");
    page += pinName(runtime_template.relays[i].pin);
    page += F("</span><div class='row'><label><input type='checkbox' name='relay_on_boot");
    page += String(i);
    page += F("' value='1'");
    if (config.relay_on_boot[i]) page += F(" checked");
    page += F(">Turn on at boot</label></div><div class='row'><label><input type='checkbox' name='relay_time_enabled");
    page += String(i);
    page += F("' value='1'");
    if (config.relay_time_enabled[i]) page += F(" checked");
    page += F(">Restore after OFF</label><input name='relay_time_seconds");
    page += String(i);
    page += F("' type='number' min='");
    page += String(kRelayEnforcementMinSeconds);
    page += F("' max='");
    page += String(kRelayEnforcementMaxSeconds);
    page += F("' step='1' placeholder='seconds' value='");
    if (config.relay_time_seconds[i] >= kRelayEnforcementMinSeconds) {
      page += String(config.relay_time_seconds[i]);
    }
    page += F("'></div></div>");
  }
  page += F("<button type='submit'>Save relay enforcement</button></form></section>");
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

void appendInputModeOption(String &page, uint8_t value, const String &label, uint8_t selected) {
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

void appendInputRelayOption(String &page, uint8_t value, uint8_t selected) {
  page += F("<option value='");
  page += String(value);
  page += F("'");
  if (selected == value) {
    page += F(" selected");
  }
  page += F(">Relay ");
  page += String(value + 1);
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
    uint8_t relay = 0;
    appendButtonActionOption(page, kButtonActionRelayToggle,
                             defaultButtonRelayTarget(button, relay) ? F("Relay toggle") : F("Light toggle"),
                             selected);
  }
  appendButtonActionOption(page, kButtonActionMqtt, F("MQTT broadcast"), selected);
  appendButtonActionOption(page, kButtonActionWebhook, F("Webhook exec"), selected);
  page += F("</select>");
}

void appendButtonActionExtra(String &page, uint8_t button, const char *name, bool hold) {
  uint8_t selected_relay = 0;
  const bool has_relay_target = buttonRelayTarget(button, hold, selected_relay);
  page += F("<div id='extra-");
  page += name;
  page += String(button);
  page += F("' class='action-extra'>");
  if (has_relay_target) {
    page += F("<div class='row relay-row'><label>Target relay<br><select name='");
    page += name;
    page += F("_relay");
    page += String(button);
    page += F("'>");
    for (uint8_t relay = 0; relay < runtime_template.relay_count; relay++) {
      if (!hasPin(runtime_template.relays[relay])) continue;
      appendInputRelayOption(page, relay, selected_relay);
    }
    page += F("</select></label></div>");
  } else {
    page += F("<div class='row relay-row'><span class='hint'>Toggles the light output.</span></div>");
  }
  page += F("<div class='row target-row'><label><span class='target-label'>MQTT topic</span><br><input class='target-input' name='");
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

  page += F("<section class='panel'><div class='panel-title'><h2>Inputs</h2><div class='help' tabindex='0'><span class='help-q'>?</span><div class='help-box'><p><strong>Action placeholders</strong></p><div class='tokens'>");
  page += F("<div><code>{BUTTONID}</code><span class='hint'>input number, starting at 1</span></div>");
  page += F("<div><code>{TYPE}</code><span class='hint'>TOGGLE on press, HOLD on hold</span></div>");
  page += F("<div><code>{TOPIC}</code><span class='hint'>current MQTT topic</span></div>");
  page += F("<div><code>{RELAYX_STATE}</code><span class='hint'>relay state, for example {RELAY1_STATE}</span></div>");
  page += F("</div><p class='hint'>MQTT broadcast sends a topic and payload through the configured broker. The default values match the switch action format used by tasmota.js: <code>stat/{TOPIC}/RESULT</code> with a <code>Switch{BUTTONID}</code> payload using <code>{TYPE}</code>.</p></div></div></div><form data-inline='1' method='post' action='/buttons'>");
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
  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;
    const uint8_t mode = effectiveInputMode(i);
    const uint8_t on_level = effectiveInputOnLevel(i);
    uint8_t target_relay = 0;
    inputRelayTarget(i, target_relay);
    page += F("<div class='button-block'><strong>");
    page += htmlEscape(inputDisplayName(i));
    page += F("</strong> <span class='hint'>");
    page += pinName(runtime_template.buttons[i].pin);
    page += F(" ");
    page += htmlEscape(inputKindName(i));
    page += F("</span> ");
    if (button_state[i].stable_pressed) {
      page += F("<span id='live-button-");
      page += String(i);
      page += F("' class='pill ok'>");
      page += htmlEscape(inputStateName(i, true));
      page += F("</span>");
    } else {
      page += F("<span id='live-button-");
      page += String(i);
      page += F("' class='pill bad'>");
      page += htmlEscape(inputStateName(i, false));
      page += F("</span>");
    }
    page += F("<div class='row'><label>Kind<br><select class='input-mode' data-input='");
    page += String(i);
    page += F("' name='mode");
    page += String(i);
    page += F("'>");
    appendInputModeOption(page, kInputModeButton, F("Button actions"), mode);
    if (inputCanFollowOutput(i)) {
      appendInputModeOption(page, kInputModeSwitch, F("Switch follows output"), mode);
    }
    page += F("</select></label></div>");

    page += F("<div id='input-switch-");
    page += String(i);
    page += F("' class='mode-extra");
    if (mode == kInputModeSwitch) page += F(" show");
    page += F("'>");
    uint8_t unused_relay = 0;
    if (defaultButtonRelayTarget(i, unused_relay)) {
      page += F("<div class='row'><label>Target relay<br><select name='relay");
      page += String(i);
      page += F("'>");
      for (uint8_t relay = 0; relay < runtime_template.relay_count; relay++) {
        if (!hasPin(runtime_template.relays[relay])) continue;
        appendInputRelayOption(page, relay, target_relay);
      }
      page += F("</select></label></div>");
    } else if (light.present) {
      page += F("<p class='hint'>Switch follows the light output.</p>");
    }
    page += F("<div class='row'><label>Reverse<br><select name='reverse");
    page += String(i);
    page += F("'><option value='0'");
    if (on_level == kInputOnLevelHigh) page += F(" selected");
    page += F(">No, GPIO high is ON</option><option value='1'");
    if (on_level == kInputOnLevelLow) page += F(" selected");
    page += F(">Yes, GPIO low is ON</option></select></label></div></div>");

    page += F("<div id='input-button-");
    page += String(i);
    page += F("' class='mode-extra");
    if (mode == kInputModeButton) page += F(" show");
    page += F("'><div class='row'><label>Press<br>");
    appendButtonActionSelect(page, i, "press", config.button_press_action[i]);
    page += F("</label>");
    appendButtonActionExtra(page, i, "press", false);
    page += F("</div><div class='row'><label>Hold<br>");
    appendButtonActionSelect(page, i, "hold", config.button_hold_action[i]);
    page += F("</label>");
    appendButtonActionExtra(page, i, "hold", true);
    page += F("</div></div></div>");
  }
  page += F("<button type='submit'>Save inputs</button></form></section>");
}

void appendTemplateForm(String &page) {
  page += F("<section class='panel wide'><h2>Template</h2><form method='post' action='/template'>");
  page += F("<div class='row'><label>Known template<br><select id='known-template' onchange='tp(this)'><option value=''>Select a template</option>");
  page += F("<option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateMiDeskLampJson)));
  page += F("'>Mi Desk Lamp</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateNousA1TJson)));
  page += F("'>NOUS A1T 16A</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShelly1Json)));
  page += F("'>Shelly 1</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShelly1LJson)));
  page += F("'>Shelly 1L</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShelly25Json)));
  page += F("'>Shelly 2.5</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShellyDimmer2Json)));
  page += F("'>Shelly Dimmer 2</option><option data-json='");
  page += htmlEscape(String(FPSTR(kTemplateShellyPlugSJson)));
  page += F("'>Shelly Plug S</option></select></label></div>");
  page += F("<div class='row'><label>Tasmota ESP8266 template JSON<br><textarea id='template-json' name='template' rows='5' maxlength='");
  page += String(kTemplateJsonMaxLen);
  page += F("'>");
  page += htmlEscape(currentTemplateJson());
  page += F("</textarea></label></div>");
  page += F("<button type='submit'>Save template</button> <button class='danger' type='submit' name='clear' value='1'>Clear template</button></form></section>");
}

void appendMqttForm(String &page) {
  page += F("<section class='panel'><h2>MQTT Settings</h2><form data-inline='1' method='post' action='/mqtt'>");
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

void appendSettingsForm(String &page) {
  page += F("<section class='panel wide'><h2>Settings</h2>");
  page += F("<p><a class='btn secondary' href='/settings/export'>Export settings</a></p>");
  page += F("<form method='post' action='/settings/import'>");
  page += F("<div class='row'><label>Import settings JSON<br><input type='file' accept='application/json,.json' onchange='sf(this)'></label></div>");
  page += F("<div class='row'><label>Settings JSON<br><textarea id='settings-json' name='settings_json' rows='8' maxlength='");
  page += String(kSettingsImportJsonMaxLen);
  page += F("'></textarea></label></div>");
  page += F("<p class='hint'>Wi-Fi SSID, password, hostname, and PHY mode are not exported or imported.</p>");
  page += F("<button type='submit'>Import settings</button></form></section>");
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
  appendRelayEnforcementSettings(page);
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

  appendMqttForm(page);
  flushStreamChunk(page);

  page += F("<section class='panel'><h2>Firmware</h2><form method='post' action='/update' enctype='multipart/form-data'>");
  page += F("<input type='file' name='firmware' accept='.bin,.bin.gz' required><br><button type='submit'>Upload firmware</button></form>");
  page += F("<p><a class='btn secondary' href='/reboot'>Reboot</a></p>");
  page += F("<form method='post' action='/factory-reset' onsubmit=\"return confirm('Factory reset will delete Wi-Fi, template, MQTT, input, LED, relay enforcement, light, and energy settings. Continue?')\"><button class='danger' type='submit'>Factory reset</button></form></section>");
  flushStreamChunk(page);

  appendSettingsForm(page);
  flushStreamChunk(page);

  appendTemplateForm(page);
  page += F("</div>");
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

  String template_json = server.arg("template");
  template_json.trim();
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

void handleRelayEnforcementSave() {
  if (!hasConfigurableRelays()) {
    server.send(400, F("text/plain"), F("No configurable relays are available"));
    return;
  }

  uint8_t on_boot[kMaxRelays];
  uint8_t time_enabled[kMaxRelays];
  uint16_t time_seconds[kMaxRelays];
  memcpy(on_boot, config.relay_on_boot, sizeof(on_boot));
  memcpy(time_enabled, config.relay_time_enabled, sizeof(time_enabled));
  memcpy(time_seconds, config.relay_time_seconds, sizeof(time_seconds));

  for (uint8_t i = 0; i < runtime_template.relay_count && i < kMaxRelays; i++) {
    if (!relayAvailable(i)) continue;

    String on_boot_arg = F("relay_on_boot");
    on_boot_arg += String(i);
    String time_enabled_arg = F("relay_time_enabled");
    time_enabled_arg += String(i);
    String seconds_arg = F("relay_time_seconds");
    seconds_arg += String(i);

    on_boot[i] = server.hasArg(on_boot_arg) ? 1 : 0;
    time_enabled[i] = server.hasArg(time_enabled_arg) ? 1 : 0;

    String seconds_text = server.hasArg(seconds_arg) ? server.arg(seconds_arg) : String();
    seconds_text.trim();
    if (time_enabled[i]) {
      uint16_t seconds = 0;
      if (!parseUint16Input(seconds_text, kRelayEnforcementMinSeconds, kRelayEnforcementMaxSeconds, seconds)) {
        server.send(400, F("text/plain"), F("Invalid relay enforcement seconds"));
        return;
      }
      time_seconds[i] = seconds;
    } else if (seconds_text.length() == 0) {
      time_seconds[i] = 0;
    } else {
      uint16_t seconds = 0;
      if (parseUint16Input(seconds_text, kRelayEnforcementMinSeconds, kRelayEnforcementMaxSeconds, seconds)) {
        time_seconds[i] = seconds;
      }
    }
  }

  if (!saveRelayEnforcementConfig(on_boot, time_enabled, time_seconds)) {
    server.send(500, F("text/plain"), F("Could not save relay enforcement settings"));
    return;
  }

  String page;
  page.reserve(700);
  appendHeader(page, F("myMota Relay Enforcement"));
  page += F("<p class='ok'>Relay enforcement settings saved.</p>");
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
    server.send(400, F("text/plain"), F("No configurable inputs are available"));
    return;
  }

  uint16_t hold_ms = kButtonHoldDefaultMs;
  if (!parseUint16Input(server.arg("hold_ms"), kButtonHoldMinMs, kButtonHoldMaxMs, hold_ms)) {
    server.send(400, F("text/plain"), F("Invalid input hold time"));
    return;
  }

  uint16_t debounce_ms = kButtonDebounceDefaultMs;
  if (!parseUint16Input(server.arg("debounce_ms"), kButtonDebounceMinMs, kButtonDebounceMaxMs, debounce_ms)) {
    server.send(400, F("text/plain"), F("Invalid input debounce time"));
    return;
  }

  StoredConfig *candidate = new StoredConfig(config);
  if (!candidate) {
    server.send(500, F("text/plain"), F("Could not allocate input settings"));
    return;
  }

  for (uint8_t i = 0; i < runtime_template.button_count; i++) {
    if (!hasPin(runtime_template.buttons[i])) continue;

    String mode_arg = F("mode");
    mode_arg += String(i);
    if (!server.hasArg(mode_arg)) {
      delete candidate;
      server.send(400, F("text/plain"), F("Missing input mode"));
      return;
    }
    uint16_t mode_value = 0;
    if (!parseUint16Input(server.arg(mode_arg), 0, 1, mode_value)) {
      delete candidate;
      server.send(400, F("text/plain"), F("Invalid input mode"));
      return;
    }
    const uint8_t input_mode = static_cast<uint8_t>(mode_value);
    candidate->input_mode[i] = input_mode;

    if (input_mode == kInputModeSwitch) {
      String relay_arg = F("relay");
      relay_arg += String(i);
      String reverse_arg = F("reverse");
      reverse_arg += String(i);
      uint8_t unused_relay = 0;
      const bool has_relay_target = defaultButtonRelayTarget(i, unused_relay);
      if ((has_relay_target && !server.hasArg(relay_arg)) || !server.hasArg(reverse_arg)) {
        delete candidate;
        server.send(400, F("text/plain"), F("Missing switch setting"));
        return;
      }

      uint16_t relay_value = 0;
      uint16_t reverse_value = 0;
      if ((has_relay_target && !parseUint16Input(server.arg(relay_arg), 0, kMaxRelays - 1, relay_value)) ||
          !parseUint16Input(server.arg(reverse_arg), 0, 1, reverse_value)) {
        delete candidate;
        server.send(400, F("text/plain"), F("Invalid switch setting"));
        return;
      }
      if (has_relay_target) {
        const uint8_t relay = static_cast<uint8_t>(relay_value);
        if (relay >= runtime_template.relay_count || !hasPin(runtime_template.relays[relay])) {
          delete candidate;
          server.send(400, F("text/plain"), F("Invalid switch relay"));
          return;
        }
        candidate->input_relay[i] = relay;
      } else if (light.present) {
        candidate->input_relay[i] = kButtonRelayUnset;
      } else {
        delete candidate;
        server.send(400, F("text/plain"), F("Invalid switch target"));
        return;
      }
      candidate->input_on_level[i] = reverse_value ? kInputOnLevelLow : kInputOnLevelHigh;
      continue;
    }

    candidate->input_relay[i] = i;
    candidate->input_on_level[i] = kInputOnLevelUnset;

    String press_arg = F("press");
    press_arg += String(i);
    String hold_arg = F("hold");
    hold_arg += String(i);
    if (!server.hasArg(press_arg) || !server.hasArg(hold_arg)) {
      delete candidate;
      server.send(400, F("text/plain"), F("Missing button action setting"));
      return;
    }

    uint16_t press_value = 0;
    uint16_t hold_value = 0;
    if (!parseUint16Input(server.arg(press_arg), 0, 255, press_value) ||
        !parseUint16Input(server.arg(hold_arg), 0, 255, hold_value)) {
      delete candidate;
      server.send(400, F("text/plain"), F("Invalid button action setting"));
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
    if (!readButtonRelayTargetInput(i, "press", press_action, candidate->button_press_relay, error) ||
        !readButtonRelayTargetInput(i, "hold", hold_action, candidate->button_hold_relay, error) ||
        !readButtonEventText(i, "press", false, press_action, candidate->button_press_target, candidate->button_press_payload, error) ||
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
                                      candidate->button_hold_payload,
                                      candidate->input_mode,
                                      candidate->input_relay,
                                      candidate->input_on_level,
                                      candidate->button_press_relay,
                                      candidate->button_hold_relay);
  delete candidate;
  if (!saved) {
    server.send(500, F("text/plain"), F("Could not save input settings"));
    return;
  }
  updateDeviceLeds(true);

  String page;
  page.reserve(700);
  appendHeader(page, F("myMota Inputs"));
  page += F("<p class='ok'>Input settings saved.</p>");
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

void handleLightSave() {
  if (!light.present) {
    server.send(400, F("text/plain"), F("No light output is configured"));
    return;
  }
  bool light_settings_changed = false;
  bool shelly_settings_changed = false;

  if (server.hasArg("power")) {
    const String state = server.arg("power");
    if (state == "on") {
      setLightPower(true);
    } else if (state == "off") {
      setLightPower(false);
    } else if (state == "toggle") {
      toggleLightPower();
    } else {
      server.send(400, F("text/plain"), F("Invalid light power state"));
      return;
    }
  }

  if (server.hasArg("dimmer")) {
    uint16_t dimmer = 0;
    if (!parseUint16Input(server.arg("dimmer"), kLightDimmerOff, kLightDimmerMax, dimmer)) {
      server.send(400, F("text/plain"), F("Invalid dimmer"));
      return;
    }
    setLightDimmer(dimmer);
  }

  if (server.hasArg("ct")) {
    uint16_t ct = 0;
    if (!parseUint16Input(server.arg("ct"), kLightCtMin, kLightCtMax, ct)) {
      server.send(400, F("text/plain"), F("Invalid color temperature"));
      return;
    }
    setLightCt(ct);
  }

  if (server.hasArg("on_dimmer")) {
    uint16_t on_dimmer = 0;
    if (!parseUint16Input(server.arg("on_dimmer"), kLightDimmerMin, kLightDimmerMax, on_dimmer)) {
      server.send(400, F("text/plain"), F("Invalid ON dimmer"));
      return;
    }
    const uint8_t next_on_dimmer = static_cast<uint8_t>(on_dimmer);
    if (config.light_on_dimmer != next_on_dimmer) {
      config.light_on_dimmer = next_on_dimmer;
      light_settings_changed = true;
    }
  }

  if (runtime_template.shelly_dimmer && server.hasArg("shelly_edge")) {
    uint8_t edge = kShellyDimmerEdgeDefault;
    if (!parseShellyDimmerEdgeName(server.arg("shelly_edge"), edge)) {
      server.send(400, F("text/plain"), F("Invalid Shelly edge mode"));
      return;
    }
    if (config.shelly_dimmer_edge != edge) {
      config.shelly_dimmer_edge = edge;
      light_settings_changed = true;
      shelly_settings_changed = true;
    }
  }

  if (runtime_template.shelly_dimmer && server.hasArg("shelly_range_min")) {
    uint16_t range_min = 0;
    if (!parseUint16Input(server.arg("shelly_range_min"), 0, kShellyDimmerRangeMaxLimit - 1, range_min) ||
        range_min >= config.shelly_dimmer_range_max) {
      server.send(400, F("text/plain"), F("Invalid Shelly range minimum"));
      return;
    }
    if (config.shelly_dimmer_range_min != range_min) {
      config.shelly_dimmer_range_min = static_cast<uint8_t>(range_min);
      light_settings_changed = true;
      shelly_settings_changed = true;
    }
  }

  if (runtime_template.shelly_dimmer && server.hasArg("shelly_range_max")) {
    uint16_t range_max = 0;
    if (!parseUint16Input(server.arg("shelly_range_max"), 1, kShellyDimmerRangeMaxLimit, range_max) ||
        range_max <= config.shelly_dimmer_range_min) {
      server.send(400, F("text/plain"), F("Invalid Shelly range maximum"));
      return;
    }
    if (config.shelly_dimmer_range_max != range_max) {
      config.shelly_dimmer_range_max = static_cast<uint8_t>(range_max);
      light_settings_changed = true;
      shelly_settings_changed = true;
    }
  }

  if (light_settings_changed) {
    if (!commitConfig()) {
      server.send(500, F("text/plain"), F("Could not save light settings"));
      return;
    }
    if (shelly_settings_changed) {
      shellyDimmerSendSettings();
      updateLightOutputs();
    }
  }

  if (server.hasArg("_inline")) {
    server.send(204, F("text/plain"), "");
    return;
  }

  server.sendHeader(F("Location"), F("/"), true);
  server.send(303, F("text/plain"), "");
}

bool executeDeviceCommand(const char *raw, size_t cmd_len, const char *arg, size_t arg_len, String &out, String &error) {
  if (!raw || !arg || cmd_len == 0) {
    error = F("Invalid cmnd");
    return false;
  }

  while (arg_len > 0) {
    const char c = arg[0];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    arg++;
    arg_len--;
  }
  while (arg_len > 0) {
    const char c = arg[arg_len - 1];
    if (c != ' ' && c != '\t' && c != '\r' && c != '\n') break;
    arg_len--;
  }

  uint8_t relay = 0;
  char response_key[12];
  if (parsePowerCommand(raw, cmd_len, relay, response_key, sizeof(response_key))) {
    bool on = false;
    if (relay < kMaxRelays && hasPin(runtime_template.relays[relay])) {
      if (arg_len == 0) {
        on = relay_state[relay];
      } else {
        uint8_t state = kPowerStateOff;
        if (!parsePowerState(arg, arg_len, state)) {
          error = F("Invalid power state");
          return false;
        }
        on = state == kPowerStateToggle ? !relay_state[relay] : state == kPowerStateOn;
        setRelay(relay, on);
        updateDeviceLeds(true);
      }
    } else if (relay == 0 && light.present) {
      if (arg_len == 0) {
        on = light.power;
      } else {
        uint8_t state = kPowerStateOff;
        if (!parsePowerState(arg, arg_len, state)) {
          error = F("Invalid power state");
          return false;
        }
        on = state == kPowerStateToggle ? !light.power : state == kPowerStateOn;
        setLightPower(on);
      }
    } else {
      error = F("Invalid relay");
      return false;
    }

    out.reserve(24);
    out += F("{\"");
    out += response_key;
    out += F("\":\"");
    out += (on ? F("ON") : F("OFF"));
    out += F("\"}");
    return true;
  }

  if (commandEquals(raw, cmd_len, "dimmer")) {
    if (!light.present) {
      error = F("No light output is configured");
      return false;
    }
    if (arg_len > 0) {
      String value = commandArgument(arg, arg_len);
      value.trim();
      uint16_t dimmer = 0;
      if (!parseUint16Input(value, kLightDimmerOff, kLightDimmerMax, dimmer)) {
        error = F("Invalid dimmer");
        return false;
      }
      setLightDimmer(dimmer);
    }
    out.reserve(20);
    out += F("{\"Dimmer\":");
    out += light.dimmer;
    out += F("}");
    return true;
  }

  if (commandEquals(raw, cmd_len, "dimmerrange")) {
    if (!runtime_template.shelly_dimmer) {
      error = F("No Shelly dimmer is configured");
      return false;
    }
    if (arg_len > 0) {
      uint8_t range_min = config.shelly_dimmer_range_min;
      uint8_t range_max = config.shelly_dimmer_range_max;
      if (!parseDimmerRangeCommandArgument(arg, arg_len, range_min, range_max)) {
        error = F("Invalid dimmer range");
        return false;
      }
      config.shelly_dimmer_range_min = range_min;
      config.shelly_dimmer_range_max = range_max;
      if (!commitConfig()) {
        error = F("Could not save dimmer range");
        return false;
      }
      shellyDimmerSendSettings();
      updateLightOutputs();
    }
    out.reserve(42);
    out += F("{\"DimmerRange\":{\"Min\":");
    out += String(config.shelly_dimmer_range_min);
    out += F(",\"Max\":");
    out += String(config.shelly_dimmer_range_max);
    out += F("}}");
    return true;
  }

  if (commandEquals(raw, cmd_len, "shdleadingedge")) {
    if (!runtime_template.shelly_dimmer) {
      error = F("No Shelly dimmer is configured");
      return false;
    }
    if (arg_len > 0) {
      String value = commandArgument(arg, arg_len);
      value.trim();
      uint16_t leading_edge = 0;
      if (!parseUint16Input(value, 0, 1, leading_edge)) {
        error = F("Invalid Shelly edge mode");
        return false;
      }
      config.shelly_dimmer_edge = leading_edge ? kShellyDimmerEdgeLeading : kShellyDimmerEdgeTrailing;
      if (!commitConfig()) {
        error = F("Could not save Shelly edge mode");
        return false;
      }
      shellyDimmerSendSettings();
      updateLightOutputs();
    }
    out.reserve(24);
    out += F("{\"ShdLeadingEdge\":");
    out += String(config.shelly_dimmer_edge == kShellyDimmerEdgeLeading ? 1 : 0);
    out += F("}");
    return true;
  }

  if (commandEquals(raw, cmd_len, "ct") || commandEquals(raw, cmd_len, "colortemperature")) {
    if (!light.present) {
      error = F("No light output is configured");
      return false;
    }
    if (arg_len > 0) {
      String value = commandArgument(arg, arg_len);
      value.trim();
      uint16_t ct = 0;
      if (!parseUint16Input(value, kLightCtMin, kLightCtMax, ct)) {
        error = F("Invalid color temperature");
        return false;
      }
      setLightCt(ct);
    }
    out.reserve(16);
    out += F("{\"CT\":");
    out += light.ct;
    out += F("}");
    return true;
  }

  error = F("Unsupported command");
  return false;
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
  if (cmd_len == 0) {
    server.send(400, F("text/plain"), F("Invalid cmnd"));
    return;
  }
  const size_t state_len = state_start < total_len ? total_len - state_start : 0;

  String out;
  String error;
  if (!executeDeviceCommand(raw, cmd_len, raw + state_start, state_len, out, error)) {
    server.send(400, F("text/plain"), error);
    return;
  }
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

void handleSettingsExport() {
  String out;
  out.reserve(5200);
  appendSettingsExportJson(out);
  String disposition = F("attachment; filename=\"mymota-settings-");
  disposition += chipIdHex();
  disposition += F(".json\"");
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.sendHeader(F("Content-Disposition"), disposition);
  server.send(200, F("application/json"), out);
}

void sendApiSettingsError(uint16_t status, const __FlashStringHelper *message) {
  String out;
  out.reserve(120);
  out += F("{\"ok\":false,\"error\":\"");
  out += message;
  out += F("\"}");
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(status, F("application/json"), out);
}

void finishApiSettingsUpdate(const StoredConfig &before, const StoredConfig &candidate, const SettingsImportStats &stats) {
  if (stats.applied == 0) {
    String out;
    out.reserve(260);
    out += F("{\"ok\":false,\"error\":\"No API settings were applied\",\"skipped\":");
    out += stats.skipped;
    if (stats.skipped_fields.length()) {
      out += F(",\"skipped_fields\":\"");
      out += settingsJsonEscape(stats.skipped_fields.c_str());
      out += F("\"");
    }
    out += F("}");
    server.sendHeader(F("Cache-Control"), F("no-store"));
    server.send(400, F("application/json"), out);
    return;
  }

  const bool input_changed = inputConfigDiffers(before, candidate);
  config = candidate;
  if (!commitConfig()) {
    config = before;
    sendApiSettingsError(500, F("Could not save API settings"));
    return;
  }
  if (input_changed) updateDeviceLeds(true);

  String out;
  out.reserve(2200);
  out += F("{\"ok\":true,\"applied\":");
  out += stats.applied;
  out += F(",\"skipped\":");
  out += stats.skipped;
  if (stats.skipped_fields.length()) {
    out += F(",\"skipped_fields\":\"");
    out += settingsJsonEscape(stats.skipped_fields.c_str());
    out += F("\"");
  }
  out += F(",\"settings\":");
  appendApiSettingsJson(out);
  out += F("}");
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), out);
}

void handleApiSettingsGet() {
  StoredConfig before = config;
  StoredConfig candidate = config;
  SettingsImportStats stats = {0, 0, String()};
  if (applyApiSettingsGetArgs(candidate, stats)) {
    finishApiSettingsUpdate(before, candidate, stats);
    return;
  }

  String out;
  out.reserve(1800);
  appendApiSettingsJson(out);
  server.sendHeader(F("Cache-Control"), F("no-store"));
  server.send(200, F("application/json"), out);
}

void handleApiSettingsUpdate() {
  String body = server.arg("plain");
  body.trim();
  if (body.length() == 0 || body.length() > kSettingsImportJsonMaxLen) {
    sendApiSettingsError(400, F("Missing or oversized JSON body"));
    return;
  }

  DynamicJsonDocument doc(kApiSettingsDocCapacity);
  const DeserializationError json_error = deserializeJson(doc, body);
  if (json_error) {
    sendApiSettingsError(400, F("Invalid JSON body"));
    return;
  }

  JsonObjectConst root = doc.as<JsonObjectConst>();
  if (root.isNull()) {
    sendApiSettingsError(400, F("JSON root must be an object"));
    return;
  }

  StoredConfig before = config;
  StoredConfig candidate = config;
  SettingsImportStats stats = {0, 0, String()};
  applyApiInputSettings(root, candidate, stats);

  finishApiSettingsUpdate(before, candidate, stats);
}

void appendSettingsImportSummary(String &page, const SettingsImportStats &stats) {
  page += F("<p><code>");
  page += String(stats.applied);
  page += F("</code> setting fields imported");
  if (stats.skipped > 0) {
    page += F(", <code>");
    page += String(stats.skipped);
    page += F("</code> skipped.</p><p class='hint'>Skipped: ");
    page += htmlEscape(stats.skipped_fields);
    page += F("</p>");
  } else {
    page += F(".</p>");
  }
}

void handleSettingsImport() {
  String settings_json = server.arg("settings_json");
  if (settings_json.length() == 0 && server.hasArg("plain")) {
    settings_json = server.arg("plain");
  }
  settings_json.trim();
  if (settings_json.length() == 0) {
    server.send(400, F("text/plain"), F("Settings JSON is empty"));
    return;
  }
  if (settings_json.length() > kSettingsImportJsonMaxLen) {
    server.send(400, F("text/plain"), F("Settings JSON is too large"));
    return;
  }

  DynamicJsonDocument doc(kSettingsImportDocCapacity);
  const DeserializationError json_error = deserializeJson(doc, settings_json);
  if (json_error) {
    String msg = F("Settings JSON parse failed: ");
    msg += json_error.c_str();
    server.send(400, F("text/plain"), msg);
    return;
  }

  JsonObjectConst root = doc.as<JsonObjectConst>();
  if (root.isNull()) {
    server.send(400, F("text/plain"), F("Settings JSON root must be an object"));
    return;
  }

  const char *format = root["format"] | "";
  if (strcmp(format, "mymota-settings") != 0) {
    server.send(400, F("text/plain"), F("Unsupported settings format"));
    return;
  }
  uint16_t format_version = 0;
  if (!settingsReadUint16(root["format_version"], 1, 65535U, format_version) ||
      format_version != kSettingsFormatVersion) {
    server.send(400, F("text/plain"), F("Unsupported settings format version"));
    return;
  }

  StoredConfig before = config;
  StoredConfig candidate = config;
  SettingsImportStats stats = {0, 0, String()};

  importSettingsTemplate(root, candidate, stats);
  RuntimeTemplate candidate_runtime{};
  decodeTemplateConfigInto(candidate, candidate_runtime);
  importSettingsMqtt(root, candidate, stats);
  importSettingsEnergy(root, candidate, stats);
  importSettingsLight(root, candidate, candidate_runtime, stats);
  importSettingsLeds(root, candidate, candidate_runtime, stats);
  importSettingsRelayEnforcement(root, candidate, candidate_runtime, stats);
  importSettingsInputs(root, candidate, candidate_runtime, stats);

  String page;
  page.reserve(1200);
  appendHeader(page, F("myMota Settings"));
  if (stats.applied == 0) {
    page += F("<p class='bad'>No valid settings were imported.</p>");
    appendSettingsImportSummary(page, stats);
    page += F("<p><a href='/'>Back</a></p>");
    appendFooter(page);
    sendHtml(page);
    return;
  }

  const bool template_changed = templatesDiffer(before, candidate);
  const bool mqtt_changed = mqttConfigDiffers(before, candidate);
  const bool energy_changed = energyConfigDiffers(before, candidate);
  const bool light_changed = lightConfigDiffers(before, candidate);
  const bool led_changed = ledConfigDiffers(before, candidate);
  const bool relay_enforcement_changed = relayEnforcementConfigDiffers(before, candidate);
  const bool input_changed = inputConfigDiffers(before, candidate);

  config = candidate;
  if (!commitConfig()) {
    config = before;
    server.send(500, F("text/plain"), F("Could not save imported settings"));
    return;
  }

  if (template_changed) {
    decodeTemplateConfig();
    page += F("<p class='ok'>Settings imported. Rebooting.</p>");
    appendSettingsImportSummary(page, stats);
    if (runtime_template.unsupported_count) {
      page += F("<p class='bad'>The imported template contains unsupported GPIO functions. Check the Template section after reboot.</p>");
    }
    page += F("<p>The page will return to the dashboard when the device is reachable again.</p>");
    appendFooter(page, false, true);
    sendHtml(page);
    scheduleRestart(1200);
    return;
  }

  if (mqtt_changed) resetMqttRuntimeState();
  if (energy_changed) {
    last_mqtt_energy_publish = 0;
    last_mqtt_energy_power = NAN;
    last_mqtt_energy_report_reason = kMqttEnergyReportReasonNone;
  }
  if (light_changed) {
    loadLightStateFromConfig();
    if (runtime_template.shelly_dimmer) {
      shellyDimmerSendSettings();
    }
    updateLightOutputs();
  }
  if (relay_enforcement_changed) refreshRelayEnforcementRuntime(true);
  if (led_changed || input_changed) updateDeviceLeds(true);

  page += F("<p class='ok'>Settings imported.</p>");
  appendSettingsImportSummary(page, stats);
  page += F("<p><a href='/'>Back</a></p>");
  appendFooter(page);
  sendHtml(page);
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
  out += F("},\"light\":");
  if (light.present) {
    out += F("{\"power\":");
    out += light.power ? F("true") : F("false");
    out += F(",\"dimmer\":");
    out += light.dimmer;
    out += F(",\"ct\":");
    out += light.ct;
    out += F(",\"ct_supported\":");
    out += lightSupportsColorTemperature() ? F("true") : F("false");
    out += F(",\"on_dimmer\":");
    out += config.light_on_dimmer;
    out += F(",\"pwm\":[");
    for (uint8_t i = 0; i < runtime_template.pwm_count && i < kMaxLightPwms; i++) {
      if (i) out += F(",");
      if (!hasPin(runtime_template.light_pwm[i])) {
        out += F("null");
      } else {
        out += F("{\"pin\":");
        out += runtime_template.light_pwm[i].pin;
        out += F(",\"duty\":");
        out += lightPwmDuty(i);
        out += F("}");
      }
    }
    out += F("]");
    if (runtime_template.shelly_dimmer) {
      out += F(",\"shelly_dimmer\":{\"mcu_version\":\"");
      out += String(shelly_dimmer.version_major);
      out += F(".");
      out += String(shelly_dimmer.version_minor);
      out += F("\",\"hw_version\":");
      out += String(shelly_dimmer.hw_version);
      out += F(",\"actual_brightness\":");
      out += shelly_dimmer.actual_brightness;
      out += F(",\"requested_brightness\":");
      out += shelly_dimmer.requested_brightness == 0xffffU ? 0 : shelly_dimmer.requested_brightness;
      out += F(",\"edge\":\"");
      out += shellyDimmerEdgeName(config.shelly_dimmer_edge);
      out += F("\",\"range_min\":");
      out += String(config.shelly_dimmer_range_min);
      out += F(",\"range_max\":");
      out += String(config.shelly_dimmer_range_max);
      out += F(",\"last_rx_ms_ago\":");
      if (shelly_dimmer.last_rx_ms == 0) {
        out += F("null");
      } else {
        out += millis() - shelly_dimmer.last_rx_ms;
      }
      out += F("}");
    }
    out += F("}");
  } else {
    out += F("null");
  }
  out += F(",\"power\":[");
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
      out += F(",\"state\":\"");
      out += inputStateName(i, button_state[i].stable_pressed);
      out += F("\",\"kind\":\"");
      out += inputKindName(i);
      out += F("\",\"mode\":\"");
      out += inputModeName(i);
      out += F("\",\"on_level\":\"");
      out += effectiveInputOnLevel(i) == kInputOnLevelHigh ? F("high") : F("low");
      out += F("\"");
      if (effectiveInputMode(i) == kInputModeSwitch) {
        uint8_t relay = 0;
        if (inputRelayTarget(i, relay)) {
          out += F(",\"target_relay\":");
          out += relay + 1;
        }
      }
      out += F(",\"press_action\":\"");
      out += buttonActionName(config.button_press_action[i]);
      out += F("\"");
      if (config.button_press_action[i] == kButtonActionRelayToggle) {
        uint8_t relay = 0;
        if (buttonRelayTarget(i, false, relay)) {
          out += F(",\"press_relay\":");
          out += relay + 1;
        }
      }
      out += F(",\"hold_action\":\"");
      out += buttonActionName(config.button_hold_action[i]);
      out += F("\"");
      if (config.button_hold_action[i] == kButtonActionRelayToggle) {
        uint8_t relay = 0;
        if (buttonRelayTarget(i, true, relay)) {
          out += F(",\"hold_relay\":");
          out += relay + 1;
        }
      }
      out += F("}");
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
    out += F("{\"driver\":\"");
    if (energy.driver == kEnergyDriverAde7953) {
      out += F("ade7953");
    } else if (energy.driver == kEnergyDriverShellyDimmer) {
      out += F("shelly_dimmer");
    } else {
      out += energy.hjl ? F("hjl_bl0937") : F("hlw8012");
    }
    out += F("\",\"voltage\":");
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
    out += F(",\"last_mqtt_report_ms_ago\":");
    if (last_mqtt_energy_publish == 0) {
      out += F("null");
    } else {
      out += millis() - last_mqtt_energy_publish;
    }
    out += F(",\"last_mqtt_report_reason\":\"");
    out += mqttEnergyReportReasonName(last_mqtt_energy_report_reason);
    out += F("\"");
    if (energy.channel_count > 1) {
      out += F(",\"channels\":[");
      for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
        if (i) out += F(",");
        out += F("{\"voltage\":");
        out += String(energy.channel[i].voltage, 1);
        out += F(",\"current\":");
        out += String(energy.channel[i].current, 3);
        out += F(",\"power\":");
        out += String(energy.channel[i].power, 1);
        out += F("}");
      }
      out += F("]");
    }
    out += F(",\"debug\":{");
    if (energy.driver == kEnergyDriverAde7953) {
      out += F("\"i2c_errors\":");
      out += energy.i2c_error_count;
      out += F(",\"last_success_ms_ago\":");
      out += millis() - energy.last_success_ms;
      out += F(",\"acc_mode\":");
      out += energy.ade7953_acc_mode;
      out += F(",\"skip_reads\":");
      out += energy.ade7953_skip_reads;
      out += F(",\"sample_ms\":");
      out += energy.ade7953_sample_ms;
      out += F(",\"raw\":[");
      for (uint8_t i = 0; i < energy.channel_count && i < kEnergyMaxChannels; i++) {
        if (i) out += F(",");
        out += F("{\"vrms\":");
        out += energy.channel[i].voltage_raw;
        out += F(",\"irms\":");
        out += energy.channel[i].current_raw;
        out += F(",\"active\":");
        out += energy.channel[i].active_power_raw;
        out += F(",\"apparent\":");
        out += energy.channel[i].apparent_power_raw;
        out += F("}");
      }
      out += F("]");
    } else if (energy.driver == kEnergyDriverShellyDimmer) {
      out += F("\"last_success_ms_ago\":");
      out += millis() - energy.last_success_ms;
      out += F(",\"mcu_version\":\"");
      out += String(shelly_dimmer.version_major);
      out += F(".");
      out += String(shelly_dimmer.version_minor);
      out += F("\",\"hw_version\":");
      out += String(shelly_dimmer.hw_version);
      out += F(",\"rx_ms_ago\":");
      if (shelly_dimmer.last_rx_ms == 0) {
        out += F("null");
      } else {
        out += millis() - shelly_dimmer.last_rx_ms;
      }
      out += F(",\"timeouts\":");
      out += shelly_dimmer.timeout_count;
      out += F(",\"errors\":");
      out += shelly_dimmer.error_count;
      out += F(",\"raw\":{\"wattage\":");
      out += shelly_dimmer.wattage_raw;
      out += F(",\"voltage\":");
      out += shelly_dimmer.voltage_raw;
      out += F(",\"current\":");
      out += shelly_dimmer.current_raw;
      out += F("}");
    } else {
      out += F("\"cf_us\":");
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
    }
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
    persistLightConfig(true);
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

void stopAp() {
  if (!ap_started) return;
  if (WiFi.softAPdisconnect(true)) {
    ap_started = false;
    last_ap_attempt = 0;
  }
}

void applyPhyMode(uint8_t phy_mode) {
  phy_mode = sanitizePhyMode(phy_mode);
  if (phy_mode != kPhyModeAuto) {
    WiFi.setPhyMode(static_cast<WiFiPhyMode_t>(phy_mode));
  }
}

void beginWifiReconnect(uint32_t now) {
  if (!config_ok || WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(ap_started ? WIFI_AP_STA : WIFI_STA);
  applyPhyMode(config.phy_mode);
  WiFi.begin(config.ssid, config.password);
  last_wifi_begin_attempt = now;
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
  last_wifi_begin_attempt = millis();
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

  disconnected_since = millis();
  disconnected_timer_active = true;
  if (connectWifiWithPhy(config.phy_mode, kConnectTimeoutMs)) {
    sta_connected_once = true;
    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    if (config.phy_mode != kPhyModeAuto && config.phy_mode != kPhyModeFailsafe) {
      if (connectWifiWithPhy(kPhyModeFailsafe, kConnectTimeoutMs)) {
        sta_connected_once = true;
        return;
      }
    }
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
  server.on(F("/relay-enforcement"), HTTP_POST, handleRelayEnforcementSave);
  server.on(F("/buttons"), HTTP_POST, handleButtonSave);
  server.on(F("/power"), HTTP_POST, handlePowerSave);
  server.on(F("/light"), HTTP_POST, handleLightSave);
  server.on(F("/cm"), HTTP_GET, handleCmnd);
  server.on(F("/reboot"), HTTP_GET, handleReboot);
  server.on(F("/factory-reset"), HTTP_POST, handleFactoryReset);
  server.on(F("/settings/export"), HTTP_GET, handleSettingsExport);
  server.on(F("/settings/import"), HTTP_POST, handleSettingsImport);
  server.on(F("/api/settings"), HTTP_GET, handleApiSettingsGet);
  server.on(F("/api/settings"), HTTP_POST, handleApiSettingsUpdate);
  server.on(F("/api/settings"), HTTP_PUT, handleApiSettingsUpdate);
  server.on(F("/api/settings"), HTTP_PATCH, handleApiSettingsUpdate);
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
    sta_connected_once = true;
    stopAp();
    disconnected_since = 0;
    disconnected_timer_active = false;
    last_wifi_begin_attempt = 0;
    return;
  }
  const uint32_t now = millis();
  if (!disconnected_timer_active) {
    disconnected_since = now;
    disconnected_timer_active = true;
    last_wifi_begin_attempt = now;
  }
  if (now - last_wifi_begin_attempt >= kWifiReconnectBeginMs) {
    beginWifiReconnect(now);
  }
  if (!sta_connected_once && !ap_started && now - disconnected_since >= kInitialFallbackApMs) {
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

  if (!shelly_dimmer.serial_claimed) {
    Serial.printf("HTTP server started; STA %s AP %s\n",
                  WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "not-connected",
                  ap_started ? WiFi.softAPIP().toString().c_str() : "off");
  }
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
    persistLightConfig(true);
    persistEnergyTotal(true);
    delay(50);
    ESP.restart();
  }

  recordLoopPerf(loop_started_us, micros());
  yield();
}
