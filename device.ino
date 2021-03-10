/**
   Azure IoT Central example for esp32-azure-kit.
*/

// Include Libraries
extern "C" {
#include "soc/pcnt_struct.h"
}
#include "driver/pcnt.h"

byte pulsePin = 35;

#include <WiFi.h>

#include "AzureIotHub.h"
#include "src/parson.h"
#include "src/sensor_manager.h"
#include "src/led.h"
#include <Adafruit_ADS1015.h>
#define _GNU_SOURCE
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Please input the SSID and password of WiFi
const char *ssid = "yehiaraslan";
const char *password = "Pentiums911";
// Set your Static IP address
IPAddress local_IP(192, 168, 1, 184);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

/*String containing Hostname, Device Id & Device Key in the format:                         */
/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessKey=<device_key>"                */
/*  "HostName=<host_name>;DeviceId=<device_id>;SharedAccessSignature=<device_sas_token>"    */
static const char* connectionString = "HostName=iotc-076b63fb-27fc-4698-98ff-02a06046e3ea.azure-devices.net;DeviceId=1des5emt3vy;SharedAccessKey=tLvrvOmASW/dyTKMLlZyPvx/HCgr+h+423dXwMjdCAg=";
#define TELEMETRY_SEND_INTERVAL 5000  // telemetry data sent every 10 minutes
#define PROPERTY_SEND_INTERVAL  60000 * 60 * 24 // property data sent every day minutes
#define SENSOR_READ_INTERVAL        1000      // read sensors every 1 seconds
#define FLOW_READ_INTERVAL          1000      // read sensors every 1 second


typedef struct EVENT_MESSAGE_INSTANCE_TAG
{
  IOTHUB_MESSAGE_HANDLE messageHandle;
  size_t messageTrackingId; // For tracking the messages within the user callback.
} EVENT_MESSAGE_INSTANCE_TAG;

IOTHUB_CLIENT_LL_HANDLE iotHubClientHandle = NULL;
static char propText[1024];
static char msgText[1024];
static int trackingId = 0;

static int fanSpeed = 10;
static int temThreshold = 30;
static bool fan_running = false;
static bool fan_running_with_command = false;
static bool needs_reconnect = false;

static bool hasIoTHub = false;
static bool hasWifi = false;
static uint64_t send_interval_ms;
static uint64_t check_interval_ms;


/////////////////////////////////////  Valve A Variables /////////////////////////////////////////////
bool sendMessage  = true;
bool startValve = false;
unsigned long startTimer = 0;
unsigned long totalMilliLitres = 0;
float flowRate = 0.0;
float VarVWC = 0;
int pulseCount = 0;
unsigned long elapsedTime;
int SolenoidValve_valSensorMax = 60;
int SolenoidValve_valSensorMin = 40;
bool ManualOverride = false;
int telemetrySendInterval = TELEMETRY_SEND_INTERVAL;
int Counter = 1;
unsigned long oldTime;
unsigned long lastFlowReadMillis = 0;
unsigned long lastMillis = 0;
unsigned long lastTelemetryMillis = 0;
unsigned long lastPropertyMillis = 0;
unsigned long lastSensorReadMillis = 0;
unsigned int flowMilliLitres;


struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button1 = {35, 0, false};

void IRAM_ATTR isr(void* arg) {
    Button* s = static_cast<Button*>(arg);
    s->numberKeyPresses += 1;
    s->pressed = true;
}

void IRAM_ATTR isr() {
    
}

// Assign output variables to GPIO pins
const int RelayOutputPin = 13;
const int Power12V= 23; // Power for Sensor and flow meter 12Volts

Adafruit_ADS1115 ads(0x49);  /* Use this for the 16-bit version */
int reading[8] = {0};
String adcString[8];
int analog_value = 0;
int flowcountint = 0;
int16_t flowCounter = 0;
int16_t Pulses = 0;
#define ANALOG_PIN_0 36
#define PCNT_TEST_UNIT PCNT_UNIT_0
#define PCNT_H_LIM_VAL 32767
#define PCNT_L_LIM_VAL -1
////////////////////////////////////////////////////////////////////

static bool showHumitureScreen = true;

// forward declarations
static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE update_state, const unsigned char *payLoad, size_t size, void *userContextCallback);
static int deviceMethodCallback(const char *method_name, const unsigned char *payload, size_t size, unsigned char **response, size_t *response_size, void *userContextCallback);
static void connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void *user_context);
static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *userContextCallback);
static void reportedStateCallback(int status_code, void *userContextCallback);

static bool initIotHubClient(void)
{
  Serial.println("initIotHubClient Start!");
  if (platform_init() != 0)
  {
    Serial.println("Failed to initialize the platform.");
    return false;
  }

  if ((iotHubClientHandle = IoTHubClient_LL_CreateFromConnectionString(connectionString, MQTT_Protocol)) == NULL)
  {
    Serial.println("ERROR: iotHubClientHandle is NULL!");
    return false;
  }

  IoTHubClient_LL_SetRetryPolicy(iotHubClientHandle, IOTHUB_CLIENT_RETRY_EXPONENTIAL_BACKOFF, 1200);
  bool traceOn = true;
  IoTHubClient_LL_SetOption(iotHubClientHandle, "logtrace", &traceOn);

  // Setting twin call back for desired properties receiving.
  if (IoTHubClient_LL_SetDeviceTwinCallback(iotHubClientHandle, deviceTwinCallback, NULL) != IOTHUB_CLIENT_OK)
  {
    Serial.println("IoTHubClient_LL_SetDeviceTwinCallback..........FAILED!");
    return false;
  }

  // Setting direct method callback for direct method calls receiving
  if (IoTHubClient_LL_SetDeviceMethodCallback(iotHubClientHandle, deviceMethodCallback, NULL) != IOTHUB_CLIENT_OK)
  {
    Serial.println("IoTHubClient_LL_SetDeviceMethodCallback..........FAILED!");
    return false;
  }

  // Connection status change callback
  if (IoTHubClient_LL_SetConnectionStatusCallback(iotHubClientHandle, connectionStatusCallback, NULL) != IOTHUB_CLIENT_OK)
  {
    Serial.println("IoTHubClient_LL_SetDeviceMethodCallback..........FAILED!");
    return false;
  }
  Serial.println("initIotHubClient End!");

  // toggle azure led to default off
  toggle_azure_led(0);
  return true;
}

static void closeIotHubClient()
{
  if (iotHubClientHandle != NULL)
  {
    IoTHubClient_LL_Destroy(iotHubClientHandle);
    platform_deinit();
    iotHubClientHandle = NULL;
  }
  Serial.println("closeIotHubClient!");
}

static void sendTelemetry(const char *payload)
{
  if (needs_reconnect)
  {
    closeIotHubClient();
    initIotHubClient();
    needs_reconnect = false;
  }

  EVENT_MESSAGE_INSTANCE_TAG *thisMessage = (EVENT_MESSAGE_INSTANCE_TAG *)malloc(sizeof(EVENT_MESSAGE_INSTANCE_TAG));
  thisMessage->messageHandle = IoTHubMessage_CreateFromByteArray((const unsigned char *)payload, strlen(payload));

  if (thisMessage->messageHandle == NULL)
  {
    Serial.println("ERROR: iotHubMessageHandle is NULL!");
    free(thisMessage);
    return;
  }

  thisMessage->messageTrackingId = trackingId++;

  MAP_HANDLE propMap = IoTHubMessage_Properties(thisMessage->messageHandle);

  (void)sprintf_s(propText, sizeof(propText), "PropMsg_%zu", trackingId);
  if (Map_AddOrUpdate(propMap, "PropName", propText) != MAP_OK)
  {
    Serial.println("ERROR: Map_AddOrUpdate Failed!");
  }

  // send message to the Azure Iot hub
  if (IoTHubClient_LL_SendEventAsync(iotHubClientHandle,
                                     thisMessage->messageHandle, sendConfirmationCallback, thisMessage) != IOTHUB_CLIENT_OK)
  {
    Serial.println("ERROR: IoTHubClient_LL_SendEventAsync..........FAILED!");
    return;
  }

  /* Turn on Azure LED */
  toggle_azure_led(1);

  IoTHubClient_LL_DoWork(iotHubClientHandle);
  Serial.println("IoTHubClient sendTelemetry completed!");
}

static void sendReportedProperty(const char *payload)
{
  if (needs_reconnect)
  {
    closeIotHubClient();
    initIotHubClient();
    needs_reconnect = false;
  }
  if (IoTHubClient_LL_SendReportedState(iotHubClientHandle, (const unsigned char *)payload,
                                        strlen((const char *)payload), reportedStateCallback, NULL) != IOTHUB_CLIENT_OK)
  {
    Serial.println("ERROR: IoTHubClient sendReportedProperty..........FAILED!");
    return;
  }

  Serial.println("IoTHubClient sendReportedProperty completed!");
}

static void set_device_desired_property(const char *name, int value)
{
  if (strcmp("VWCMin", name) == 0)
  {
    SolenoidValve_valSensorMin = value;
  }
  else if (strcmp("VWCMax", name) == 0)
  {
    SolenoidValve_valSensorMax = value;
  }
}

static void deviceTwinCallback(DEVICE_TWIN_UPDATE_STATE update_state, const unsigned char *payLoad, size_t size, void *userContextCallback)
{
  Serial.print("Device Twin Callback method with : ");
  Serial.println((const char *)payLoad);

  JSON_Value *root_value = json_parse_string((const char *)payLoad);
  JSON_Object *root_object = json_value_get_object(root_value);

  if (update_state == DEVICE_TWIN_UPDATE_PARTIAL)
  {
    Serial.print("DEVICE_TWIN_UPDATE_STATE is :");
    Serial.println(update_state);

    int version = (uint8_t)json_object_dotget_number(root_object, "$version");
    for (int i = 0, count = json_object_get_count(root_object); i < count; i++)
    {
      const char *partialName = json_object_get_name(root_object, i);
      if (partialName != NULL && partialName[0] != '$')
      {
        JSON_Object *partialObject = json_object_dotget_object(root_object, partialName);
        int partialValue = (uint8_t)json_object_dotget_number(partialObject, "value");

        (void)sprintf_s(propText, sizeof(propText),
                        "{\"%s\":{\"value\":%d,\"status\":\"completed\",\"desiredVersion\":%d}}",
                        partialName, partialValue, version);

        sendReportedProperty(propText);
        set_device_desired_property(partialName, partialValue);
      }
    }
  }
  else
  {
    JSON_Object *desired, *reported;

    desired = json_object_dotget_object(root_object, "desired");
    reported = json_object_dotget_object(root_object, "reported");

    int version = (uint8_t)json_object_dotget_number(desired, "$version");
    for (int i = 0, count = json_object_get_count(desired); i < count; i++)
    {
      const char *itemName = json_object_get_name(desired, i);
      if (itemName != NULL && itemName[0] != '$')
      {
        int desiredVersion = 0, value = 0;

        JSON_Object *itemObject = json_object_dotget_object(desired, itemName);
        value = (uint8_t)json_object_dotget_number(itemObject, "value");
        set_device_desired_property(itemName, value);
        JSON_Object *keyObject = json_object_dotget_object(reported, itemName);
        if (keyObject != NULL)
        {
          desiredVersion = (uint8_t)json_object_dotget_number(keyObject, "desiredVersion");
        }

        if (keyObject != NULL && (version == desiredVersion))
        {
          Serial.print("key: ");
          Serial.print(itemName);
          Serial.println(" found in reported and versions match.");
        }
        else
        {
          Serial.print("key: ");
          Serial.print(itemName);
          Serial.println(" either not found in reported or versions do not match.");

          (void)sprintf_s(propText, sizeof(propText),
                          "{\"%s\":{\"value\":%d,\"status\":\"completed\",\"desiredVersion\":%d}}",
                          itemName, value, version);
          sendReportedProperty(propText);
        }
      }
    }
  }
  json_value_free(root_value);
}

static int deviceMethodCallback(const char *method_name, const unsigned char *payload, size_t size, unsigned char **response, size_t *response_size, void *userContextCallback)
{
  (void)userContextCallback;
  (void)payload;
  (void)size;

  int result = 200;

  Serial.print("Executed direct method: ");
  Serial.println(method_name);

  Serial.print("Executed direct method payload: ");
  Serial.println((const char *)payload);

  char search_for_true[] = "true";
  char search_for_false[] = "false";
// received payload
 
  // pointer to value in payload; will be 'calculated'
  char *val;

  
 if (strcmp("COMMANDVALVESTATUSA", method_name) == 0)
  {
    JSON_Value *root_value = json_parse_string((const char *)payload);
    JSON_Object *root_object = json_value_get_object(root_value);
    JSON_Value *displayedValue;     
    displayedValue = json_object_dotget_value(root_object, "displayedValue");
    
       
// find colon; note the casting to a character pointer
  val = strchr((char*)payload, 't');
  // if colon found
  if (val != NULL)
  {
    // replace colon by null terminator
    *val = '\0';
    // let val point to the actual value
    val++;

    // print the 'label'; again note the cast to a character pointer
    Serial.println((char*)payload);
    // print the value
    Serial.println(val);
         ManualOverride = true;

  }
  else
  {
    Serial.println("Could not  find colon");
          ManualOverride = false;    

  }
   
  
  json_value_free(root_value);

  }
 
  const char *responseMessage = "{ \"Response\": \"Successful\" }";
  *response_size = strlen(responseMessage);
  *response = (unsigned char *)strdup(responseMessage);

  return result;
}

static void connectionStatusCallback(IOTHUB_CLIENT_CONNECTION_STATUS result, IOTHUB_CLIENT_CONNECTION_STATUS_REASON reason, void *user_context)
{
  (void)reason;
  (void)user_context;

  Serial.print("iotHubClient connectionStatusCallback result: ");
  Serial.println(result);

  if (result == IOTHUB_CLIENT_CONNECTION_AUTHENTICATED)
  {
    Serial.println("Network connection.");
  }
  else
  {
    Serial.println("No network connection.");
    Serial.println("Needs reconnect to the IoT Hub.");
    needs_reconnect = true;
  }
}

static void sendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void *userContextCallback)
{
  EVENT_MESSAGE_INSTANCE_TAG *eventInstance = (EVENT_MESSAGE_INSTANCE_TAG *)userContextCallback;
  size_t id = eventInstance->messageTrackingId;

  Serial.print("Confirmation received for message tracking id = ");
  Serial.print(id);
  Serial.print(" with result = ");
  Serial.println(ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));

  IoTHubMessage_Destroy(eventInstance->messageHandle);
  free(eventInstance);
  /* Turn off Azure LED */
  toggle_azure_led(0);
}

static void reportedStateCallback(int status_code, void *userContextCallback)
{
  (void)userContextCallback;
  Serial.print("Device Twin reported properties update completed with result: ");
  Serial.println(status_code);
}

static void SetSensorState(float temperature)
{
  if (temperature < temThreshold)
  {
    if (fan_running && !fan_running_with_command)
    {
      stop_motor();
      fan_running = false;
    }
  }
  else
  {
    if (!fan_running && !fan_running_with_command)
    {
      start_motor_with_speed(fanSpeed);
      fan_running = true;
    }
  }
}

static void button_tap_cb(void *arg)
{
  Serial.println("Button tapped");
  // Toggle to show huimiture screen
  showHumitureScreen = ~showHumitureScreen;
}
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32 Device");
  Serial.println("Initializing...");
  Serial.println(" > WiFi");
  Serial.println("Starting connecting WiFi.");
  Wire.begin(16, 17);

  ads.setGain(GAIN_TWO);
  ads.begin();
  pinMode(RelayOutputPin, OUTPUT);
  pinMode(Power12V, OUTPUT);

  
  digitalWrite(RelayOutputPin, LOW);
  digitalWrite(Power12V, LOW);
  //digitalWrite(pulsePin, LOW);
  pinMode(button1.PIN, INPUT_PULLUP);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);

    attachInterruptArg(button1.PIN, isr, &button1, FALLING);
  // WiFi.mode(WIFI_AP);
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }
  WiFi.begin(ssid, password);
  int i = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
    i++;
    
    hasWifi = false;
  if(i==10)
    esp_deep_sleep_start();

  }
  hasWifi = true;


  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println(" > IoT Hub");
  if (!initIotHubClient())
  {
    hasIoTHub = false;
    Serial.println("Initializing IoT hub failed.");
    return;
  }
  hasIoTHub = true;
  Serial.println("Start sending events.");
  send_interval_ms = millis();
  check_interval_ms = millis();
}

void loop()
{
  if (ManualOverride)
  {
    delay(100);
    digitalWrite(RelayOutputPin, HIGH);
    digitalWrite(Power12V, HIGH);
  //  telemetrySendInterval = 1000;
  }
  else
  {
     delay(100);
    digitalWrite(RelayOutputPin, LOW);
    digitalWrite(Power12V, LOW);

    //telemetrySendInterval = TELEMETRY_SEND_INTERVAL;
  }
  
  if (millis() - lastSensorReadMillis > SENSOR_READ_INTERVAL) {
    // read the sensor values
    checkStatusGarden();
    lastSensorReadMillis = millis();
  }
  if (WiFi.status() == WL_CONNECTED && hasIoTHub)
  {
    if ((int)(millis() - send_interval_ms) >= TELEMETRY_SEND_INTERVAL)
    {
      //get sensor data
      String SSAStr = String(VarVWC);
      String VWCMaxStr = String(SolenoidValve_valSensorMax);
      String VWCMinStr = String(SolenoidValve_valSensorMin);
      String ELTStr = String(elapsedTime/1000);
      String FMLStr = String(button1.numberKeyPresses);
      String VAStr = String(ManualOverride);
      sprintf_s(msgText, sizeof(msgText), "{\"SSA\":%s,\"TVWCMax\":%s,\"TVWCMin\":%s,\"VA\":%s,\"FML\":%s}", SSAStr, VWCMaxStr, VWCMinStr, VAStr, FMLStr);
      sendTelemetry(msgText);
  Serial.println(digitalRead(RelayOutputPin));

      send_interval_ms = millis();
    }
  }
  delay(10);
}

void checkStatusGarden()
{
  VarVWC = readVH400();
  Serial.println(digitalRead(RelayOutputPin));
  if (VarVWC > 5.0 && VarVWC < SolenoidValve_valSensorMin)
  {
    unsigned long timenow = millis();         // now: timestamp
    elapsedTime = timenow - startTimer;  // elapsed: duration

    //Open Valve and close others

    Serial.println(F("Main Valve On"));      

    if (sendMessage == true) {
      ManualOverride = true;
      Serial.println(digitalRead(RelayOutputPin));


      pulseCount = 0;
      totalMilliLitres = 0;
      button1.numberKeyPresses = 0;
      startTimer = millis();
      sendMessage = false;
      if (WiFi.status() == WL_CONNECTED)
      {
        String SSAStr = String(VarVWC);
        String VWCMaxStr = String(SolenoidValve_valSensorMax);
        String VWCMinStr = String(SolenoidValve_valSensorMin);
        String VAStr = String(ManualOverride);

        sprintf_s(msgText, sizeof(msgText), "{\"SSA\":%s,\"TVWCMax\":%s,\"TVWCMin\":%s,\"VA\":%s}", SSAStr, VWCMaxStr, VWCMinStr, VAStr);
        sendTelemetry(msgText);

      }
    }
  }
  if (VarVWC > 5.0 &&  VarVWC >= SolenoidValve_valSensorMax)
  {
    //Close Valve
    

    startValve = false;
    Serial.println(F("Valve Off"));

    if (sendMessage == false) {
      
      elapsedTime = millis() - startTimer;
      sendMessage = true;
      startValve = false;
      totalMilliLitres = 0;
      flowRate = 0;
      startTimer = 0 ;
      ManualOverride = false;
      if (WiFi.status() == WL_CONNECTED)
      {
        String SSAStr = String(VarVWC);
        String VWCMaxStr = String(SolenoidValve_valSensorMax);
        String VWCMinStr = String(SolenoidValve_valSensorMin);
        String ELTStr = String(elapsedTime/1000);
        String FMLStr = String(button1.numberKeyPresses);
        String VAStr = "1";
        sprintf_s(msgText, sizeof(msgText), "{\"SSA\":%s,\"TVWCMax\":%s,\"TVWCMin\":%s,\"VA\":%s,\"ELT\":%s,\"FML\":%s}",  SSAStr, VWCMaxStr, VWCMinStr, VAStr, ELTStr, FMLStr);
        sendTelemetry(msgText);
      }
      Counter++;

      if (Counter >= 99)
      {
        Counter = 1;
      }
    }

    // Reset the pulse counter so we can start incrementing again
    pulseCount = 0;
    totalMilliLitres = 0;
  }
}



float readVH400()
{
  float sensor1DN = 0.0;
float sensorVoltage = 0.0;
  Serial.println(sensor1DN);
  sensor1DN = ads.readADC_SingleEnded(0);
  sensorVoltage = (sensor1DN * 0.1875) / 1000;
   sensorVoltage = sensorVoltage *2.5;
    Serial.println(sensorVoltage);

  float VWC;
  // Calculate VWC
  if (sensorVoltage <= 1.1) {
    VWC = 10 * sensorVoltage - 1;
  } else if (sensorVoltage > 1.1 && sensorVoltage <= 1.3) {
    VWC = 25 * sensorVoltage - 17.5;
  } else if (sensorVoltage > 1.3 && sensorVoltage <= 1.82) {
    VWC = 48.08 * sensorVoltage - 47.5;
  } else if (sensorVoltage > 1.82) {
    VWC = 26.32 * sensorVoltage - 7.89;
  }
  Serial.println(VWC );
  return (VWC);
}
