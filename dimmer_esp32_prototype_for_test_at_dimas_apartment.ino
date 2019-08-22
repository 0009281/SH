#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <esp_wifi.h>
#include <soc/rtc.h>
#include "EEPROM.h"
#include <string>
#include <HTTPUpdate.h>


#define SKETCH_VERSION "1.0.22"


#define EEPROM_SIZE 5
#define RED_LED 2
#define GREEN_LED 18
#define ONBOARD_BUTTON 0
#define BUTTON0 15
#define BUTTON1 21
#define BUTTON2 23
#define BUTTON3 22
#define mqttPort 1883
#define DIMMER_DAC_CHANNEL 25
const int FL5150_ON_OFF = 12; // Could be different depending on the dev board. I used the DOIT ESP32 dev board.
const int LEADING_TRAILING = 32; // Could be different depending on the dev board. I used the DOIT ESP32 dev board.


#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)

/* Useful Macros for getting elapsed time */
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)  
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN) 
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)  


#define SHORT_PRESS 0x1
#define LONG_PRESS 0x2
#define RELEASE_AFTER_LONG_PRESS 0x3
#define debounceDelay 5


//https://raw.githubusercontent.com/0009281/SH/master/dimmer_esp32_prototype_for_test_at_dimas_apartment.ino.nodemcu-32s.bin
const char* rootCACertificate = \
"-----BEGIN CERTIFICATE-----\n" \
"MIIDxTCCAq2gAwIBAgIQAqxcJmoLQJuPC3nyrkYldzANBgkqhkiG9w0BAQUFADBsMQswCQYDVQQG\n" \
"EwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3d3cuZGlnaWNlcnQuY29tMSsw\n" \
"KQYDVQQDEyJEaWdpQ2VydCBIaWdoIEFzc3VyYW5jZSBFViBSb290IENBMB4XDTA2MTExMDAwMDAw\n" \
"MFoXDTMxMTExMDAwMDAwMFowbDELMAkGA1UEBhMCVVMxFTATBgNVBAoTDERpZ2lDZXJ0IEluYzEZ\n" \
"MBcGA1UECxMQd3d3LmRpZ2ljZXJ0LmNvbTErMCkGA1UEAxMiRGlnaUNlcnQgSGlnaCBBc3N1cmFu\n" \
"Y2UgRVYgUm9vdCBDQTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAMbM5XPm+9S75S0t\n" \
"Mqbf5YE/yc0lSbZxKsPVlDRnogocsF9ppkCxxLeyj9CYpKlBWTrT3JTWPNt0OKRKzE0lgvdKpVMS\n" \
"OO7zSW1xkX5jtqumX8OkhPhPYlG++MXs2ziS4wblCJEMxChBVfvLWokVfnHoNb9Ncgk9vjo4UFt3\n" \
"MRuNs8ckRZqnrG0AFFoEt7oT61EKmEFBIk5lYYeBQVCmeVyJ3hlKV9Uu5l0cUyx+mM0aBhakaHPQ\n" \
"NAQTXKFx01p8VdteZOE3hzBWBOURtCmAEvF5OYiiAhF8J2a3iLd48soKqDirCmTCv2ZdlYTBoSUe\n" \
"h10aUAsgEsxBu24LUTi4S8sCAwEAAaNjMGEwDgYDVR0PAQH/BAQDAgGGMA8GA1UdEwEB/wQFMAMB\n" \
"Af8wHQYDVR0OBBYEFLE+w2kD+L9HAdSYJhoIAu9jZCvDMB8GA1UdIwQYMBaAFLE+w2kD+L9HAdSY\n" \
"JhoIAu9jZCvDMA0GCSqGSIb3DQEBBQUAA4IBAQAcGgaX3NecnzyIZgYIVyHbIUf4KmeqvxgydkAQ\n" \
"V8GK83rZEWWONfqe/EW1ntlMMUu4kehDLI6zeM7b41N5cdblIZQB2lWHmiRk9opmzN6cN82oNLFp\n" \
"myPInngiK3BD41VHMWEZ71jFhS9OMPagMRYjyOfiZRYzy78aG6A9+MpeizGLYAiJLQwGXFK3xPkK\n" \
"mNEVX58Svnw2Yzi9RKR/5CYrCsSXaQ3pjOLAEFe4yHYSkVXySGnYvCoCWw9E1CAx2/S6cCZdkGCe\n" \
"vEsXCS+0yx5DaMkHJ8HSXPfqIbloEpw8nL+e/IBcm2PN7EeqJSdnoDfzAIJ9VNep+OkuE6N36B9K\n" \
"-----END CERTIFICATE-----\n";


void led_status_red() {
 digitalWrite(RED_LED, HIGH);
 digitalWrite(GREEN_LED, LOW);
}

void led_status_green() {
 digitalWrite(RED_LED, LOW);
 digitalWrite(GREEN_LED, HIGH);
}

void led_status_off() {
 digitalWrite(RED_LED, LOW);
 digitalWrite(GREEN_LED, LOW);
}


void led_status_red_blink() {
 digitalWrite(RED_LED, HIGH);
 digitalWrite(GREEN_LED, LOW);
 delay(10);
 digitalWrite(RED_LED, LOW);
}

void led_status_green_blink() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  delay(10);
  digitalWrite(GREEN_LED, HIGH);
}


struct DimmerSetupData {
  unsigned char last_brightness;
  unsigned char last_onoff_state;
  unsigned char max_brightness;
  unsigned char min_brightness;
  unsigned char dimming_type;
};
unsigned long IdleTime, IdleTimeMillis;

byte mac[6];

DimmerSetupData eeprom_settings;
unsigned char current_brightness, max_brightness, min_brightness, dimming_type, last_onoff_state;
unsigned long brightness_step_delay_millis=0, brightness_step_delay_speed=20;

unsigned char detector_state = 0, last_detector_state = 0, i, button2_last_state, button3_state, button3_last_state, reading_detector_state=0, dim_value=197, max_dim_value=0,button_state, button_last_state, current_brighness, max_brighness, min_brighness, fw_update_counter=0;

//const char *str_firmware_version="1.0";


unsigned char last_button1_state=1, button1_state, button1_click_event;
unsigned char last_button2_state=1, button2_state, button2_click_event;
bool button1_changed_detected, button1_low_press_detected;
bool button2_changed_detected, button2_low_press_detected;

unsigned long high_low_edge_detected, detector_start, detector_end, detector_middle, dim_delay = 5000, previousMillis = 0, detector_debounce, wifi_check_connection_Millis=0, temperature_sensor_Millis=0, last_debounce_time_button1, \
               last_debounce_time_button2, last_debounce_time_button1_shift, last_debounce_time_button2_shift;
bool high_low_edge, cycle_mode=0, dim_forward, button1_bingo=false , button2_bingo=false, loadDefaultEeprom = false;
int inc_value;
uint16_t button1_value=0xffff, button2_value=0xffff;

const char* ssid = "slowbad";
const char* password = "slowbad1234567!@";

const char* mqttServer = "broker.mqttdashboard.com";
const char* mqttUser = "dkarev";
const char* mqttPassword = "123";
IPAddress ip;
float lnk_voltage_value_prev=3.3, lnk_voltage_value;

//Analog Input
#define LNK_VOLTAGE_PIN 34
int lnk_adc_value = 0, max_adc_value;
char str_sensor[6];


hw_timer_t *timer = NULL;


WiFiClient espClient;

// Callback function header
void callback(char* topic, byte* payload, unsigned int length);

void eeprom_settings_check_and_save(void) {
  if ((current_brightness != eeprom_settings.last_brightness) || dimming_type!=eeprom_settings.dimming_type || last_onoff_state!=eeprom_settings.last_onoff_state) {
    eeprom_settings.last_brightness = current_brightness;
    eeprom_settings.dimming_type = dimming_type;
    eeprom_settings.last_onoff_state = last_onoff_state;
    EEPROM.put(0, eeprom_settings);
    EEPROM.commit();
   }
  
}

void IRAM_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}


PubSubClient client(mqttServer, mqttPort, callback, espClient);

// Callback function
void callback(char* topic, byte* payload, unsigned int length) {
  // In order to republish this payload, a copy must be made
  // as the orignal payload buffer will be overwritten whilst
  // constructing the PUBLISH packet.

  // Allocate the correct amount of memory for the payload copy
  //byte* p = (byte*)malloc(length);
  char* p = (char*)malloc(length);
  // Copy the payload to the new buffer
  memcpy(p,payload,length);
  //client.publish("outTopic", p, length);
      Serial.print("Topic: ");
      Serial.print(String(topic));
      Serial.print(" Payload length: ");
      //Serial.println((char)payload[0]);
//      Serial.println(atoi(p));
      Serial.println(length);
      
    cycle_mode = 0;
    
    //////////////////////////////////////////////////Dimming type/////////////////////
    if (!strcmp(topic,MacAddress2Str("dimmers/",mac,"/dimming_type"))) {
      led_status_green_blink();
      //GPIO is inverted at the FL5150 DIM_MODE pin
      dimming_type = atoi(p);
      if (dimming_type==1) {
        digitalWrite(FL5150_ON_OFF, LOW); //disable
        delay(1000);
        digitalWrite(LEADING_TRAILING, LOW); // leading, cuz it is inverted in the schematics
        //delay(100);
        //digitalWrite(FL5150_ON_OFF, HIGH); //enable
        client.publish(MacAddress2Str("dimmers/",mac,"/dimming_type_fb"), "1");
      }
      else
      {
        digitalWrite(FL5150_ON_OFF, LOW); //disable
        delay(1000);
        digitalWrite(LEADING_TRAILING, HIGH); //trailing, cuz it is inverted in the schematics
        //delay(100);
        //digitalWrite(FL5150_ON_OFF, HIGH); //enable
        client.publish(MacAddress2Str("dimmers/",mac,"/dimming_type_fb"), "0");
      }

      if (current_brightness) {
        delay(100);
        digitalWrite(FL5150_ON_OFF, HIGH); //enable
      }
      eeprom_settings_check_and_save();  
      IdleTime = millis();
    }

     //////////////////////////////////////////////////Dimmer brightness/////////////////////
    if ((!strcmp(topic,MacAddress2Str("dimmers/",mac,"/brightness"))) && (current_brightness != atoi(p))) {
      led_status_green_blink();
      if ((atoi(p)>0) && (atoi(p)<=200)) {
        last_onoff_state = 1;
        current_brightness = atoi(p);
        digitalWrite(FL5150_ON_OFF, HIGH); //enable
        dacWrite(25, current_brightness);
        itoa(current_brightness, str_sensor, 10);
        client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
      }
      eeprom_settings_check_and_save();  
      IdleTime = millis();
    }

   
    if (!strcmp(topic,MacAddress2Str("dimmers/",mac,"/toggle"))) {
      led_status_green_blink();
      digitalWrite(FL5150_ON_OFF, !(digitalRead(FL5150_ON_OFF))); //disable
      if (digitalRead(FL5150_ON_OFF)) {
        last_onoff_state = 1;
        current_brightness = eeprom_settings.last_brightness;
        itoa(current_brightness, str_sensor, 10);
        client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
      }
      else {
        last_onoff_state = 0;
        itoa(0, str_sensor, 10);
        client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
      }
      eeprom_settings_check_and_save();  
      IdleTime = millis();
    }


    if (!strcmp(topic,MacAddress2Str("dimmers/",mac,"/on"))) {
      led_status_green_blink();
      if (!last_onoff_state) {
        digitalWrite(FL5150_ON_OFF, HIGH); //enable
        current_brightness = eeprom_settings.last_brightness;
        last_onoff_state = 1;
        dacWrite(DIMMER_DAC_CHANNEL, current_brightness);
        itoa(current_brightness, str_sensor, 10);
        client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
        eeprom_settings_check_and_save();  
      }
      IdleTime = millis();
    }

    if (!strcmp(topic,MacAddress2Str("dimmers/",mac,"/off"))) {
      led_status_green_blink();
      if (last_onoff_state) {
        digitalWrite(FL5150_ON_OFF, LOW); //disable
        last_onoff_state = 0;
        eeprom_settings_check_and_save();
        //dacWrite(DIMMER_DAC_CHANNEL, current_brightness);
        itoa(0, str_sensor, 10);
        client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
      }
      IdleTime = millis();
    }

    if (!strcmp(topic,MacAddress2Str("dimmers/",mac,"/cycle"))) {
      digitalWrite(GREEN_LED, HIGH); 
      delay(40);
      digitalWrite(GREEN_LED, LOW); 
      cycle_mode = 1;
      dim_value = 0;
      //GPIO is inverted at the FL5150 DIM_MODE pin
      IdleTime = millis();
    }

    if (!strcmp(topic,MacAddress2Str("dimmers/",mac,"/reboot"))) {
      led_status_off();
      delay(500);
      esp_restart();
    }


    if (!strcmp(topic,MacAddress2Str("dimmers/",mac,"/get_current_state"))) {
      led_status_green_blink();
      client.publish(MacAddress2Str("dimmers/",mac,"/firmware_version"), SKETCH_VERSION);
      if (dimming_type) client.publish(MacAddress2Str("dimmers/",mac,"/dimming_type_fb"), "1"); else client.publish(MacAddress2Str("dimmers/",mac,"/dimming_type_fb"), "0");

      char strIdleTime[20];
      IdleTimeMillis = millis();
      sprintf(strIdleTime, "%02ud:%02uh:%02um:%02us", elapsedDays((IdleTimeMillis - IdleTime)/1000), numberOfHours((IdleTimeMillis - IdleTime)/1000), numberOfMinutes((IdleTimeMillis - IdleTime)/1000), numberOfSeconds((IdleTimeMillis - IdleTime)/1000));    
      client.publish(MacAddress2Str("dimmers/",mac,"/idle_time"), strIdleTime);    
      sprintf(strIdleTime, "%03d", WiFi.RSSI());    
      client.publish(MacAddress2Str("dimmers/",mac,"/rssi_level"), strIdleTime);
      if (last_onoff_state) itoa(current_brightness, str_sensor, 10); else itoa(0, str_sensor, 10);
      client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
      sprintf(strIdleTime, "%02ud:%02uh:%02um:%02us", elapsedDays(IdleTimeMillis/1000), numberOfHours(IdleTimeMillis/1000), numberOfMinutes(IdleTimeMillis/1000), numberOfSeconds(IdleTimeMillis/1000));    
      client.publish(MacAddress2Str("dimmers/",mac,"/uptime"), strIdleTime);    

      float sensor = temperatureRead();//(temprature_sens_read() - 32) / 1.8;
      dtostrf(sensor, 4, 2, str_sensor);
      client.publish(MacAddress2Str("dimmers/",mac,"/temperature"), str_sensor);     

    }


  if (!strcmp(topic,MacAddress2Str("dimmers/",mac,"/fw_update"))) {
    char* p1 = (char*)malloc(length+1);
    memcpy(p1,payload,length);
    p1[length]=0;
    Serial.println(String(p1));
    IdleTime = millis();
    if (fw_update_counter>1) {
    //if (strcmp(SKETCH_VERSION,p1)) {

    
    
    //led_status_red();
      //led_status_green_blink();
      //led_status_red_blink();
      Serial.println("Update...");
       led_status_red();
       //client.disconnect();

           WiFiClientSecure Secure_client;
    Secure_client.setCACert(rootCACertificate);

    // Reading data over SSL may be slow, use an adequate timeout
    Secure_client.setTimeout(12000/1000);

    // The line below is optional. It can be used to blink the LED on the board during flashing
    // The LED will be on during download of one buffer of data from the network. The LED will
    // be off during writing that buffer to flash
    // On a good connection the LED should flash regularly. On a bad connection the LED will be
    // on much longer than it will be off. Other pins than LED_BUILTIN may be used. The second
    // value is used to put the LED on. If the LED is on with HIGH, that value should be passed
    // httpUpdate.setLedPin(LED_BUILTIN, HIGH);

    t_httpUpdate_return ret = httpUpdate.update(Secure_client, p1);
      
    free(p1);   
      // t_httpUpdate_return ret = httpUpdate.update(espClient, p1 , SKETCH_VERSION);
       Serial.println("Update is completed");
    // Or:
    //t_httpUpdate_return ret = httpUpdate.update(client, "server", 80, "file.bin");

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        led_status_green();
        delay(2000);
        ESP.restart();

        break;
    }

    
      led_status_off();

    //}
    }
    else fw_update_counter++;
    
  }



  // Free the memory
  free(p);
}

char _strMacAddress[50];
char *MacAddress2Str(char* s1, byte *_mac, char* s2)
{
  sprintf(_strMacAddress, "%s%02x:%02x:%02x:%02x:%02x:%02x%s", s1, _mac[5], _mac[4], _mac[3], _mac[2], _mac[1], _mac[0],s2);
  return _strMacAddress;
}

void reconnect() {
 
// char tmp_str[30+18];
 //String dimmers="dimmers/";
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    
    if (client.connect( MacAddress2Str("",mac,""), mqttUser, mqttPassword )) {
    
        
        Serial.println("connected");
        client.subscribe(MacAddress2Str("dimmers/",mac,"/brightness"));
        client.subscribe(MacAddress2Str("dimmers/",mac,"/dimming_type"));
        client.subscribe(MacAddress2Str("dimmers/",mac,"/on"));
        client.subscribe(MacAddress2Str("dimmers/",mac,"/off"));
        client.subscribe(MacAddress2Str("dimmers/",mac,"/toggle"));
        client.subscribe(MacAddress2Str("dimmers/",mac,"/up"));
        client.subscribe(MacAddress2Str("dimmers/",mac,"/down"));
        client.subscribe(MacAddress2Str("dimmers/",mac,"/stop"));
        client.subscribe(MacAddress2Str("dimmers/",mac,"/fw_update"));
        client.subscribe(MacAddress2Str("dimmers/",mac,"/reboot"));
        client.subscribe(MacAddress2Str("dimmers/",mac,"/get_current_state"));
        client.publish(MacAddress2Str("dimmers/",mac,"/firmware_version"), SKETCH_VERSION);
        if (last_onoff_state) itoa(current_brightness, str_sensor, 10); else itoa(0, str_sensor, 10);
        
        client.publish(MacAddress2Str("dimmers/",mac,"/brightness") , str_sensor);
        client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
        Serial.print("current_brightness reconnect(): ");
        Serial.println(current_brightness);

        
        if (dimming_type) client.publish(MacAddress2Str("dimmers/",mac,"/dimming_type_fb"), "1"); else client.publish(MacAddress2Str("dimmers/",mac,"/dimming_type_fb"), "0");

        char strIdleTime[20];
        IdleTimeMillis = millis();
        sprintf(strIdleTime, "%02ud:%02uh:%02um:%02us", elapsedDays((IdleTimeMillis - IdleTime)/1000), numberOfHours((IdleTimeMillis - IdleTime)/1000), numberOfMinutes((IdleTimeMillis - IdleTime)/1000), numberOfSeconds((IdleTimeMillis - IdleTime)/1000));    
        client.publish(MacAddress2Str("dimmers/",mac,"/idle_time"), strIdleTime); 
        sprintf(strIdleTime, "%03d", WiFi.RSSI());    
        client.publish(MacAddress2Str("dimmers/",mac,"/rssi_level"), strIdleTime);

        sprintf(strIdleTime, "%02ud:%02uh:%02um:%02us", elapsedDays(IdleTimeMillis/1000), numberOfHours(IdleTimeMillis/1000), numberOfMinutes(IdleTimeMillis/1000), numberOfSeconds(IdleTimeMillis/1000));    
        client.publish(MacAddress2Str("dimmers/",mac,"/uptime"), strIdleTime);    

        float sensor = temperatureRead();//(temprature_sens_read() - 32) / 1.8;
        dtostrf(sensor, 4, 2, str_sensor);
        client.publish(MacAddress2Str("dimmers/",mac,"/temperature"), str_sensor);     

        
        led_status_green();
        
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event) {
        case SYSTEM_EVENT_WIFI_READY: 
            led_status_red_blink();
            Serial.println("WiFi interface ready");
            break;
        case SYSTEM_EVENT_SCAN_DONE:
            Serial.println("Completed scan for access points");
            break;
        case SYSTEM_EVENT_STA_START:
            led_status_red_blink();
            Serial.println("WiFi client started");
            break;
        case SYSTEM_EVENT_STA_STOP:
            Serial.println("WiFi clients stopped");
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            Serial.println("Connected to access point");
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            Serial.println("Disconnected from WiFi access point");
            WiFi.reconnect();
            break;
        case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:
            Serial.println("Authentication mode of access point has changed");
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            led_status_red();
            Serial.print("Obtained IP address: ");
            Serial.println(WiFi.localIP());
            break;
        case SYSTEM_EVENT_STA_LOST_IP:
            Serial.println("Lost IP address and IP address is reset to 0");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
            Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_FAILED:
            Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
            Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
            break;
        case SYSTEM_EVENT_STA_WPS_ER_PIN:
            Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
            break;
        case SYSTEM_EVENT_AP_START:
            Serial.println("WiFi access point started");
            break;
        case SYSTEM_EVENT_AP_STOP:
            Serial.println("WiFi access point  stopped");
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            Serial.println("Client connected");
            break;
        case SYSTEM_EVENT_AP_STADISCONNECTED:
            Serial.println("Client disconnected");
            break;
        case SYSTEM_EVENT_AP_STAIPASSIGNED:
            Serial.println("Assigned IP address to client");
            break;
        case SYSTEM_EVENT_AP_PROBEREQRECVED:
            Serial.println("Received probe request");
            break;
        case SYSTEM_EVENT_GOT_IP6:
            Serial.println("IPv6 is preferred");
            break;
        case SYSTEM_EVENT_ETH_START:
            Serial.println("Ethernet started");
            break;
        case SYSTEM_EVENT_ETH_STOP:
            Serial.println("Ethernet stopped");
            break;
        case SYSTEM_EVENT_ETH_CONNECTED:
            Serial.println("Ethernet connected");
            break;
        case SYSTEM_EVENT_ETH_DISCONNECTED:
            Serial.println("Ethernet disconnected");
            break;
        case SYSTEM_EVENT_ETH_GOT_IP:
            Serial.println("Obtained IP address");
            break;
    }}

//***************************************************************************************************** setup **************************************************************

void setup() {
  unsigned char k = 0, j = 0, l=0;
  uint16_t loadDefaultEepromFlag = 0xFFFF, button1Millis, adc_array[10], reading_button1;
  
  
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  led_status_off();

  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  Serial.begin(115200);
  Serial.println("Booting");

  
  //if the BUTTON1 is pressed check 10 times to be sure that is not a noise, then the user MUST release the button within 1 sec to resume the setting to the default state
  pinMode(BUTTON1, INPUT);
  if (!digitalRead(BUTTON1)) {
    for (int i=0; i<10; i++) {
      reading_button1 = digitalRead(BUTTON1);
      loadDefaultEepromFlag=(loadDefaultEepromFlag<<1) | reading_button1;
      delay(100);
    }
    if (!digitalRead(BUTTON1)) {
      unsigned long currentMillis = millis();  
      do {
        led_status_red_blink();
        delay(10);
        reading_button1 = digitalRead(BUTTON1);
        button1Millis = millis();
      } while (!reading_button1 && (button1Millis - currentMillis < 1000));
      
      if (reading_button1 &&  (button1Millis - currentMillis < 1000) && loadDefaultEepromFlag==0xFC00) {
        loadDefaultEeprom = true;        
      }
    }
 }  
  
  

  pinMode(ONBOARD_BUTTON, INPUT);
  pinMode(BUTTON0, INPUT);
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(LEADING_TRAILING, OUTPUT);
  pinMode(FL5150_ON_OFF, OUTPUT);
  digitalWrite(FL5150_ON_OFF, LOW);
 
  // delete old config
  WiFi.disconnect(true);

 

  // Examples of different ways to register wifi events
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
  WiFi.setAutoReconnect(true);
  Serial.println("Wait for WiFi... ");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      //Serial.printf("Progress: %u%%\r", (progress / (total / 100))); led_status_green_blink();
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();



  analogSetWidth(12);
  adcAttachPin(34);
  analogSetPinAttenuation(34, ADC_0db);
  

  last_detector_state = 1;

  //corrupt EEPROM or reset to default is required
  if ((!EEPROM.begin(EEPROM_SIZE)) || loadDefaultEeprom) {
    Serial.println("Failed to initialise EEPROM"); 
    loadDefaultEeprom = false;
    eeprom_settings.last_brightness = 30;
    eeprom_settings.max_brightness = 200;
    eeprom_settings.min_brightness = 0;
    eeprom_settings.dimming_type = 1  ;
    eeprom_settings.last_onoff_state = 1;
    current_brightness = eeprom_settings.last_brightness;
    max_brightness = eeprom_settings.max_brightness;
    min_brightness = eeprom_settings.min_brightness;
    dimming_type = eeprom_settings.dimming_type;
    last_onoff_state = eeprom_settings.last_onoff_state;
    EEPROM.put(0, eeprom_settings);
    EEPROM.commit();
  }
  else {
    Serial.println("OK to initialise EEPROM"); 
    EEPROM.get(0, eeprom_settings);
    current_brightness = eeprom_settings.last_brightness;
    max_brightness = eeprom_settings.max_brightness;
    min_brightness = eeprom_settings.min_brightness;
    dimming_type = eeprom_settings.dimming_type;
    last_onoff_state = eeprom_settings.last_onoff_state;
    if (isnan(current_brightness)) current_brightness = 30;
    if (isnan(max_brightness)) max_brightness = 200;
    if (isnan(min_brightness)) min_brightness = 0;
    if (isnan(dimming_type)) dimming_type = 1;
    if (isnan(last_onoff_state)) last_onoff_state = 1;
 }


  if (dimming_type==1) {
    digitalWrite(FL5150_ON_OFF, LOW); //disable
    delay(1000);
    digitalWrite(LEADING_TRAILING, LOW); //leading, cuz it is inverted in the schematics
    delay(100);
    client.publish(MacAddress2Str("dimmers/",mac,"/dimming_type_fb"), "1");
  }
  else {
    digitalWrite(FL5150_ON_OFF, LOW); //disable
    delay(1000);
    digitalWrite(LEADING_TRAILING, HIGH); //trailing, cuz it is inverted in the schematics
    delay(100);
    client.publish(MacAddress2Str("dimmers/",mac,"/dimming_type_fb"), "0");
  }

  if (last_onoff_state) {
    dacWrite(DIMMER_DAC_CHANNEL, current_brightness);
    digitalWrite(FL5150_ON_OFF, HIGH); //enable
  }

  WiFi.macAddress(mac);

  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, 600 * 1000000, false); //600 * sec
  timerAlarmEnable(timer);                          //enable interrupt

} //setup

//***************************************************************************************************** loop **************************************************************

void loop() {
  unsigned long currentMillis = millis();  
  unsigned char ii=0;
  unsigned int adc_array[10];
  

  timerWrite(timer, 0); //reset timer (feed watchdog) 
  ArduinoOTA.handle();
  client.loop();  


  lnk_adc_value = analogRead(LNK_VOLTAGE_PIN);
  //lnk_voltage_value = (lnk_adc_value * 1.1) / 4096.0;
  //if (lnk_adc_value == 0) delayMicroseconds(500);

/*  
//if (currentMillis - previousMillis >= 1000) {
//  previousMillis = currentMillis;
  do {
  //lnk_adc_value = analogRead(LNK_VOLTAGE_PIN);
  } while (analogRead(LNK_VOLTAGE_PIN));
  do {
  //lnk_adc_value = analogRead(LNK_VOLTAGE_PIN);
  } while (analogRead(LNK_VOLTAGE_PIN)<1000);
  
  //lnk_voltage_value = (lnk_adc_value * 1.1) / 4096.0;
  //if (lnk_adc_value == 0) {
    
    noInterrupts(); 
    //delay(2);
    for (ii=0; ii<6;ii++) {
     adc_array[ii] = analogRead(LNK_VOLTAGE_PIN);
    }
    interrupts();
    max_adc_value = 0;
    for (ii=0; ii<6;ii++) {
     if (adc_array[ii] > max_adc_value) max_adc_value =  adc_array[ii];
    }
*/


    
    //max_adc_value = analogRead(LNK_VOLTAGE_PIN);
    //if (lnk_adc_value > max_adc_value) max_adc_value = lnk_adc_value;
    
    //delay(1);
    //lnk_adc_value = analogRead(LNK_VOLTAGE_PIN);
    //if (max_adc_value < lnk_adc_value) max_adc_value = lnk_adc_value;

    /*delay(1);
    lnk_adc_value = analogRead(LNK_VOLTAGE_PIN);
    if (max_adc_value < lnk_adc_value) max_adc_value = lnk_adc_value;
    delay(1);
    lnk_adc_value = analogRead(LNK_VOLTAGE_PIN);
    if (max_adc_value < lnk_adc_value) max_adc_value = lnk_adc_value;
    delay(1);
    lnk_adc_value = analogRead(LNK_VOLTAGE_PIN);
    if (max_adc_value < lnk_adc_value) max_adc_value = lnk_adc_value;
    */

    
    //lnk_voltage_value = (max_adc_value * 1.1) / 4096.0;
    //if ((lnk_voltage_value < 0.58) /*&& (dim_value>max_dim_value)*/) {
      //dim_value = max_dim_value;
      //dacWrite(25, dim_value); 
      //digitalWrite(FL5150_ON_OFF, LOW); //disable
    //}
  if ((max_adc_value < 1800) && (dim_value>80)) {
      //dim_value = 97;
      //dacWrite(25, dim_value); 
      //digitalWrite(FL5150_ON_OFF, LOW); //disable
  }
  
  
  
  //} //lnk_adc_value == 0

//}
 
   if (currentMillis - wifi_check_connection_Millis >= 3000) {
 
//     Serial.printf("button1_value: ");
//     Serial.printf("0x%x\n",button1_value);
     wifi_check_connection_Millis = currentMillis;
     //if (WiFi.waitForConnectResult() != WL_CONNECTED) {
//       WiFi.begin(ssid, password);      
       if (!client.connected()) reconnect();
       
  //   }
   }
 
/*    
   if (currentMillis - temperature_sensor_Millis >= 3000) {
     temperature_sensor_Millis = currentMillis;
     float sensor = temperatureRead();//(temprature_sens_read() - 32) / 1.8;
     dtostrf(sensor, 4, 2, str_sensor);
     client.publish("dimmer1/temperature", str_sensor, true);     
   }
  
*/
  
  if (cycle_mode == 1) {
    if (currentMillis - previousMillis >= 30) {
      previousMillis = currentMillis;
      if (dim_value==90) inc_value = -1;
      if (dim_value==0) inc_value = 1;
      dim_value = dim_value + inc_value;
      dacWrite(25, dim_value); 
     }
    
  }

   
  //}
 


  // read the state of the switch into a local variable:
  int reading_button1 = digitalRead(BUTTON1);
  
  if (reading_button1 != last_button1_state) {
    // reset the debouncing timer
    last_debounce_time_button1 = millis();
    button1_changed_detected = 1;
    button1_state = reading_button1;
    last_debounce_time_button1_shift = millis();
  }

  if (((millis() - last_debounce_time_button1_shift) > 2) && (button1_changed_detected == 1)) { 
    last_debounce_time_button1_shift = millis();
    button1_value=(button1_value<<1) | reading_button1 | 0xe000;
    if (button1_value==0xf000) button1_bingo=1;
  }

  
  if (((millis() - last_debounce_time_button1) > debounceDelay) && (button1_changed_detected == 1) && (button1_state == reading_button1) && (button1_bingo) ) {
    if ((reading_button1 == HIGH) && (!button1_low_press_detected)) {
      button1_click_event = SHORT_PRESS;
      Serial.println("Button1 High transition has been detected after short press");
      button1_changed_detected = 0;
      button1_bingo = 0;
    }
    if ((reading_button1 == HIGH) && (button1_low_press_detected)) {
      button1_click_event = RELEASE_AFTER_LONG_PRESS;
      Serial.println("Button1 High transition after long press has been detected");
      button1_low_press_detected = false;
      button1_changed_detected = 0;
      button1_bingo = 0;
    }
    if (((millis() - last_debounce_time_button1) > 800) && ((reading_button1 == LOW))) {
      button1_click_event = LONG_PRESS;  
      Serial.println("Button1 Long press has been detected");
      button1_low_press_detected = true;
      button1_changed_detected = 0;
      
    }
 
  
  }
  

  // read the state of the switch into a local variable:
  int reading_button2 = digitalRead(BUTTON2);
  if (reading_button2 != last_button2_state) {
    // reset the debouncing timer
    last_debounce_time_button2 = millis();
    button2_changed_detected = 1;
    button2_state = reading_button2;
    last_debounce_time_button2_shift = millis();
  }
  
  if (((millis() - last_debounce_time_button2_shift) > 2) && (button2_changed_detected == 1)) { 
    last_debounce_time_button2_shift = millis();
    button2_value=(button2_value<<1) | reading_button2 | 0xe000;
    if (button2_value==0xf000) button2_bingo=1;
  }
  
  if (((millis() - last_debounce_time_button2) > debounceDelay) && (button2_changed_detected == 1) && (button2_state == reading_button2) && (button2_bingo)) {
    if ((reading_button2 == HIGH) && (!button2_low_press_detected)) {
       button2_click_event = SHORT_PRESS;
       Serial.println("Button2 High transition has been detected after short press");
       button2_changed_detected = 0;
       button2_bingo = 0;
    }
    if ((reading_button2 == HIGH) && (button2_low_press_detected)) {
      button2_click_event = RELEASE_AFTER_LONG_PRESS;
      Serial.println("Button2 High transition after long press has been detected");
      button2_low_press_detected = false;
      button2_changed_detected = 0;
      button2_bingo = 0;
    }
    if (((millis() - last_debounce_time_button2) > 800) && ((reading_button2 == LOW))) {
      button2_click_event = LONG_PRESS;  
      Serial.println("Button2 Long press has been detected");
      button2_low_press_detected = true;
      button2_changed_detected = 0;
    }

  
  
  }



switch (button1_click_event) {
  case SHORT_PRESS: current_brightness = 0;
                    dacWrite(DIMMER_DAC_CHANNEL, current_brightness);
                    digitalWrite(FL5150_ON_OFF, LOW); //disable
                    //client.publish("dimmer1/off", "");
                    itoa(current_brightness, str_sensor, 10);
                    client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
                    button1_click_event=0;
                    IdleTime = millis();
                    break;   
  case LONG_PRESS: 
                   if (!current_brightness) {button1_click_event=0; break;}
                   //if (current_brightness<=30) brightness_step_delay_speed = 200; else brightness_step_delay_speed = 20;
            
                   if (currentMillis - brightness_step_delay_millis > brightness_step_delay_speed) { current_brightness--; dacWrite(DIMMER_DAC_CHANNEL, current_brightness);  brightness_step_delay_millis = currentMillis; }
                   break;      
  case RELEASE_AFTER_LONG_PRESS: 
                   if (!current_brightness) digitalWrite(FL5150_ON_OFF, LOW); //disable
                   eeprom_settings_check_and_save();
                   button1_click_event=0; 
                   itoa(current_brightness, str_sensor, 10);
                   client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
                   break;      

}




switch (button2_click_event) {
  case SHORT_PRESS: 
                    current_brightness = eeprom_settings.last_brightness;
                    dacWrite(DIMMER_DAC_CHANNEL, current_brightness);
                    digitalWrite(FL5150_ON_OFF, HIGH); //enable
                    //client.publish("dimmer1/on", ""); 
                    itoa(current_brightness, str_sensor, 10);
                    client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
                    button2_click_event=0; 
                    IdleTime = millis();
                    break;
  case LONG_PRESS: 
                    digitalWrite(FL5150_ON_OFF, HIGH); //enable
                    if (current_brightness>=200) {
                      dacWrite(DIMMER_DAC_CHANNEL, 0);
                      delay(200);
                      dacWrite(DIMMER_DAC_CHANNEL, current_brightness);  
                      delay(200);
                    }
                    if (current_brightness>=200) {/*button2_click_event=0;*/ break;}
                    //if (current_brightness<=30) brightness_step_delay_speed = 200; else brightness_step_delay_speed = 20;
                    if (currentMillis - brightness_step_delay_millis > brightness_step_delay_speed) { 
                      current_brightness++; 
                      dacWrite(DIMMER_DAC_CHANNEL, current_brightness);  
                      brightness_step_delay_millis = currentMillis; 
                    }
                    
                    break;      
  case RELEASE_AFTER_LONG_PRESS: 
                    eeprom_settings_check_and_save();
                    button2_click_event=0; 
                    itoa(current_brightness, str_sensor, 10);
                    client.publish(MacAddress2Str("dimmers/",mac,"/brightness_fb") , str_sensor);
                    break;      


}

  
  last_button1_state = reading_button1;
  last_button2_state = reading_button2;

}
