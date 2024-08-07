#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>
#include "arduino_secrets.h"

#if !(defined(USE_NOTECARD) || defined(BOARD_HAS_WIFI) || defined(BOARD_HAS_GSM) || defined(BOARD_HAS_LORA) || \
      defined(BOARD_HAS_NB) || defined(BOARD_HAS_ETHERNET) || defined(BOARD_HAS_CATM1_NBIOT))
  #error "Please check Arduino IoT Cloud supported boards list: https://github.com/arduino-libraries/ArduinoIoTCloud/#what"
#endif

#if defined(BOARD_HAS_SECRET_KEY)
  #define BOARD_ID "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx"
#endif

#if defined(USE_NOTECARD)
  /* The Notecard can provide connectivity to almost any board via ESLOV (I2C)
   * or UART. An empty string (or the default value provided below) will not
   * override the Notecard's existing configuration.
   * Learn more at: https://dev.blues.io */
  #define NOTECARD_PRODUCT_UID "com.domain.you:product"
#endif

void onSwitchButtonChange();

bool switchButton;
CloudSchedule oneShot;
CloudSchedule minute;
CloudSchedule hourly;
CloudSchedule daily;
CloudSchedule weekly;
CloudSchedule monthly;
CloudSchedule yearly;

void initProperties() {
#if defined(BOARD_HAS_SECRET_KEY)
  ArduinoCloud.setBoardId(BOARD_ID);
  ArduinoCloud.setSecretDeviceKey(SECRET_DEVICE_KEY);
#endif
#if defined(USE_NOTECARD) || defined(BOARD_HAS_WIFI) || defined(BOARD_HAS_GSM) || defined(BOARD_HAS_NB) || defined(BOARD_HAS_ETHERNET) || defined(BOARD_HAS_CATM1_NBIOT)
  ArduinoCloud.addProperty(switchButton, Permission::Write);
  ArduinoCloud.addProperty(oneShot, Permission::ReadWrite);
  ArduinoCloud.addProperty(minute, Permission::ReadWrite);
  ArduinoCloud.addProperty(hourly, Permission::ReadWrite);
  ArduinoCloud.addProperty(daily, Permission::ReadWrite);
  ArduinoCloud.addProperty(weekly, Permission::ReadWrite);
  ArduinoCloud.addProperty(monthly, Permission::ReadWrite);
  ArduinoCloud.addProperty(yearly, Permission::ReadWrite);
#elif defined(BOARD_HAS_LORA)
  ArduinoCloud.addProperty(switchButton, 1, Permission::Write);
#endif
}

#if defined(USE_NOTECARD)
  NotecardConnectionHandler ArduinoIoTPreferredConnection(NOTECARD_PRODUCT_UID);
#elif defined(BOARD_HAS_WIFI)
  WiFiConnectionHandler ArduinoIoTPreferredConnection(SECRET_WIFI_SSID, SECRET_WIFI_PASS);
#elif defined(BOARD_HAS_GSM)
  GSMConnectionHandler ArduinoIoTPreferredConnection(SECRET_PIN, SECRET_APN, SECRET_LOGIN, SECRET_PASS);
#elif defined(BOARD_HAS_LORA)
  LoRaConnectionHandler ArduinoIoTPreferredConnection(SECRET_APP_EUI, SECRET_APP_KEY, _lora_band::EU868, NULL, _lora_class::CLASS_A);
#elif defined(BOARD_HAS_NB)
  NBConnectionHandler ArduinoIoTPreferredConnection(SECRET_PIN, SECRET_APN, SECRET_LOGIN, SECRET_PASS);
#elif defined(BOARD_HAS_CATM1_NBIOT)
  CatM1ConnectionHandler ArduinoIoTPreferredConnection(SECRET_PIN, SECRET_APN, SECRET_LOGIN, SECRET_PASS);
#elif defined(BOARD_HAS_ETHERNET)
  /* DHCP mode */
  //EthernetConnectionHandler ArduinoIoTPreferredConnection;
  /* Manual mode. It will fallback in DHCP mode if SECRET_OPTIONAL_IP is invalid or equal to "0.0.0.0" */
  EthernetConnectionHandler ArduinoIoTPreferredConnection(SECRET_OPTIONAL_IP, SECRET_OPTIONAL_DNS, SECRET_OPTIONAL_GATEWAY, SECRET_OPTIONAL_NETMASK);
#endif
