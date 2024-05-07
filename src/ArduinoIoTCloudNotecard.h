/*
   This file is part of ArduinoIoTCloud.

   Copyright 2024 Blues (http://www.blues.com/)

   This software is released under the GNU General Public License version 3,
   which covers the main part of arduino-cli.
   The terms of this license can be found at:
   https://www.gnu.org/licenses/gpl-3.0.en.html

   You can be released from the requirements of the above licenses by purchasing
   a commercial license. Buying such a license is mandatory if you want to
   modify or otherwise use the software for commercial activities involving the
   Arduino software without disclosing the source code of your own applications.
   To purchase a commercial license, send an email to license@arduino.cc.
*/

#ifndef ARDUINO_IOT_CLOUD_NOTECARD_H
#define ARDUINO_IOT_CLOUD_NOTECARD_H

/******************************************************************************
 * INCLUDE
 ******************************************************************************/

#include "ArduinoIoTCloud.h"

/******************************************************************************
 * DEFINES
 ******************************************************************************/

#define USE_LIGHT_PAYLOADS (false)

/******************************************************************************
 * TYPEDEF
 ******************************************************************************/

typedef bool (*onOTARequestCallbackFunc)(void);

/******************************************************************************
 * CLASS DECLARATION
 ******************************************************************************/

class ArduinoIoTCloudNotecard : public ArduinoIoTCloudClass
{
  public:

             ArduinoIoTCloudNotecard();
    virtual ~ArduinoIoTCloudNotecard() { }

    virtual void update        () override;
    virtual int  connected     () override;
    virtual void printDebugInfo() override;

    int begin(ConnectionHandler &connection, int interrupt_pin = -1);

#ifdef BOARD_HAS_SECRET_KEY
    inline void setBoardId        (String const & device_id) { setDeviceId(device_id); }
    inline void setSecretDeviceKey(String const & secret_device_key)  { _secret_device_key = secret_device_key; }
#endif

  private:

    enum class State
    {
      ConnectPhy,
      SyncTime,
      SendDeviceProperties,
      SubscribeDeviceTopic,
      WaitDeviceConfig,
      Connected,
    };

    uint32_t _last_poll_ms;
    uint32_t _last_device_subscribe_attempt_tick;
    uint32_t _subscribe_retry_delay;
    size_t _last_device_subscribe_cnt;
    int _interrupt_pin;
    State _state;
    volatile bool _data_available;

#ifdef BOARD_HAS_SECRET_KEY
    String _secret_device_key;
#endif

    // OTA member variables
    bool _ota_cap;
    int _ota_error;
    String _ota_img_sha256;
    String _ota_url;
    bool _ota_req;
    bool _ask_user_before_executing_ota;
    onOTARequestCallbackFunc _get_ota_confirmation;

    State handle_ConnectPhy();
    State handle_SyncTime();
    State handle_SendDeviceProperties();
    State handle_SubscribeDeviceTopic();
    State handle_WaitDeviceConfig();
    State handle_Connected();

    bool available (void);
    void checkOTARequest(void);
    void decodePropertiesFromCloud(void);
    void requestThingIdFromNotehub(void);
    void sendDevicePropertiesToCloud(void);
    void sendDevicePropertyToCloud(String const name);
    void sendThingPropertiesToCloud(void);

    friend void ISR_dataAvailable (void);
};

/******************************************************************************
 * EXTERN DECLARATION
 ******************************************************************************/

extern ArduinoIoTCloudNotecard ArduinoCloud;

#endif
