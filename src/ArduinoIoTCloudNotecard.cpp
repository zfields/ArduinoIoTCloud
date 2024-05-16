/*
   This file is part of ArduinoIoTCloud.

   Copyright 2019 ARDUINO SA (http://www.arduino.cc/)

   This software is released under the GNU General Public License version 3,
   which covers the main part of arduino-cli.
   The terms of this license can be found at:
   https://www.gnu.org/licenses/gpl-3.0.en.html

   You can be released from the requirements of the above licenses by purchasing
   a commercial license. Buying such a license is mandatory if you want to modify or
   otherwise use the software for commercial activities involving the Arduino
   software without disclosing the source code of your own applications. To purchase
   a commercial license, send an email to license@arduino.cc.
*/

/******************************************************************************
 * INCLUDE
 ******************************************************************************/

#include "AIoTC_Config.h"

#if defined(USE_NOTECARD)

#include <algorithm>

#include "ArduinoIoTCloudNotecard.h"
#include "Arduino_NotecardConnectionHandler.h"
#include "cbor/CBOREncoder.h"

#if OTA_ENABLED
#include "utility/ota/OTA.h"
#endif

/******************************************************************************
   CONSTANTS
 ******************************************************************************/

static size_t const CBOR_NOTE_MSG_MAX_SIZE = 255;
static size_t const POLL_INTERVAL_MS = 10000;

/******************************************************************************
   LOCAL MODULE FUNCTIONS
 ******************************************************************************/

unsigned long getTime()
{
  return ArduinoCloud.getInternalTime();
}

void ISR_dataAvailable(void)
{
  ArduinoCloud._data_available = true;
}

void setThingIdOutdated()
{
  ArduinoCloud.setThingIdOutdatedFlag();
}

void updateTimezoneInfo()
{
  ArduinoCloud.updateInternalTimezoneInfo();
}

/******************************************************************************
   CTOR/DTOR
 ******************************************************************************/

ArduinoIoTCloudNotecard::ArduinoIoTCloudNotecard()
:
  _last_poll_ms{0},
  _last_device_subscribe_attempt_tick{0},
  _subscribe_retry_delay{0},
  _last_device_subscribe_cnt{0},
  _interrupt_pin{-1},
  _state{State::ConnectPhy},
  _data_available{false},
#if defined(BOARD_HAS_SECRET_KEY)
  _secret_device_key{""},
#endif
  _ota_cap{false},
  //OTA: _ota_error{static_cast<int>(OTAError::None)},
  _ota_error{0},
  _ota_img_sha256{"Inv."},
  _ota_url{""},
  _ota_req{false},
  _ask_user_before_executing_ota{false},
  _get_ota_confirmation{nullptr}
{

}

/******************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 ******************************************************************************/

int ArduinoIoTCloudNotecard::connected()
{
  return (_connection->check() == NetworkConnectionState::CONNECTED);
}

int ArduinoIoTCloudNotecard::begin(ConnectionHandler &connection, int interrupt_pin) {
  _connection = &connection;

  // Configure the interrupt pin
  if (interrupt_pin >= 0) {
    ::pinMode(interrupt_pin, INPUT);
    ::attachInterrupt(digitalPinToInterrupt(interrupt_pin), ISR_dataAvailable, RISING);
    _interrupt_pin = interrupt_pin;
  }

  // Begin the Notecard time service
  _time_service.begin(&connection);

  // Configure the Device and Thing property containers
  Property* p;
  p = new CloudWrapperString(_lib_version);
  addPropertyToContainer(_device_property_container, *p, "LIB_VERSION", Permission::Read);
  p = new CloudWrapperBool(_ota_cap);
  addPropertyToContainer(_device_property_container, *p, "OTA_CAP", Permission::Read);
  p = new CloudWrapperInt(_ota_error);
  addPropertyToContainer(_device_property_container, *p, "OTA_ERROR", Permission::Read);
  p = new CloudWrapperString(_ota_img_sha256);
  addPropertyToContainer(_device_property_container, *p, "OTA_SHA256", Permission::Read);
  p = new CloudWrapperString(_ota_url);
  addPropertyToContainer(_device_property_container, *p, "OTA_URL", Permission::ReadWrite);
  p = new CloudWrapperBool(_ota_req);
  addPropertyToContainer(_device_property_container, *p, "OTA_REQ", Permission::ReadWrite);
  p = new CloudWrapperString(_thing_id);
  addPropertyToContainer(_device_property_container, *p, "thing_id", Permission::ReadWrite).onUpdate(setThingIdOutdated);

  addPropertyReal(_tz_offset, "tz_offset", Permission::ReadWrite).onSync(CLOUD_WINS).onUpdate(updateTimezoneInfo);
  addPropertyReal(_tz_dst_until, "tz_dst_until", Permission::ReadWrite).onSync(CLOUD_WINS).onUpdate(updateTimezoneInfo);

  return 1;
}

void ArduinoIoTCloudNotecard::update()
{
  /* Run through the state machine. */
  State next_state = _state;
  switch (_state)
  {
  case State::ConnectPhy:           next_state = handle_ConnectPhy();           break;
  case State::SyncTime:             next_state = handle_SyncTime();             break;
  case State::SendDeviceProperties: next_state = handle_SendDeviceProperties(); break;
  case State::SubscribeDeviceTopic: next_state = handle_SubscribeDeviceTopic(); break;
  case State::WaitDeviceConfig:     next_state = handle_WaitDeviceConfig();     break;
  case State::Connected:            next_state = handle_Connected();            break;
  }
  _state = next_state;

  if (State::Connected == _state) {
    checkOTARequest();
  }
}

void ArduinoIoTCloudNotecard::printDebugInfo()
{
  // Initiate the connection to the Notecard to enable
  // the discovery of unique hardware identifiers
  NetworkConnectionState conn_state = _connection->check();

  if (NetworkConnectionState::INIT == conn_state)
  {
    // Delay for NotecardConnectionHander initialization
    DEBUG_VERBOSE("Awaiting Notecard connection...");
    delay(CHECK_INTERVAL_TABLE[static_cast<unsigned int>(NetworkConnectionState::INIT)]);
    conn_state = _connection->check();
  }

  // Fetch the unique device identifier from the Notecard Connection Handler
  NotecardConnectionHandler *notecard_connection = reinterpret_cast<NotecardConnectionHandler *>(_connection);
  String device_id(getDeviceId());
  if (device_id == "xxxxxxxx-xxxx-xxxx-xxxx-xxxxxxxxxxxx" || device_id.length() == 0) {
    setDeviceId(notecard_connection->getArduinoDeviceId());
  }
  String arduino_thing_id = getThingId();

  // Print the debug information
  DEBUG_INFO("***** Arduino IoT Cloud Notecard - configuration info *****");
  DEBUG_INFO("Notecard UID: %s", notecard_connection->getNotecardUid().c_str());
  DEBUG_INFO("Arduino Device ID: %s", getDeviceId().c_str());
  if (NetworkConnectionState::CONNECTED == conn_state)
  {
    DEBUG_INFO("Arduino Thing ID: %s", arduino_thing_id.c_str());
  }
  else
  {
    DEBUG_INFO("Arduino Thing ID: awaiting connection...");
  }
}

/******************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 ******************************************************************************/

ArduinoIoTCloudNotecard::State ArduinoIoTCloudNotecard::handle_ConnectPhy()
{
  // Set Thing ID to a known state before establishing a connection
  _thing_id = "";
  clrThingIdOutdatedFlag();

  if (_connection->check() == NetworkConnectionState::CONNECTED)
    return State::SyncTime;
  else
    return State::ConnectPhy;
}

ArduinoIoTCloudNotecard::State ArduinoIoTCloudNotecard::handle_SyncTime()
{
  DEBUG_DEBUG("ArduinoIoTCloudNotecard::%s internal clock configured to posix timestamp %d", __FUNCTION__, static_cast<unsigned long>(_time_service.getTime()));
  return State::SendDeviceProperties;
}

ArduinoIoTCloudNotecard::State ArduinoIoTCloudNotecard::handle_SendDeviceProperties()
{
  if (!connected())
  {
    DEBUG_ERROR("ArduinoIoTCloudNotecard::%s connection to Notehub lost", __FUNCTION__);
    return State::ConnectPhy;
  }

#if defined(BOARD_HAS_SECRET_KEY)
  // Provide device ID to Notehub
  String device_id = getDeviceId();
  if (J *req = NoteNewRequest("hub.set"))
  {
    JAddStringToObject(req, "sn", device_id.c_str());
    NoteRequest(req);
  }
  if (device_id.length() > 0) {
    DEBUG_DEBUG("Reporting device ID to Cloud: %s", device_id.c_str());
  }

  // Confirm `_secret_device_key` is not the default from the examples
  if (_secret_device_key != "my-device-password") {
    // Provide `SECRET_DEVICE_KEY` to Notehub
    if (J *req = NoteNewRequest("env.set"))
    {
      JAddStringToObject(req, "name", "arduino_iot_cloud_secret_key");
      JAddStringToObject(req, "text", _secret_device_key.c_str());
      NoteRequest(req);
    }
  }
#endif

  sendDevicePropertiesToCloud();
  return State::SubscribeDeviceTopic;
}

ArduinoIoTCloudNotecard::State ArduinoIoTCloudNotecard::handle_SubscribeDeviceTopic()
{
  if (!connected())
  {
    DEBUG_ERROR("ArduinoIoTCloudNotecard::%s connection to Notehub lost", __FUNCTION__);
    return State::ConnectPhy;
  }

  if (_last_device_subscribe_cnt > AIOT_CONFIG_LASTVALUES_SYNC_MAX_RETRY_CNT)
  {
    DEBUG_ERROR("ArduinoIoTCloudNotecard::%s failed to receive Thing ID", __FUNCTION__);
    _last_device_subscribe_cnt = 0;
    _last_device_subscribe_attempt_tick = 0;
    execCloudEventCallback(ArduinoIoTCloudEvent::DISCONNECT);
    return State::ConnectPhy;
  }

  /* No device configuration reply. Wait: 5s -> 10s -> 20s -> 40s */
  _subscribe_retry_delay = (1 << _last_device_subscribe_cnt) * AIOT_CONFIG_DEVICE_TOPIC_SUBSCRIBE_RETRY_DELAY_ms;
  _subscribe_retry_delay = std::min(_subscribe_retry_delay, static_cast<unsigned long>(AIOT_CONFIG_MAX_DEVICE_TOPIC_SUBSCRIBE_RETRY_DELAY_ms));
  _last_device_subscribe_attempt_tick = millis();
  _last_device_subscribe_cnt++;

  return State::WaitDeviceConfig;
}

ArduinoIoTCloudNotecard::State ArduinoIoTCloudNotecard::handle_WaitDeviceConfig()
{
  if (!connected())
  {
    DEBUG_ERROR("ArduinoIoTCloudNotecard::%s connection to Notehub lost", __FUNCTION__);
    return State::ConnectPhy;
  }

  /* Decode available data. */
  if (available())
    decodePropertiesFromCloud();

  /* Check if a Thing ID has been returned from the Arduino IoT Cloud. */
  if (getThingIdOutdatedFlag())
  {
    clrThingIdOutdatedFlag();
    const String thing_id = getThingId();
    if (thing_id.length() > 0)
    {
      DEBUG_INFO("Connected to Arduino IoT Cloud");
      DEBUG_INFO("Thing ID: %s", thing_id.c_str());
      return State::Connected;
    } else {
      DEBUG_WARNING("ArduinoIoTCloudNotecard::%s received an empty Thing ID", __FUNCTION__);
      requestThingIdFromNotehub();
      return State::SubscribeDeviceTopic;
    }
  }

  /* Configuration not received or device not attached to a valid thing. */
  if ((millis() - _last_device_subscribe_attempt_tick) > _subscribe_retry_delay)
  {
    {
      DEBUG_WARNING("ArduinoIoTCloudNotecard::%s timed out waiting for valid Thing ID", __FUNCTION__);
      requestThingIdFromNotehub();
      return State::SubscribeDeviceTopic;
    }
  }

  return State::WaitDeviceConfig;
}

ArduinoIoTCloudNotecard::State ArduinoIoTCloudNotecard::handle_Connected()
{
  if (!connected())
  {
    DEBUG_ERROR("ArduinoIoTCloudNotecard::%s connection to Notehub lost", __FUNCTION__);
    return State::ConnectPhy;
  }
  if (getThingIdOutdatedFlag() && !getThingId().length()) {
    DEBUG_INFO("Disconnected from Arduino IoT Cloud");
    return State::ConnectPhy;
  }

  /* Check if a primitive property wrapper is locally changed. */
  updateTimestampOnLocallyChangedProperties(_thing_property_container);

  /* Decode available data. */
  if (available())
    decodePropertiesFromCloud();

  /* If properties need updating sent them to the cloud. */
  sendThingPropertiesToCloud();

  return State::Connected;
}

bool ArduinoIoTCloudNotecard::available(void)
{
  bool result;

  const bool interrupts_enabled = (_interrupt_pin >= 0);
  bool check_data = true;
  if (interrupts_enabled) {
    check_data = (_data_available || ((::millis() - _last_poll_ms) > POLL_INTERVAL_MS));
  }

  if (check_data) {
    result = _connection->available();
    _data_available = ::digitalRead(_interrupt_pin);
    _last_poll_ms = ::millis();
  } else {
    result = false;
  }

  return result;
}

void ArduinoIoTCloudNotecard::checkOTARequest(void) {
  /* Request a OTA download if the hidden property
  * OTA request has been set.
  */

  if (_ota_req)
  {
    bool const ota_execution_allowed_by_user = (_get_ota_confirmation != nullptr && _get_ota_confirmation());
    bool const perform_ota_now = ota_execution_allowed_by_user || !_ask_user_before_executing_ota;
    if (perform_ota_now) {
      /* Clear the error flag. */
      //OTA: _ota_error = static_cast<int>(OTAError::None);
      _ota_error = 0;
      /* Clear the request flag. */
      _ota_req = false;
      /* Transmit the cleared request flags to the cloud. */
      sendDevicePropertyToCloud("OTA_REQ");
      /* Call member function to handle OTA request. */
      //OTA: _ota_error = OTA::onRequest(_ota_url, _connection->getInterface());
      DEBUG_WARNING("OTA request received. OTA is not currently supported by Notecard.");
      /* If something fails send the OTA error to the cloud */
      sendDevicePropertyToCloud("OTA_ERROR");
    }
  }

  /* Check if we have received the OTA_URL property and provide
  * echo to the cloud.
  */
  sendDevicePropertyToCloud("OTA_URL");
}

void ArduinoIoTCloudNotecard::decodePropertiesFromCloud(void)
{
  uint8_t note_buf[CBOR_NOTE_MSG_MAX_SIZE];
  size_t bytes_received;
  for (bytes_received = 0;
       _connection->available() && (bytes_received < CBOR_NOTE_MSG_MAX_SIZE);
       bytes_received++)
  {
    note_buf[bytes_received] = _connection->read();
  }
  if (bytes_received > 0) {
    DEBUG_VERBOSE("Received %d bytes from cloud. Decoding...", bytes_received);
    NotecardConnectionHandler *notecard_connection = reinterpret_cast<NotecardConnectionHandler *>(_connection);
    switch (notecard_connection->getTopicType()) {
      case NotecardConnectionHandler::TopicType::Device:
      case NotecardConnectionHandler::TopicType::Notehub:
        CBORDecoder::decode(_device_property_container, note_buf, bytes_received);
        break;
      case NotecardConnectionHandler::TopicType::Shadow:
      case NotecardConnectionHandler::TopicType::Thing:
        CBORDecoder::decode(_thing_property_container, note_buf, bytes_received);
        break;
      default:
        DEBUG_WARNING("Unable to decode unknown topic type");
        break;
    }
  }
}

void ArduinoIoTCloudNotecard::requestThingIdFromNotehub(void)
{
  NotecardConnectionHandler *notecard_connection = reinterpret_cast<NotecardConnectionHandler *>(_connection);

  notecard_connection->setTopicType(NotecardConnectionHandler::TopicType::Notehub);
  notecard_connection->write(nullptr, 0);
}

void ArduinoIoTCloudNotecard::sendDevicePropertiesToCloud(void)
{
  int bytes_encoded = 0;
  uint8_t data[CBOR_NOTE_MSG_MAX_SIZE];
  PropertyContainer ro_device_property_container;
  unsigned int last_device_property_index = 0;
  NotecardConnectionHandler *notecard_connection = reinterpret_cast<NotecardConnectionHandler *>(_connection);

  // Iterate over the list of read-only device properties
  std::list<String> ro_device_property_list {"LIB_VERSION", "OTA_CAP", "OTA_ERROR", "OTA_SHA256"};

  for (
    std::list<String>::iterator it = ro_device_property_list.begin();
    it != ro_device_property_list.end();
    ++it
  ) {
    Property* p = getProperty(this->_device_property_container, *it);
    if(p != nullptr) {
      addPropertyToContainer(ro_device_property_container, *p, p->name(), p->isWriteableByCloud() ? Permission::ReadWrite : Permission::Read);
    }
  }

  if (CBOREncoder::encode(ro_device_property_container, data, sizeof(data), bytes_encoded, last_device_property_index, USE_LIGHT_PAYLOADS) == CborNoError) {
    if (bytes_encoded > 0) {
      notecard_connection->setTopicType(NotecardConnectionHandler::TopicType::Device);
      notecard_connection->write(data, bytes_encoded);
    }
  } else {
    DEBUG_ERROR("Failed to encode Device properties");
  }
}

void ArduinoIoTCloudNotecard::sendDevicePropertyToCloud(String const name_)
{
  int bytes_encoded = 0;
  uint8_t data[CBOR_NOTE_MSG_MAX_SIZE];
  PropertyContainer temp_device_property_container;
  unsigned int last_device_property_index = 0;
  NotecardConnectionHandler *notecard_connection = reinterpret_cast<NotecardConnectionHandler *>(_connection);

  Property* p = getProperty(this->_device_property_container, name_);
  if(p != nullptr)
  {
    addPropertyToContainer(temp_device_property_container, *p, p->name(), p->isWriteableByCloud() ? Permission::ReadWrite : Permission::Read);
    if (CBOREncoder::encode(temp_device_property_container, data, sizeof(data), bytes_encoded, last_device_property_index, USE_LIGHT_PAYLOADS) == CborNoError) {
      if (bytes_encoded > 0) {
        notecard_connection->setTopicType(NotecardConnectionHandler::TopicType::Device);
        notecard_connection->write(data, bytes_encoded);
      }
    } else {
      DEBUG_ERROR("Failed to encode Device property: %s", name_.c_str());
    }
  }
}

void ArduinoIoTCloudNotecard::sendThingPropertiesToCloud(void)
{
  int bytes_encoded = 0;
  uint8_t data[CBOR_NOTE_MSG_MAX_SIZE];
  NotecardConnectionHandler *notecard_connection = reinterpret_cast<NotecardConnectionHandler *>(_connection);

  if (CBOREncoder::encode(_thing_property_container, data, sizeof(data), bytes_encoded, _last_checked_property_index, USE_LIGHT_PAYLOADS) == CborNoError) {
    if (bytes_encoded > 0) {
      notecard_connection->setTopicType(NotecardConnectionHandler::TopicType::Thing);
      notecard_connection->write(data, bytes_encoded);
    }
  } else {
    DEBUG_ERROR("Failed to encode Thing properties");
  }
}

/******************************************************************************
 * EXTERN DEFINITION
 ******************************************************************************/

ArduinoIoTCloudNotecard ArduinoCloud;

#endif  // USE_NOTECARD
