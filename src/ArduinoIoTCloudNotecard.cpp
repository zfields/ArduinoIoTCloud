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

#include "ArduinoIoTCloudNotecard.h"
#include "cbor/CBOREncoder.h"

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

/******************************************************************************
   CTOR/DTOR
 ******************************************************************************/

ArduinoIoTCloudNotecard::ArduinoIoTCloudNotecard()
:
  _last_poll_ms{0},
  _state{State::ConnectPhy},
  _interrupt_pin{-1},
  _data_available{false}
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
  return 1;
}

void ArduinoIoTCloudNotecard::update()
{
  /* Run through the state machine. */
  State next_state = _state;
  switch (_state)
  {
  case State::ConnectPhy: next_state = handle_ConnectPhy(); break;
  case State::SyncTime:   next_state = handle_SyncTime();   break;
  case State::Connected:  next_state = handle_Connected();  break;
  }
  _state = next_state;
}

void ArduinoIoTCloudNotecard::printDebugInfo()
{
  DEBUG_INFO("***** Arduino IoT Cloud Notecard - configuration info *****");
  DEBUG_INFO("Thing ID: %s", getThingId().c_str());
}

/******************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 ******************************************************************************/

ArduinoIoTCloudNotecard::State ArduinoIoTCloudNotecard::handle_ConnectPhy()
{
  if (_connection->check() == NetworkConnectionState::CONNECTED)
    return State::SyncTime;
  else
    return State::ConnectPhy;
}

ArduinoIoTCloudNotecard::State ArduinoIoTCloudNotecard::handle_SyncTime()
{
  DEBUG_VERBOSE("ArduinoIoTCloudNotecard::%s internal clock configured to posix timestamp %d", __FUNCTION__, static_cast<unsigned long>(_time_service.getTime()));
  DEBUG_INFO("Connected to Arduino IoT Cloud");
  return State::Connected;
}

ArduinoIoTCloudNotecard::State ArduinoIoTCloudNotecard::handle_Connected()
{
  if (!connected())
  {
    DEBUG_ERROR("ArduinoIoTCloudNotecard::%s connection to gateway lost", __FUNCTION__);
    return State::ConnectPhy;
  }

  /* Check if a primitive property wrapper is locally changed. */
  updateTimestampOnLocallyChangedProperties(_thing_property_container);

  /* Decode available data. */
  if (available())
    decodePropertiesFromCloud();

  /* If properties need updating sent them to the cloud. */
  sendPropertiesToCloud();

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

void ArduinoIoTCloudNotecard::decodePropertiesFromCloud()
{
  uint8_t note_buf[CBOR_NOTE_MSG_MAX_SIZE];
  size_t bytes_received;
  for (bytes_received = 0;
       _connection->available() && (bytes_received < CBOR_NOTE_MSG_MAX_SIZE);
       bytes_received++)
  {
    note_buf[bytes_received] = _connection->read();
  }
  CBORDecoder::decode(_thing_property_container, note_buf, bytes_received);
}

void ArduinoIoTCloudNotecard::sendPropertiesToCloud()
{
  int bytes_encoded = 0;
  uint8_t data[CBOR_NOTE_MSG_MAX_SIZE];

  if (CBOREncoder::encode(_thing_property_container, data, sizeof(data), bytes_encoded, _last_checked_property_index, true) == CborNoError) {
    if (bytes_encoded > 0) {
      _connection->write(data, bytes_encoded);
    }
  }
}

/******************************************************************************
 * EXTERN DEFINITION
 ******************************************************************************/

ArduinoIoTCloudNotecard ArduinoCloud;

#endif  // USE_NOTECARD
