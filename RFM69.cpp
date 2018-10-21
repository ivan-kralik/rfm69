#include "RFM69.h"
#include "RFM69_registers.h"
#include <SPI.h>
#include <DebugUtils.h>

RFM69::RFM69(uint8_t dio0_pin, uint8_t ss_pin, uint8_t rst_pin):m_dio0_pin(dio0_pin),m_ss_pin(ss_pin),m_rst_pin(rst_pin)
{
  ;
}

void RFM69::begin(uint8_t network_id, uint8_t node_id, uint8_t channel)
{
  m_network_id = network_id;
  m_node_id = node_id;
  m_channel = channel;
  
  pinMode(m_dio0_pin, INPUT);
  pinMode(m_ss_pin, OUTPUT);
  pinMode(m_rst_pin, OUTPUT);
  digitalWrite(m_ss_pin, HIGH);
  digitalWrite(m_rst_pin, LOW);

  SPI.begin();

  DEBUGPRINTLN1("Resetting RFM69 module");
  
  reset();

  if (getMode() != RFM69_MODE_STBY) {
    DEBUGPRINTLN1("RFM69 module not present or wired wrong");

    return;
  } else {
    m_ready = true;
  }
  
  DEBUGPRINTLN1("Begin RFM69 OSC calibration");
  calibrateOsc();
  DEBUGPRINTLN1("Done RFM69 OSC calibration");
  
  setChannel(m_channel);
  
  writeReg(RFM69_REG_LISTEN_1, 0x94);
  writeReg(RFM69_REG_LISTEN_2, 0xF5);
  writeReg(RFM69_REG_LISTEN_3, 0x28);
  writeReg(RFM69_REG_RX_TIMEOUT_2, 0x40);
  writeReg(RFM69_REG_PACKET_CONFIG_1, 0x94);
  writeReg(RFM69_REG_PACKET_CONFIG_2, 0x12);
  writeReg(RFM69_REG_FIFO_THRESH, 0x8F);
  writeReg(RFM69_REG_PAYLOAD_LENGTH, RFM69_MAX_PACKET_LENGTH);
  writeReg(RFM69_REG_RSSI_THRESH, ((uint8_t) -m_rssi_threshold) * 2);
  writeReg(RFM69_REG_NODE_ADRS, m_node_id);
  writeReg(RFM69_REG_BROADCAST_ADRS, getBroadcastId());
  writeReg(RFM69_REG_SYNC_CONFIG, 0x80);
  writeReg(RFM69_REG_SYNC_VALUE_1, m_network_id);
  writeReg(RFM69_REG_TEST_DAGC, 0x30);
}

void RFM69::updateRssi()
{
  m_rssi = readReg(RFM69_REG_RSSI_VALUE);
}

void RFM69::update()
{
  if (!m_ready) {
    return;
  }

  if (m_state == SEND_LBT && isModeReady()) {
    if (isChannelFree()) {
      beginTransmission();
      DEBUGPRINTLN1("Transmitting message");
      transmitMessage();
      m_state = SEND_WAIT;
    }
  } else if (m_state == SEND_WAIT && isTransmissionFinished()) {
    endTransmission();

    if (m_recipient == getBroadcastId()) {
      restoreDefaultMode();
    } else {
      setMode(RFM69_MODE_RX);
      DEBUGPRINTLN1("Awaiting ack");
      m_ack_wait_start = millis();
      m_state = SEND_WAIT_ACK;
    }
  } else if (m_state == SEND_WAIT_ACK && isPayloadReady()) {
    uint8_t ackLength;
    int8_t rssi;
    receivePacket(m_ack_buffer, &ackLength, sizeof(m_ack_buffer), &rssi);

    if (m_ack_buffer[4] == m_send_seq_num) {
      DEBUGPRINTLN1("Ack received");
      restoreDefaultMode();
    }
  } else if (m_state == SEND_WAIT_ACK && millis() - m_ack_wait_start > 5) {
    if (m_send_retries > 0) {
      updateRssi();

      if (getRssi() <= m_rssi_threshold) {
        beginTransmission();
        DEBUGPRINTLN1("Re-transmitting message");
        transmitMessage();
        m_send_retries--;
        m_state = SEND_WAIT;
      }
    } else {
      DEBUGPRINTLN1("Ack not received");
      restoreDefaultMode();
    }
  } else if (m_state == ACK_WAIT && isTransmissionFinished()) {
    endTransmission();

    if (!isInHistory(m_ack_recipient, m_ack_seq_num)) {
      putInHistory(m_ack_recipient, m_ack_seq_num);
      doReceiveCallback();
    }

    restoreDefaultMode();
  } else if (m_state == RECV && isPayloadReady()) {
    receivePacket(m_packet_buffer, &m_packet_length, RFM69_MAX_PACKET_LENGTH, &m_recv_rssi);
    uint8_t recipient = m_packet_buffer[2];
    
    if (recipient != getBroadcastId()) {
      m_ack_recipient = recipient;
      m_ack_seq_num = m_packet_buffer[3];
      
      beginTransmission();
      DEBUGPRINTLN1("Packet received, transmitting ack");
      transmitAck();
      m_state = ACK_WAIT;
    } else {
      DEBUGPRINTLN1("Packet received");
      doReceiveCallback();
      restoreDefaultMode();
    }
  }
}

bool RFM69::isSending() {
  return m_state == SEND_LBT || m_state == SEND_WAIT || m_state == SEND_WAIT_ACK;
}

void RFM69::waitForSend() {
  while (isSending()) {
    update();
    yield();
  }
}

uint8_t RFM69::getNetworkId()
{
  return m_network_id;
}

uint8_t RFM69::getNodeId()
{
  return m_node_id;
}

uint8_t RFM69::getBroadcastId()
{
  return 255;
}

void RFM69::setHighPower(bool high_power)
{
  m_high_power = high_power;
}

bool RFM69::isHighPower()
{
  return m_high_power;
}

void RFM69::setReceiveHandler(RFM69_receive_handler handler)
{
  m_receive_handler = handler;
}

void RFM69::sendMessage(uint8_t recipient, uint8_t * buffer, uint8_t len)
{
  m_seq_num = m_seq_num == 255 ? 1 : m_seq_num + 1;
  m_send_seq_num = m_seq_num;
  m_send_retries = 50;
  m_recipient = recipient;
  m_packet_length = len > RFM69_MAX_PACKET_LENGTH ? RFM69_MAX_PACKET_LENGTH : len;

  for (uint8_t i = 0; i < m_packet_length; i++) {
    m_packet_buffer[i] = buffer[i];
  }

  setPayloadReadyDio0Interrupt();
  setMode(RFM69_MODE_RX);
  
  DEBUGPRINTLN1("Waiting for channel to be free");

  m_state = SEND_LBT;
}

void RFM69::transmitAck()
{ 
  select();
  sendQueueWriteCommand();
  
  SPI.transfer(4);
  SPI.transfer(m_ack_recipient);
  SPI.transfer(getNodeId());
  SPI.transfer(0);
  SPI.transfer(m_ack_seq_num);
  
  unselect();
}

void RFM69::transmitMessage()
{ 
  select();
  sendQueueWriteCommand();
  
  SPI.transfer(m_packet_length + 4);
  SPI.transfer(m_recipient);
  SPI.transfer(getNodeId());
  SPI.transfer(m_send_seq_num);
  SPI.transfer(0);

  for (uint8_t i = 0; i < m_packet_length; i++) {
    SPI.transfer(((uint8_t*) m_packet_buffer)[i]);
  }
  
  unselect();
}

bool RFM69::isChannelFree()
{
  if (isPayloadReady()) {
    writeReg(RFM69_REG_PACKET_CONFIG_2, readReg(RFM69_REG_PACKET_CONFIG_2) | 0x04);
  }
    
  updateRssi();
    
  return getRssi() <= m_rssi_threshold;
}

void RFM69::beginTransmission()
{
  setMode(RFM69_MODE_STBY);
  
  if (isHighPower()) {
    enableHighPower();
  }
      
  setMode(RFM69_MODE_TX);
  setPacketSentDio0Interrupt();

  m_tx_success = false;
  m_tx_start = millis();
}

bool RFM69::isTransmissionFinished()
{
  return ((m_tx_success = (digitalRead(m_dio0_pin) == 1)) || millis() - m_tx_start >= 100);
}

void RFM69::endTransmission()
{
  setMode(RFM69_MODE_STBY);
  disableHighPower();
  setPayloadReadyDio0Interrupt();

  if (m_tx_success) {
    DEBUGPRINTLN1("Transmission successful");
  }
}

void RFM69::doReceiveCallback()
{
  if (m_receive_handler != NULL) {
    (*m_receive_handler)(m_packet_buffer[2], m_recv_rssi, m_packet_buffer + 5, m_packet_length - 5);
  }
}

void RFM69::receivePacket(uint8_t * buffer, uint8_t * length, uint8_t maxLength, int8_t * rssi)
{
  updateRssi();
  *rssi = getRssi();

  select();
  sendQueueReadCommand();
  
  buffer[0] = SPI.transfer(0);
  *length = buffer[0] + 1;

  for (uint8_t i = 1; i < buffer[0] + 1; i++) {
    uint8_t data = SPI.transfer(0);

    if (i < maxLength) {
      buffer[i] = data;
    }
  }

  unselect();
}

void RFM69::receive()
{
  m_default_mode = RFM69_RECEIVE;
  restoreDefaultMode();
}

void RFM69::listen()
{
  m_default_mode = RFM69_LISTEN;
  restoreDefaultMode();
}

void RFM69::restoreDefaultMode()
{
  if (m_default_mode == RFM69_STANDBY) {
    DEBUGPRINTLN1("Standby");
    
    setMode(RFM69_MODE_STBY);
  } else if (m_default_mode == RFM69_RECEIVE) {
    DEBUGPRINTLN1("Receiving");

    setPayloadReadyDio0Interrupt();
    setMode(RFM69_MODE_RX);

    m_state = RECV;
  } else if (m_default_mode == RFM69_LISTEN) {
    DEBUGPRINTLN1("Listening");
  
    setMode(RFM69_MODE_STBY);
    waitForModeReady();
    setPayloadReadyDio0Interrupt();
    writeReg(RFM69_REG_MODE, RFM69_MODE_STBY_LISTEN);

    m_state = RECV;
  }
}

bool RFM69::isInHistory(uint8_t sender, uint8_t seq_num)
{
  for (uint8_t i = 0; i < m_history_length; i++) {
    if (m_history[i].sender == sender && m_history[i].seq_num == seq_num) {
      return millis() - m_history[i].received < 1000;
    }
  }
  
  return false;
}

void RFM69::putInHistory(uint8_t sender, uint8_t seq_num)
{
  uint8_t position = m_history_end;

  for (uint8_t i = 0; i < m_history_length; i++) {
    if (m_history[i].sender == sender) {
      position = i;
      break;
    }
  }
  
  m_history[position].sender = sender;
  m_history[position].seq_num = seq_num;
  m_history[position].received = millis();
  m_history_length = position == m_history_end && m_history_length < RFM69_HISTORY_SIZE ? m_history_length + 1 : m_history_length;
  m_history_end = (m_history_end + 1) % RFM69_HISTORY_SIZE;
}

void RFM69::setChannel(uint8_t channel)
{
  if (!m_ready) {
    return;
  }
  
  setFrequency(m_frequency + (m_channel * m_channel_step));
}

void RFM69::calibrateOsc()
{ 
  writeReg(RFM69_REG_OSC_1, 0x00);
  while (readReg(RFM69_REG_OSC_1) & 0x40 != 0x40);
}

void RFM69::setMode(uint8_t mode)
{
  writeReg(RFM69_REG_MODE, mode);
}

uint8_t RFM69::getMode()
{
  return readReg(RFM69_REG_MODE);
}

void RFM69::reset()
{ 
  digitalWrite(m_rst_pin, HIGH);
  delayMicroseconds(200);
  digitalWrite(m_rst_pin, LOW);
  delay(10);
}

int8_t RFM69::getRssi()
{
  return -(m_rssi / 2);
}

void RFM69::setFrequency(uint32_t frequency)
{
  uint32_t frf = frequency / RFM69_FSTEP;
  
  writeReg(RFM69_REG_FRF_MSB, (uint8_t) (frf >> 16));
  writeReg(RFM69_REG_FRF_MID, (uint8_t) (frf >> 8));
  writeReg(RFM69_REG_FRF_LSB, (uint8_t) (frf));
}

void RFM69::setPacketSentDio0Interrupt()
{
  writeReg(RFM69_REG_DIO_MAPPING_1, 0x00);
}

void RFM69::setPayloadReadyDio0Interrupt()
{
  writeReg(RFM69_REG_DIO_MAPPING_1, 0x40);
}

void RFM69::setRssiDio0Interrupt()
{
  writeReg(RFM69_REG_DIO_MAPPING_1, 0xC0);
}

bool RFM69::isModeReady()
{
  return (bool) (readReg(RFM69_REG_IRQ_FLAGS_1) & 0x80);
}

bool RFM69::isPayloadReady()
{
  //return (bool) (readReg(RFM69_REG_IRQ_FLAGS_2) & 0x04);

  return (bool) digitalRead(m_dio0_pin);
}

void RFM69::sendQueueWriteCommand()
{
  SPI.transfer(0x00 | 0x80);
}

void RFM69::sendQueueReadCommand()
{
  SPI.transfer(0x00);
}

void RFM69::enableHighPower()
{
  writeReg(RFM69_REG_PA_LEVEL, 0x7F);
  writeReg(RFM69_REG_OCP, 0x0F);
  writeReg(RFM69_REG_TEST_PA_1, 0x5D);
  writeReg(RFM69_REG_TEST_PA_2, 0x7C);
}

void RFM69::disableHighPower()
{
  writeReg(RFM69_REG_PA_LEVEL, 0x9F);
  writeReg(RFM69_REG_OCP, 0x1A);
  writeReg(RFM69_REG_TEST_PA_1, 0x55);
  writeReg(RFM69_REG_TEST_PA_2, 0x70);
}

void RFM69::waitForModeReady()
{
  while (!isModeReady()) {
    yield();
  }
}

uint8_t RFM69::readReg(uint8_t addr)
{
  select();
  SPI.transfer(addr & 0x7F);
  uint8_t regval = SPI.transfer(0);
  unselect();
  
  return regval;
}

void RFM69::writeReg(uint8_t addr, uint8_t value)
{
  select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  unselect();
}

void RFM69::select()
{
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  
  #ifdef __AVR__
  SPI.setClockDivider(SPI_CLOCK_DIV4);
  #else
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  #endif
  
  digitalWrite(m_ss_pin, LOW);
}

void RFM69::unselect()
{
  digitalWrite(m_ss_pin, HIGH);
}
