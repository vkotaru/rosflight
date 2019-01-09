/*
 * Copyright (c) 2017 Daniel Koch and James Jackson, BYU MAGICC Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file mavlink_comm.cpp
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#include <rosflight/mavrosflight/mavlink_comm.h>

#include <async_comm/serial.h>
#include <async_comm/udp.h>

namespace mavrosflight
{

MavlinkComm::MavlinkComm()
{}

MavlinkComm::~MavlinkComm()
{
  close();
}

bool MavlinkComm::open_serial(std::string port, unsigned int baud_rate)
{
  comm_ = std::unique_ptr<async_comm::Comm>(new async_comm::Serial(port, baud_rate));
  comm_->register_receive_callback(std::bind(&MavlinkComm::serial_callback, this, std::placeholders::_1, std::placeholders::_2));
  return comm_->init();
}

bool MavlinkComm::open_udp(std::string bind_host, uint16_t bind_port, std::string remote_host, uint16_t remote_port)
{
  comm_ = std::unique_ptr<async_comm::Comm>(new async_comm::UDP(bind_host, bind_port, remote_host, remote_port));
  comm_->register_receive_callback(std::bind(&MavlinkComm::serial_callback, this, std::placeholders::_1, std::placeholders::_2));
  return comm_->init();
}

void MavlinkComm::close()
{
  if (comm_)
  {
    comm_->close();
  }
}

void MavlinkComm::register_mavlink_listener(MavlinkListenerInterface * const listener)
{
  if (listener == NULL)
    return;

  bool already_registered = false;
  for (int i = 0; i < listeners_.size(); i++)
  {
    if (listener == listeners_[i])
    {
      already_registered = true;
      break;
    }
  }

  if (!already_registered)
    listeners_.push_back(listener);
}

void MavlinkComm::unregister_mavlink_listener(MavlinkListenerInterface * const listener)
{
  if (listener == NULL)
    return;

  for (int i = 0; i < listeners_.size(); i++)
  {
    if (listener == listeners_[i])
    {
      listeners_.erase(listeners_.begin() + i);
      i--;
    }
  }
}

void MavlinkComm::send_message(const mavlink_message_t &msg)
{
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  if (comm_)
  {
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    comm_->send_bytes(buffer, len);
  }
}

void MavlinkComm::serial_callback(const uint8_t* buf, size_t len)
{
  mavlink_message_t msg;
  mavlink_status_t status;

  for (size_t i = 0; i < len; i++)
  {
    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
    {
      for (size_t j = 0; j < listeners_.size(); j++)
      {
        listeners_[j]->handle_mavlink_message(msg);
      }
    }
  }
}

} // namespace mavrosflight
