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
 * \file mavlink_comm.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_MAVLINK_COMM_H
#define MAVROSFLIGHT_MAVLINK_COMM_H

#include <rosflight/mavrosflight/mavlink_bridge.h>
#include <rosflight/mavrosflight/mavlink_listener_interface.h>

#define ASYNC_COMM_READ_BUFFER_SIZE MAVLINK_MAX_PACKET_LEN
#define ASYNC_COMM_WRITE_BUFFER_SIZE MAVLINK_MAX_PACKET_LEN
#include <async_comm/comm.h>

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <cstdint>

namespace mavrosflight
{

/**
 * @brief Sends and receives MAVLink messages over a serial or UDP port
 *
 * Provides an API for sending MAVLink messages over the port, and allows the user to register callbacks for when
 * MAVLink messages are received.
 */
class MavlinkComm
{
public:

  MavlinkComm();
  ~MavlinkComm();

  /**
   * @brief Opens a serial port for MAVLink communication
   *
   * If another port (serial or UDP) was previously open, that port will be closed before the specified port is opened.
   *
   * @param port Name of the serial port (e.g. "/dev/ttyUSB0")
   * @param baud_rate The baud rate for the serial port
   * @return True if the port was opened successfully
   */
  bool open_serial(std::string port, unsigned int baud_rate);

  /**
   * @brief Opens a UDP port for MAVLink communication
   *
   * If another port (serial or UDP) was previously open, that port will be closed before the specified port is opened.
   *
   * \param bind_host Host where this node is running
   * \param bind_port Port number for this node
   * \param remote_host Host where the other node is running
   * \param remote_port Port number for the other node
   * @return True if the port was opened successfully
   */
  bool open_udp(std::string bind_host, uint16_t bind_port, std::string remote_host, uint16_t remote_port);

  /**
   * \brief Stops communication and closes the port
   */
  void close();

  /**
   * \brief Register a listener for mavlink messages
   * \param listener Pointer to an object that implements the MavlinkListenerInterface interface
   */
  void register_mavlink_listener(MavlinkListenerInterface * const listener);

  /**
   * \brief Unregister a listener for mavlink messages
   * \param listener Pointer to an object that implements the MavlinkListenerInterface interface
   */
  void unregister_mavlink_listener(MavlinkListenerInterface * const listener);

  /**
   * \brief Send a mavlink message
   * \param msg The message to send
   */
  void send_message(const mavlink_message_t &msg);

private:

  /**
   * @brief Callback for asynchronous serial read
   * @param buf Buffer containing the received data
   * @param len Number of bytes received
   */
  void serial_callback(const uint8_t* buf, size_t len);

  std::unique_ptr<async_comm::Comm> comm_; //!< asynchronous serial communication handler
  std::vector<MavlinkListenerInterface*> listeners_; //!< listeners for mavlink messages

};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_MAVLINK_COMM_H
