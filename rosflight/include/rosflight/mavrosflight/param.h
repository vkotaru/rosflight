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
 * \file param.h
 * \author Daniel Koch <daniel.koch@byu.edu>
 */

#ifndef MAVROSFLIGHT_PARAM_H
#define MAVROSFLIGHT_PARAM_H

#include <rosflight/mavrosflight/mavlink_bridge.h>
#include <rosflight/mavrosflight/mavlink_serial.h>

#include <yaml-cpp/yaml.h>

#include <cstdint>

namespace mavrosflight
{

class Param
{
public:
  Param();
  Param(mavlink_param_value_t msg);
  Param(std::string name, int index, MAV_PARAM_TYPE type, float raw_value);

  uint16_t pack_param_set_msg(uint8_t system, uint8_t component, mavlink_message_t *msg,
                              uint8_t target_system, uint8_t target_component);

  inline std::string name() const { return name_; }
  inline int index() const { return index_; }
  inline MAV_PARAM_TYPE type() const { return type_; }
  inline double value() const { return value_; }

  void requestSet(double value, mavlink_message_t *msg);
  bool handleUpdate(const mavlink_param_value_t &msg);

private:
  void init(std::string name, int index, MAV_PARAM_TYPE type, float raw_value);

  void setFromRawValue(float raw_value);
  float getRawValue();
  float getRawValue(double value);
  double getCastValue(double value);

  union ParamValue
  {
    float raw;

    int8_t int8;
    int16_t int16;
    int32_t int32;

    uint8_t uint8;
    uint16_t uint16;
    uint32_t uint32;

    float real32;
  };

  MavlinkSerial *serial_;

  std::string name_;
  int index_;
  MAV_PARAM_TYPE type_;
  double value_;

  bool set_in_progress_;
  double new_value_;
  float expected_raw_value_;
};

} // namespace mavrosflight

#endif // MAVROSFLIGHT_PARAM_H
