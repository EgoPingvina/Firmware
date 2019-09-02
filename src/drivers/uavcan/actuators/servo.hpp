/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file servo.hpp
 *
 * @author Biryukov Aleksey <abiryukov1996@gmail.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/actuator/ArrayCommand.hpp>
#include <uavcan/equipment/actuator/Command.hpp>
#include <perf/perf_counter.h>

class UavcanServoController
{
public:
	enum class Commands : uint8_t
	{
		PWM 		= 1,
		Ignition 	= 2
	};

	UavcanServoController(uavcan::INode &node);
	~UavcanServoController();

	int Init();

	void UpdateOutputs(float *outputs, unsigned num_outputs);

	void UpdateIgnition(bool isWork);

private:
	static constexpr unsigned MAX_RATE_HZ                       = 100;	///< XXX make this configurable
	static constexpr unsigned UAVCAN_COMMAND_TRANSFER_PRIORITY  = 5;	///< 0..31, inclusive, 0 - highest, 31 - lowest

	orb_advert_t actuatorOutputsPub                             = nullptr;

	/*
	 * libuavcan related things
	 */
	uavcan::INode								                    &_node;
	uavcan::Publisher<uavcan::equipment::actuator::ArrayCommand> 	arrayCommandPublisher;
	uavcan::Publisher<uavcan::equipment::actuator::Command>       	commandPublisher;
	uavcan::MonotonicTime							                previousPWMPublication;   		///< rate limiting
	uavcan::MonotonicTime							                previousIgnitionPublication;   	///< rate limiting

	/*
	 * Perf counters
	 */
	perf_counter_t _perfcnt_invalid_input = perf_alloc(PC_COUNT, "uavcan_esc_invalid_input");
	perf_counter_t _perfcnt_scaling_error = perf_alloc(PC_COUNT, "uavcan_esc_scaling_error");
};