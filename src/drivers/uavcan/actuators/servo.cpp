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
 * @file servo.cpp
 *
 * @author Biryukov Aleksey <abiryukov1996@gmail.com>
 */

#include "servo.hpp"
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

UavcanServoController::UavcanServoController(uavcan::INode &node) :
	_node(node),
	arrayCommandPublisher(node),
	commandPublisher(node)
{
	this->arrayCommandPublisher.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);
	this->commandPublisher.setPriority(UAVCAN_COMMAND_TRANSFER_PRIORITY);

	if (_perfcnt_invalid_input == nullptr)
		errx(1, "uavcan: couldn't allocate _perfcnt_invalid_input");
    else if (_perfcnt_scaling_error == nullptr)
		errx(1, "uavcan: couldn't allocate _perfcnt_scaling_error");
}

UavcanServoController::~UavcanServoController()
{
	perf_free(_perfcnt_invalid_input);
	perf_free(_perfcnt_scaling_error);
}

int UavcanServoController::Init()
{
	auto isOk = 0;

	/*
     * We defined two data types, but only one of them has a default Data Type ID (DTID):
     *  - sirius_cybernetics_corporation.GetCurrentTime               - default DTID 242
     *  - sirius_cybernetics_corporation.PerformLinearLeastSquaresFit - default DTID is not set
     * The first one can be used as is; the second one needs to be registered first.
     */
    isOk =
        uavcan::GlobalDataTypeRegistry::instance().registerDataType<uavcan::equipment::actuator::Command>(1012); // DTID = 1012

	/*
	 * Possible reasons for a failure:
	 * - Data type name or ID is not unique
	 * - Data Type Registry has been frozen and can't be modified anymore
	 */
    if (isOk != uavcan::GlobalDataTypeRegistry::RegistrationResultOk)
        errx(1, "Failed to register the data type: " + isOk);

	return isOk;
}

void UavcanServoController::UpdateOutputs(float *outputs, unsigned num_outputs)
{
	if ((outputs == nullptr)
        || (num_outputs > uavcan::equipment::actuator::ArrayCommand::FieldTypes::commands::MaxSize))
    {
		perf_count(_perfcnt_invalid_input);
		return;
	}

	/*
	 * Rate limiting - we don't want to congest the bus
	 */
	const auto timestamp = this->_node.getMonotonicTime();

	if ((timestamp - this->previousPWMPublication).toUSec() < (1000000 / MAX_RATE_HZ))
		return;

	this->previousPWMPublication = timestamp;

	/*
	 * Fill the command message
	 */
	uavcan::equipment::actuator::ArrayCommand message;

	static const float cmd_max  = 2000.0F;
	static const float cmd_min  = 1000.0F;

    for (unsigned i = 0; i < num_outputs; i++)
    {
        // trim negative values back to minimum
        if (outputs[i] < cmd_min)
        {
            outputs[i] = cmd_min;
            perf_count(_perfcnt_scaling_error);
        }

        if (outputs[i] > cmd_max)
        {
            outputs[i] = cmd_max;
            perf_count(_perfcnt_scaling_error);
        }

		uavcan::equipment::actuator::Command data;
		data.actuator_id     = i;
		data.command_value   = static_cast<int>(outputs[i]);
		data.command_type 	 = (uint8_t)Commands::PWM;
		message.commands.push_back(data);
    }

	/*
	 * Publish the command message to the bus
	 * Note that for a servo it takes one CAN frame
	 */
	(void)this->arrayCommandPublisher.broadcast(message);
}

void UavcanServoController::UpdateIgnition(bool isWork)
{
	/*
	* Rate limiting - we don't want to congest the bus
	*/
	const auto timestamp = this->_node.getMonotonicTime();

	if ((timestamp - this->previousIgnitionPublication).toUSec() < (1000000 / MAX_RATE_HZ))
		return;

	this->previousIgnitionPublication = timestamp;

	uavcan::equipment::actuator::Command message;

	message.actuator_id     = -1;
	message.command_value   = isWork;
	message.command_type	= (uint8_t)Commands::Ignition;

	(void)this->commandPublisher.broadcast(message);
}