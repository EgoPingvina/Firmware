/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file mavlink_orb_subscription.cpp
 * uORB subscription implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "mavlink_orb_subscription.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include <px4_defines.h>
#include <uORB/uORB.h>

MavlinkOrbSubscription::MavlinkOrbSubscription(const orb_id_t topic, int instance) :
	_topic(topic),
	_instance(instance)
{
}

MavlinkOrbSubscription::~MavlinkOrbSubscription()
{
	if (_fd >= 0) {
		orb_unsubscribe(_fd);
	}
}

orb_id_t
MavlinkOrbSubscription::get_topic() const
{
	return _topic;
}

int
MavlinkOrbSubscription::get_instance() const
{
	return _instance;
}

bool
MavlinkOrbSubscription::updated(const uint64_t &time)
{
	if (!is_published()) {
		return false;
	}

	uint64_t time_topic = 0;

	if (orb_stat(_fd, &time_topic) != PX4_OK) {
		/* error getting last topic publication time */
		return false;
	}

	if (time_topic > time) {
		return true;
	}

	return false;
}

bool
MavlinkOrbSubscription::update(uint64_t *time, void *data)
{
	// TODO this is NOT atomic operation, we can get data newer than time
	// if topic was published between orb_stat and orb_copy calls.

	if (updated(*time)) {
		if (orb_copy(_topic, _fd, data) == PX4_OK) {
			/* data copied successfully */
			orb_stat(_fd, time);

			return true;
		}
	}

	return false;
}

bool
MavlinkOrbSubscription::update(void *data)
{
	if (!is_published()) {
		return false;
	}

	if (orb_copy(_topic, _fd, data) != PX4_OK) {
		return false;
	}

	return true;
}

bool
MavlinkOrbSubscription::update_if_changed(void *data)
{
	if (!is_published()) {
		return false;
	}

	bool updated;

	if (orb_check(_fd, &updated) || !updated) {
		return false;
	}

	return update(data);
}

bool
MavlinkOrbSubscription::is_published()
{
	// If we marked it as published no need to check again
	if (_published) {
		return true;
	}

	hrt_abstime now = hrt_absolute_time();

	if (now - _last_pub_check < 300000) {
		return false;
	}

	// We are checking now
	_last_pub_check = now;

	// We don't want to subscribe to anything that does not exist
	// in order to save memory and file descriptors.
	// However, for some topics like vehicle_command_ack, we want to subscribe
	// from the beginning in order not to miss or delay the first publish respective advertise.
	if (!_subscribe_from_beginning && orb_exists(_topic, _instance)) {
		return false;
	}

	if (_fd < 0) {
		_fd = orb_subscribe_multi(_topic, _instance);
	}

	bool updated;
	orb_check(_fd, &updated);

	if (updated) {
		_published = true;
	}

	return _published;
}

void
MavlinkOrbSubscription::subscribe_from_beginning(bool from_beginning)
{
	_subscribe_from_beginning = from_beginning;
}
