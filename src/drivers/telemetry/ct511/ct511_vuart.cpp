/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * @file ct511_vuart.cpp
 *
 * Virtual serial device for the CT511 4G MAVLink bridge.
 *
 * Exposes /dev/ct511 to the mavlink module (open/read/write/ioctl/poll).
 * MAVLink frames written by the mavlink module accumulate in the TX ring
 * and are drained by the ct511 driver (uplink to MQTT); MQTT downlink
 * payloads are pushed into the RX ring via rx_push().
 *
 * Termios ioctls are accepted but mostly ignored: the mavlink module
 * validates them, the baudrate is a fiction.
 */

#include "ct511_vuart.h"

#include <px4_platform_common/log.h>

#include <errno.h>

#if defined(__PX4_NUTTX)
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/tioctl.h>
#endif

#include <termios.h>

namespace
{
inline size_t ring_used(size_t head, size_t tail, size_t cap)
{
	return (head - tail) & (cap - 1);
}

inline size_t ring_free(size_t head, size_t tail, size_t cap)
{
	return cap - 1 - ring_used(head, tail, cap);
}
} // namespace

Ct511Vuart::Ct511Vuart() :
	CDev("/dev/ct511")
{
	px4_sem_init(&_lock, 0, 1);
}

int
Ct511Vuart::init()
{
	int ret = CDev::init();

	if (ret != PX4_OK) {
		PX4_ERR("register /dev/ct511 failed: %d", ret);
	}

	return ret;
}

ssize_t
Ct511Vuart::read(cdev::file_t *filep, char *buffer, size_t buflen)
{
	size_t total = 0;

	lock();

	while (total < buflen && _rx_tail != _rx_head) {
		buffer[total++] = _rx_buf[_rx_tail];
		_rx_tail = (_rx_tail + 1) & (RX_CAP - 1);
	}

	unlock();

	return (ssize_t)total;
}

ssize_t
Ct511Vuart::write(cdev::file_t *filep, const char *buffer, size_t buflen)
{
	size_t written = 0;

	lock();

	while (written < buflen && ring_free(_tx_head, _tx_tail, TX_CAP) > 0) {
		_tx_buf[_tx_head] = (uint8_t)buffer[written++];
		_tx_head = (_tx_head + 1) & (TX_CAP - 1);
	}

	unlock();

	return (ssize_t)written;
}

int
Ct511Vuart::ioctl(cdev::file_t *filep, int cmd, unsigned long arg)
{
	switch (cmd) {
	case TCGETS: {
			if (!arg) {
				return -EINVAL;
			}

			struct termios *tio = (struct termios *)arg;

			memset(tio, 0, sizeof(*tio));

			return OK;
		}

	case TCSETS:
	case TCSETSW:
	case TCSETSF:
		/* accept and ignore */
		return OK;

	case TCFLSH:
		lock();
		_rx_head = _rx_tail;
		_tx_head = _tx_tail;
		unlock();
		return OK;

#if defined(__PX4_NUTTX)

	case FIONSPACE: {
			if (!arg) {
				return -EINVAL;
			}

			lock();
			*(int *)arg = (int)ring_free(_tx_head, _tx_tail, TX_CAP);
			unlock();
			return OK;
		}

	case FIONREAD: {
			if (!arg) {
				return -EINVAL;
			}

			lock();
			*(int *)arg = (int)ring_used(_rx_head, _rx_tail, RX_CAP);
			unlock();
			return OK;
		}

#endif

	default:
		return -ENOTTY;
	}
}

px4_pollevent_t
Ct511Vuart::poll_state(cdev::file_t *filep)
{
	px4_pollevent_t events = 0;

	if (_rx_tail != _rx_head) {
		events |= POLLIN;
	}

	if (ring_free(_tx_head, _tx_tail, TX_CAP) > 0) {
		events |= POLLOUT;
	}

	return events;
}

int
Ct511Vuart::rx_push(const uint8_t *data, size_t len)
{
	size_t pushed = 0;

	if (len == 0 || data == nullptr) {
		return 0;
	}

	lock();

	while (pushed < len && ring_free(_rx_head, _rx_tail, RX_CAP) > 0) {
		_rx_buf[_rx_head] = data[pushed++];
		_rx_head = (_rx_head + 1) & (RX_CAP - 1);
	}

	unlock();

	if (pushed > 0) {
		poll_notify(POLLIN);
	}

	return (int)pushed;
}

size_t
Ct511Vuart::tx_pull(uint8_t *data, size_t len)
{
	size_t pulled = 0;

	lock();

	while (pulled < len && _tx_tail != _tx_head) {
		data[pulled++] = _tx_buf[_tx_tail];
		_tx_tail = (_tx_tail + 1) & (TX_CAP - 1);
	}

	unlock();

	return pulled;
}
