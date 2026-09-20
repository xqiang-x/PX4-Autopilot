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
 * @file ct511_vuart.h
 *
 * Virtual serial device bridged to the CT511 4G modem: the mavlink module
 * attaches to /dev/ct511 like a normal UART, the ct511 driver moves the
 * bytes in/out of the modem's MQTT link.
 */

#pragma once

#include <lib/cdev/CDev.hpp>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/sem.h>

class Ct511Vuart : public cdev::CDev
{
public:
	Ct511Vuart();
	~Ct511Vuart() override = default;

	int init() override;

	ssize_t read(cdev::file_t *filep, char *buffer, size_t buflen) override;
	ssize_t write(cdev::file_t *filep, const char *buffer, size_t buflen) override;
	int ioctl(cdev::file_t *filep, int cmd, unsigned long arg) override;
	px4_pollevent_t poll_state(cdev::file_t *filep) override;

	/* bridge API, called from the ct511 driver thread */
	int rx_push(const uint8_t *data, size_t len);  /* downlink (MQTT -> mavlink) */
	size_t tx_pull(uint8_t *data, size_t len);     /* uplink (mavlink -> MQTT) */

private:
	void lock() { do {} while (px4_sem_wait(&_lock) != 0); }
	void unlock() { px4_sem_post(&_lock); }

	static constexpr size_t RX_CAP = 4096;
	static constexpr size_t TX_CAP = 8192;

	px4_sem_t _lock;

	uint8_t _rx_buf[RX_CAP];
	size_t _rx_head{0};
	size_t _rx_tail{0};

	uint8_t _tx_buf[TX_CAP];
	size_t _tx_head{0};
	size_t _tx_tail{0};
};
