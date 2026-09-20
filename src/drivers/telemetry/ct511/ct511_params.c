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
 * @file ct511_params.c
 *
 * CT511 driver parameters.
 */

/**
 * CT511 modem UART
 *
 * Selects the board serial port the CT511 module is wired to.
 *
 * @value 0 TELEM1
 * @value 1 TELEM2
 * @value 2 TELEM3
 * @value 3 GPS2
 *
 * @group CT511 4G
 */
PARAM_DEFINE_INT32(CT511_DEV, 0);

/**
 * CT511 relay server port
 *
 * TCP transparent port of the 1:1 relay server (host is compiled in).
 *
 * @min 1
 * @max 65535
 *
 * @group CT511 4G
 */
PARAM_DEFINE_INT32(CT511_PRT, 4002);

/**
 * CT511 relay auth password
 *
 * Numeric password used to authenticate against the relay server. The wire
 * token is derived as sha256(password || fixed salt), so it must match the
 * password configured on the relay server and in the Android proxy app.
 * Leading zeros are not representable (password is stored as an integer).
 * 0 falls back to the compiled-in default 666666.
 *
 * @min 0
 * @max 2147483647
 *
 * @group CT511 4G
 */
PARAM_DEFINE_INT32(CT511_PW, 666666);
