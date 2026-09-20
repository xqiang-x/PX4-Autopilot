/**
 * @file ct511_sha256.h
 *
 * Minimal SHA-256 (FIPS 180-4) used only for the ct511 auth-token
 * derivation: token = first 48 hex chars of SHA-256(password || salt).
 * The identical implementation/derivation lives in the relay server
 * (relay.c) and the Android proxy app; keep AUTH_SALT in sync.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

typedef struct {
	uint32_t h[8];
	uint64_t len;
	uint8_t buf[64];
	size_t idx;
} sha256_ctx;

static const uint32_t g_sha256_k[64] = {
	0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u,
	0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u,
	0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
	0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u, 0x06ca6351u, 0x14292967u,
	0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
	0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
	0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu, 0x682e6ff3u,
	0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u, 0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u,
};

static void sha256_block(sha256_ctx *c, const uint8_t *p)
{
	uint32_t w[64];
	int i;

	for (i = 0; i < 16; i++) {
		w[i] = ((uint32_t)p[i * 4] << 24) | ((uint32_t)p[i * 4 + 1] << 16)
		       | ((uint32_t)p[i * 4 + 2] << 8) | (uint32_t)p[i * 4 + 3];
	}

	for (i = 16; i < 64; i++) {
		const uint32_t s0 = (w[i - 15] >> 7 | w[i - 15] << 25)
				    ^ (w[i - 15] >> 18 | w[i - 15] << 14) ^ (w[i - 15] >> 3);
		const uint32_t s1 = (w[i - 2] >> 17 | w[i - 2] << 15)
				    ^ (w[i - 2] >> 19 | w[i - 2] << 13) ^ (w[i - 2] >> 10);
		w[i] = w[i - 16] + s0 + w[i - 7] + s1;
	}

	uint32_t a = c->h[0], b = c->h[1], cc = c->h[2], d = c->h[3];
	uint32_t e = c->h[4], f = c->h[5], g = c->h[6], h = c->h[7];

	for (i = 0; i < 64; i++) {
		const uint32_t s1 = (e >> 6 | e << 26) ^ (e >> 11 | e << 21) ^ (e >> 25 | e << 7);
		const uint32_t ch = (e & f) ^ (~e & g);
		const uint32_t t1 = h + s1 + ch + g_sha256_k[i] + w[i];
		const uint32_t s0 = (a >> 2 | a << 30) ^ (a >> 13 | a << 19) ^ (a >> 22 | a << 10);
		const uint32_t maj = (a & b) ^ (a & cc) ^ (b & cc);
		const uint32_t t2 = s0 + maj;

		h = g; g = f; f = e; e = d + t1;
		d = cc; cc = b; b = a; a = t1 + t2;
	}

	c->h[0] += a; c->h[1] += b; c->h[2] += cc; c->h[3] += d;
	c->h[4] += e; c->h[5] += f; c->h[6] += g; c->h[7] += h;
}

static void sha256_init(sha256_ctx *c)
{
	static const uint32_t iv[8] = {
		0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
		0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u,
	};

	memcpy(c->h, iv, sizeof(iv));
	c->len = 0;
	c->idx = 0;
}

static void sha256_update(sha256_ctx *c, const void *data, size_t len)
{
	const uint8_t *p = static_cast<const uint8_t *>(data);

	c->len += len;

	while (len > 0) {
		size_t take = 64 - c->idx;

		if (take > len) {
			take = len;
		}

		memcpy(c->buf + c->idx, p, take);
		c->idx += take;
		p += take;
		len -= take;

		if (c->idx == 64) {
			sha256_block(c, c->buf);
			c->idx = 0;
		}
	}
}

static void sha256_final(sha256_ctx *c, uint8_t out[32])
{
	const uint64_t bits = c->len * 8;
	uint8_t pad = 0x80;
	uint8_t zero = 0;
	uint8_t lenb[8];
	int i;

	sha256_update(c, &pad, 1);

	while (c->idx != 56) {
		sha256_update(c, &zero, 1);
	}

	for (i = 0; i < 8; i++) {
		lenb[i] = (uint8_t)(bits >> (56 - 8 * i));
	}

	sha256_update(c, lenb, 8);

	for (i = 0; i < 8; i++) {
		out[i * 4]     = (uint8_t)(c->h[i] >> 24);
		out[i * 4 + 1] = (uint8_t)(c->h[i] >> 16);
		out[i * 4 + 2] = (uint8_t)(c->h[i] >> 8);
		out[i * 4 + 3] = (uint8_t)(c->h[i]);
	}
}

static void derive_auth_token(char out[49], const char *password, const char *salt)
{
	static const char hexd[] = "0123456789abcdef";
	sha256_ctx c;
	uint8_t d[32];
	int i;

	sha256_init(&c);
	sha256_update(&c, password, strlen(password));
	sha256_update(&c, salt, strlen(salt));
	sha256_final(&c, d);

	for (i = 0; i < 24; i++) {
		out[i * 2] = hexd[d[i] >> 4];
		out[i * 2 + 1] = hexd[d[i] & 0x0f];
	}

	out[48] = 0;
}
