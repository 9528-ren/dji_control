#include "mop/Md5.h"

#include <unistd.h>

#include <cstdio>
#include <cstring>

namespace maicro {
namespace upgrade {
namespace {

struct Md5Ctx {
  uint32_t a = 0x67452301u;
  uint32_t b = 0xefcdab89u;
  uint32_t c = 0x98badcfeu;
  uint32_t d = 0x10325476u;
  uint64_t total = 0;
  unsigned char buf[64];
  size_t used = 0;
};

const uint32_t K[64] = {
    0xd76aa478u, 0xe8c7b756u, 0x242070dbu, 0xc1bdceeeu, 0xf57c0fafu,
    0x4787c62au, 0xa8304613u, 0xfd469501u, 0x698098d8u, 0x8b44f7afu,
    0xffff5bb1u, 0x895cd7beu, 0x6b901122u, 0xfd987193u, 0xa679438eu,
    0x49b40821u, 0xf61e2562u, 0xc040b340u, 0x265e5a51u, 0xe9b6c7aau,
    0xd62f105du, 0x02441453u, 0xd8a1e681u, 0xe7d3fbc8u, 0x21e1cde6u,
    0xc33707d6u, 0xf4d50d87u, 0x455a14edu, 0xa9e3e905u, 0xfcefa3f8u,
    0x676f02d9u, 0x8d2a4c8au, 0xfffa3942u, 0x8771f681u, 0x6d9d6122u,
    0xfde5380cu, 0xa4beea44u, 0x4bdecfa9u, 0xf6bb4b60u, 0xbebfbc70u,
    0x289b7ec6u, 0xeaa127fau, 0xd4ef3085u, 0x04881d05u, 0xd9d4d039u,
    0xe6db99e5u, 0x1fa27cf8u, 0xc4ac5665u, 0xf4292244u, 0x432aff97u,
    0xab9423a7u, 0xfc93a039u, 0x655b59c3u, 0x8f0ccc92u, 0xffeff47du,
    0x85845dd1u, 0x6fa87e4fu, 0xfe2ce6e0u, 0xa3014314u, 0x4e0811a1u,
    0xf7537e82u, 0xbd3af235u, 0x2ad7d2bbu, 0xeb86d391u};

const uint32_t S[64] = {7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17, 22, 7, 12, 17,
                        22, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,  14, 20, 5, 9,
                        14, 20, 4, 11, 16, 23, 4, 11, 16, 23, 4, 11, 16, 23, 4,
                        11, 16, 23, 6, 10, 15, 21, 6, 10, 15, 21, 6, 10, 15, 21,
                        6, 10, 15, 21};

inline uint32_t Rotl(uint32_t x, uint32_t c) {
  return (x << c) | (x >> (32 - c));
}

void Transform(Md5Ctx *ctx, const unsigned char block[64]) {
  uint32_t m[16];
  for (int i = 0; i < 16; ++i) {
    m[i] = (uint32_t)block[i * 4] | ((uint32_t)block[i * 4 + 1] << 8) |
           ((uint32_t)block[i * 4 + 2] << 16) |
           ((uint32_t)block[i * 4 + 3] << 24);
  }
  uint32_t a = ctx->a, b = ctx->b, c = ctx->c, d = ctx->d;
  for (uint32_t i = 0; i < 64; ++i) {
    uint32_t f;
    uint32_t g;
    if (i < 16) {
      f = (b & c) | (~b & d);
      g = i;
    } else if (i < 32) {
      f = (d & b) | (~d & c);
      g = (5 * i + 1) % 16;
    } else if (i < 48) {
      f = b ^ c ^ d;
      g = (3 * i + 5) % 16;
    } else {
      f = c ^ (b | ~d);
      g = (7 * i) % 16;
    }
    uint32_t tmp = d;
    d = c;
    c = b;
    b = b + Rotl(a + f + K[i] + m[g], S[i]);
    a = tmp;
  }
  ctx->a += a;
  ctx->b += b;
  ctx->c += c;
  ctx->d += d;
}

void Update(Md5Ctx *ctx, const void *data, size_t len) {
  const unsigned char *p = (const unsigned char *)data;
  ctx->total += len;
  while (len > 0) {
    size_t n = 64 - ctx->used;
    if (n > len)
      n = len;
    memcpy(ctx->buf + ctx->used, p, n);
    ctx->used += n;
    p += n;
    len -= n;
    if (ctx->used == 64) {
      Transform(ctx, ctx->buf);
      ctx->used = 0;
    }
  }
}

std::string Final(Md5Ctx *ctx) {
  uint64_t bits = ctx->total * 8;
  unsigned char pad = 0x80;
  Update(ctx, &pad, 1);
  unsigned char zero = 0;
  while (ctx->used != 56)
    Update(ctx, &zero, 1);
  unsigned char lenBytes[8];
  for (int i = 0; i < 8; ++i)
    lenBytes[i] = (unsigned char)((bits >> (8 * i)) & 0xFF);
  Update(ctx, lenBytes, 8);

  unsigned char digest[16];
  const uint32_t v[4] = {ctx->a, ctx->b, ctx->c, ctx->d};
  for (int i = 0; i < 4; ++i) {
    digest[i * 4] = (unsigned char)(v[i] & 0xFF);
    digest[i * 4 + 1] = (unsigned char)((v[i] >> 8) & 0xFF);
    digest[i * 4 + 2] = (unsigned char)((v[i] >> 16) & 0xFF);
    digest[i * 4 + 3] = (unsigned char)((v[i] >> 24) & 0xFF);
  }
  char hex[33];
  for (int i = 0; i < 16; ++i)
    snprintf(hex + i * 2, 3, "%02x", digest[i]);
  return std::string(hex, 32);
}

} // namespace

std::string Md5Hex(const void *data, size_t len) {
  Md5Ctx ctx;
  Update(&ctx, data, len);
  return Final(&ctx);
}

bool Md5File(const std::string &path, std::string *out) {
  FILE *fp = fopen(path.c_str(), "rb");
  if (fp == nullptr)
    return false;
  Md5Ctx ctx;
  unsigned char buf[64 * 1024];
  size_t n;
  while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
    Update(&ctx, buf, n);
  }
  bool ok = (ferror(fp) == 0);
  fclose(fp);
  if (!ok)
    return false;
  if (out != nullptr)
    *out = Final(&ctx);
  return true;
}

std::string SelfExePath() {
  char buf[4096];
  ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
  if (n <= 0)
    return std::string();
  buf[n] = '\0';
  return std::string(buf);
}

} // namespace upgrade
} // namespace maicro
