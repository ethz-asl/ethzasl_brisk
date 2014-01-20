/*
 Copyright (C) 2011 The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

 Copyright(C) 2013 The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 BRISK - Binary Robust Invariant Scalable Keypoints
 Reference implementation of
 [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision(ICCV2011).

 This file is part of BRISK.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INTERNAL_HAMMING_INL_H_
#define INTERNAL_HAMMING_INL_H_

#include <brisk/internal/neon-helpers.h>

namespace brisk {

#ifdef __GNUC__
static const char __attribute__((aligned(16))) MASK_4bit[16] = {0xf, 0xf, 0xf,
  0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf};
static const uint8_t __attribute__((aligned(16))) POPCOUNT_4bit[16] = {0, 1, 1,
  2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
#ifdef __ARM_NEON__
uint8_t tmpmask[16] = {0x4, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
    0x0, 0x0, 0x0, 0x0, 0x0};
static const uint8x16_t shiftval = vld1q_u8(&tmpmask[0]);
#else
static const __m128i shiftval = _mm_set_epi32(0, 0, 0, 4);
#endif
#endif
#ifdef _MSC_VER
__declspec(align(16)) static const char MASK_4bit[16] = {0xf, 0xf, 0xf, 0xf,
  0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf};
__declspec(align(16)) static const uint8_t POPCOUNT_4bit[16] = {0, 1, 1, 2,
  1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
static const __m128i shiftval = _mm_set_epi32(0, 0, 0, 4);
#endif

#ifdef __ARM_NEON__
__inline__ uint32_t Hamming::NEONPopcntofXORed(const uint8x16_t* signature1,
                                               const uint8x16_t*signature2,
                                               const int numberOf128BitWords) {
  uint32_t result = 0;

  register uint8x16_t xmm0;
  register uint8x16_t xmm1;
  register uint8x16_t xmm2;
  register uint8x16_t xmm3;
  register uint8x16_t xmm4;
  register uint8x16_t xmm5;
  register uint8x16_t xmm6;
  register uint8x16_t xmm7;

//  xmm7 = _mm_load_si128(reinterpret_cast<const __m128i*>(POPCOUNT_4bit));
  xmm7 = vld1q_u8(reinterpret_cast<const uint8_t*>(POPCOUNT_4bit));
//  xmm6 = _mm_load_si128(reinterpret_cast<const __m128i*>(MASK_4bit));
  xmm6 = vld1q_u8(reinterpret_cast<const uint8_t*>(MASK_4bit));
//  xmm5 = _mm_setzero_si128();  // xmm5 -- global accumulator.
  xmm5 = vdupq_n_u8(0);  // xmm5 -- global accumulator.

  const uint8x16_t* end = signature1 + numberOf128BitWords;
  xmm4 = xmm5;  // xmm4 -- local accumulator.

  do {
//    xmm0 = _mm_xor_si128(*signature1++, *signature2++);
    xmm0 = veorq_u8(*signature1++, *signature2++);
    xmm1 = xmm0;
//    xmm1 = _mm_srl_epi16(xmm1, shiftval);
    xmm1 = vsriq_n_u8(xmm1, shiftval, 0);
//    xmm0 = _mm_and_si128(xmm0, xmm6);  // xmm0 := lower nibbles.
    xmm0 = vandq_s8(xmm0, xmm6);  // xmm0 := lower nibbles.
//    xmm1 = _mm_and_si128(xmm1, xmm6);  // xmm1 := higher nibbles.
    xmm1 = vandq_s8(xmm1, xmm6);  // xmm1 := higher nibbles.
    xmm2 = xmm7;
    xmm3 = xmm7;  // Get popcount.
//    xmm2 = _mm_shuffle_epi8(xmm2, xmm0);  // For all nibbles.
    xmm2 = brisk::shuffle_epi8_neon(xmm2, xmm0);// For all nibbles.
//    xmm3 = _mm_shuffle_epi8(xmm3, xmm1);  // Using PSHUFB.
    xmm3 = brisk::shuffle_epi8_neon(xmm3, xmm1);  // Using PSHUFB.
//    xmm4 = _mm_add_epi8(xmm4, xmm2);  // Update local.
    xmm4 = vaddq_s8(xmm4, xmm2);  // Update local.
//    xmm4 = _mm_add_epi8(xmm4, xmm3);  // Accumulator.
    xmm4 = vaddq_s8(xmm4, xmm3);  // Accumulator.
  }while (signature1 < end);
  // Update global accumulator(two 32-bits counters).
//  xmm4 = _mm_sad_epu8(xmm4, xmm5);
  xmm4 = vabdq_s8(xmm4, xmm5);
//  xmm5 = _mm_add_epi32(xmm5, xmm4);
  xmm5 = vaddq_s32(xmm5, xmm4);
  // finally add together 32-bits counters stored in global accumulator.
  // __asm__ volatile(
  //  "movhlps  %%xmm5, %%xmm0 \n"
  //  xmm0 = _mm_cvtps_epi32(_mm_movehl_ps(_mm_cvtepi32_ps(xmm0),
  //      _mm_cvtepi32_ps(xmm5)));
  int64x1_t upper_xmm5 = vget_high_u64(xmm5);
  int64x1_t upper_xmm0 = vget_high_u64(xmm0);
  xmm0 = vcombine_u64(upper_xmm0, upper_xmm5);
//  xmm0 = _mm_add_epi32(xmm0, xmm5);
  xmm0 = vaddq_s32(xmm0, xmm5);
//  result = _mm_cvtsi128_si32(xmm0);
  result = vget_lane_s32(xmm0, 0);
  return result;
}
#else
// - SSSE3 - better alorithm, minimized psadbw usage -
// adapted from http://wm.ite.pl/articles/sse-popcount.html
__inline__ uint32_t Hamming::SSSE3PopcntofXORed(const __m128i* signature1,
const __m128i*signature2,
const int numberOf128BitWords) {
  uint32_t result = 0;

  register __m128i xmm0;
  register __m128i xmm1;
  register __m128i xmm2;
  register __m128i xmm3;
  register __m128i xmm4;
  register __m128i xmm5;
  register __m128i xmm6;
  register __m128i xmm7;

  xmm7 = _mm_load_si128(reinterpret_cast<const __m128i*>(POPCOUNT_4bit));
  xmm6 = _mm_load_si128(reinterpret_cast<const __m128i*>(MASK_4bit));
  xmm5 = _mm_setzero_si128();  // xmm5 -- global accumulator.

  const __m128i* end = signature1 + numberOf128BitWords;
  xmm4 = xmm5;  // xmm4 -- local accumulator.

  do {
    xmm0 = _mm_xor_si128(*signature1++, *signature2++);
    xmm1 = xmm0;
    xmm1 = _mm_srl_epi16(xmm1, shiftval);
    xmm0 = _mm_and_si128(xmm0, xmm6);  // xmm0 := lower nibbles.
    xmm1 = _mm_and_si128(xmm1, xmm6);  // xmm1 := higher nibbles.
    xmm2 = xmm7;
    xmm3 = xmm7;  // Get popcount.
    xmm2 = _mm_shuffle_epi8(xmm2, xmm0);  // For all nibbles.
    xmm3 = _mm_shuffle_epi8(xmm3, xmm1);  // Using PSHUFB.
    xmm4 = _mm_add_epi8(xmm4, xmm2);  // Update local.
    xmm4 = _mm_add_epi8(xmm4, xmm3);  // Accumulator.
  }while (signature1 < end);
  // Update global accumulator(two 32-bits counters).
  xmm4 = _mm_sad_epu8(xmm4, xmm5);
  xmm5 = _mm_add_epi32(xmm5, xmm4);
  // finally add together 32-bits counters stored in global accumulator.
  // __asm__ volatile(
  //  "movhlps  %%xmm5, %%xmm0 \n"
  // TODO(slynen): fix with appropriate intrinsic
  xmm0 = _mm_cvtps_epi32(_mm_movehl_ps(_mm_cvtepi32_ps(xmm0),
      _mm_cvtepi32_ps(xmm5)));
  xmm0 = _mm_add_epi32(xmm0, xmm5);
  result = _mm_cvtsi128_si32(xmm0);
  return result;
}
#endif  // __ARM_NEON__

}  // namespace brisk
#endif  // INTERNAL_HAMMING_INL_H_
