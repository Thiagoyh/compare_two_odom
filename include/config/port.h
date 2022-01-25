

#pragma once

#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <cinttypes>
#include <cmath>
#include <string>


using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;  // 2个字节的无符号整型
using uint32 = uint32_t;
using uint64 = uint64_t;  // 8个字节的无符号整型

namespace common {

// c++11: std::lround 返回最接近x的long int整数 eg: lround(15.2) -> 15, lround(15.8) -> 16
// 返回最接近x的整数
int RoundToInt(const float x);

int RoundToInt(const double x);

int64 RoundToInt64(const float x);

int64 RoundToInt64(const double x);

/**
 * @brief 将字符串进行压缩
 *
 * @param[in] uncompressed 压缩前的string
 * @param[out] compressed 压缩后的string
 */
void FastGzipString(const std::string &uncompressed,
                           std::string *compressed);

/**
 * @brief 将字符串进行解压
 *
 * @param[in] compressed 压缩的string
 * @param[out] decompressed 解压后的string
 */
void FastGunzipString(const std::string &compressed,
                             std::string *decompressed);

}


