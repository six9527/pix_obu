#pragma once

/*
1.0 结构体和字节流之间的序列化和反序列化
2.0 结构体组成：基础类型，string, std::vector<uint8_t>
*/
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <type_traits>
#include <vector>

template <typename T> void show(const T &byteStream) {
  // 输出存储结果
  for (uint8_t byte : byteStream) {
    std::cout << std::hex << std::setfill('0') << std::setw(2)
              << static_cast<int>(byte) << ' ';
  }
  std::cout << '\n';
}

// 判断主机字节序
constexpr bool isLittleEndian() {
  uint16_t num = 1;
  return *(reinterpret_cast<uint8_t *>(&num)) == 1;
}

// 定义模板结构体来检查类型是否是 std::array<uint8_t, N> 的特化
template <typename T> struct is_std_array_of_uint8 : std::false_type {};

// 对模板进行特化，当类型是 std::array<uint8_t, N> 时返回 true
template <std::size_t N>
struct is_std_array_of_uint8<std::array<uint8_t, N>> : std::true_type {};

template <typename T>
bool storeToByteStreamBigEndian(const T &data,
                                std::vector<uint8_t> &byteStream) {
  // data 类型是否是 arrry<uint8_t, N> std::string std::vector<uint8_t>
  if constexpr (std::is_same_v<T, std::string> ||
                std::is_same_v<T, std::vector<uint8_t>> ||
                is_std_array_of_uint8<T>::value) {
    byteStream.insert(byteStream.end(), data.begin(), data.end());
    return true;
  }

  // 基础数据类型
  if constexpr (std::is_fundamental<T>::value) {
    const uint8_t *bytePtr = reinterpret_cast<const uint8_t *>(&data);
    if (isLittleEndian()) {
      for (size_t i = 0; i < sizeof(T); ++i) {
        size_t index = sizeof(T) - 1 - i;
        byteStream.push_back(bytePtr[index]);
      }
    } else {
      byteStream.insert(byteStream.end(), bytePtr, bytePtr + sizeof(T));
    }
    return true;
  } else {
    std::cerr << "[storeToByteStreamBigEndian]不是基础类型, "
                 "std::vector<uint8_t>, string, array<uint8_t, N>:"
              << typeid(T).name()   
              << std::endl;
    return false;
  }

  return false;
}

template <typename T>
bool storeFromByteStreamBigEndian(T &data,
                                  const std::vector<uint8_t> &byteStream) {

  if constexpr (std::is_same_v<T, std::string> ||
                std::is_same_v<T, std::vector<uint8_t>>) {

    data.resize(byteStream.size());
    std::copy(byteStream.begin(), byteStream.end(), data.begin());
    return true;
  }
  // data 类型是否是 arrry<uint8_t, N>
  if constexpr (is_std_array_of_uint8<T>::value) {
    std::copy(byteStream.begin(), byteStream.end(), data.begin());
    return true;
  }

  // 基础数据类型
  if constexpr (std::is_fundamental<T>::value) {
    if (byteStream.size() != sizeof(T)) {
      std::cout
          << "Size of byte stream does not match the size of the target type"
          << '\n';
      return false;
    }
    if (isLittleEndian()) {
      std::vector<uint8_t> reversedStream(byteStream.rbegin(),
                                          byteStream.rend());
      std::memcpy(&data, reversedStream.data(), sizeof(T));

    } else {
      std::memcpy(&data, byteStream.data(), sizeof(T));
    }
    return true;
  } else {
    std::cerr << "[storeFromByteStreamBigEndian]不是基础类型, "
                 "std::vector<uint8_t>, string, array<uint8_t, N>:"
              << typeid(T).name()
              << std::endl;
    return false;
  }
  return false;
}

template <typename T> uint16_t getStoreByteSize(T &data) {
  if constexpr (std::is_same_v<T, std::string>) {
    return 0;
  }
  
  if constexpr (std::is_same_v<T, std::vector<uint8_t>> ||
                is_std_array_of_uint8<T>::value) {
    return data.size();
  } else if constexpr (std::is_fundamental<T>::value) {
    return sizeof(data);
  } else {
    std::cerr << "[getStoreByteSize]不是基础类型, std::vector<uint8_t>, "
                 "string, array<uint8_t, N>:"
              << typeid(T).name()
              << std::endl;
    return 0xffff;
  }
  return 0xffff;
}

// template <typename T>
// bool is_size_illegal(const T &struct_data,
//     const std::vector<uint8_t> &byteStream) {
//   if constexpr (std::is_same_v<T, std::string> ||
//                 std::is_same_v<T, std::vector<uint8_t>> ||
//                 std::is_fundamental<T>::value) {
//     return data.size();
//   } else if (std::is_fundamental<T>::value) {
//     return sizeof(data);
//   } else {
//     std::cerr << "不是基础类型, std::vector<uint8_t>, string, array<uint8_t,
//     N>" << std::endl;
//   }
// }