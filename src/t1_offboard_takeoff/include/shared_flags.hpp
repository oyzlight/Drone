// shared_flags.h
#ifndef SHARED_FLAGS_H  // 防止头文件重复包含
#define SHARED_FLAGS_H

#include <cstdint>  // 包含uint8_t定义

// 声明全局标志位（仅声明，不定义）
extern uint8_t g_shared_flag;  // g_前缀表示全局变量，避免命名冲突

#endif  // SHARED_FLAGS_H
