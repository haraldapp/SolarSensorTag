
// file: sst_em_device.h
//
// em_device like includes for SST supported mcu chips
//
// includes all headers with path relative to gecko sdk base
//
#ifndef EM_DEVICE_H
#define EM_DEVICE_H

// em_device.h like includes stripped
// EFM32 mcu's with QFN32 p0,65 6x6mm package and at least 32kb flash (64kb is recommended)

// gecko
#define EM_DEVICE_H
#if defined(EFM32G200F32)
#include "platform\Device\SiliconLabs\EFM32G\Include\efm32g200f32.h"

#elif defined(EFM32G200F64)
#include "platform\Device\SiliconLabs\EFM32G\Include\efm32g200f64.h"

#elif defined(EFM32G200F128)
#include "platform\Device\SiliconLabs\EFM32G\Include\efm32g200f128.h"

#elif defined(EFM32G210F128)
#include "platform\Device\SiliconLabs\EFM32G\Include\efm32g210f128.h"

// happy gecko
#elif defined(EFM32HG210F32)
#define ARM_MATH_CM0PLUS
#include "platform\Device\SiliconLabs\EFM32HG\Include\efm32hg210f32.h"

#elif defined(EFM32HG210F64)
#include "platform\Device\SiliconLabs\EFM32HG\Include\efm32hg210f64.h"

#elif defined(EFM32HG310F32)
#include "platform\Device\SiliconLabs\EFM32HG\Include\efm32hg310f32.h"

#elif defined(EFM32HG310F64)
#include "platform\Device\SiliconLabs\EFM32HG\Include\efm32hg310f64.h"

// Tiny Gecko
#elif defined(EFM32TG210F32)
#include "platform\Device\SiliconLabs\EFM32TG\Include\efm32tg210f32.h"

// Zero Gecko
#elif defined(EFM32ZG210F32)
#include "platform\Device\SiliconLabs\EFM32ZG\Include\efm32zg210f32.h"

#else
#error "sst_em_device.h: PART NUMBER unsupported or undefined (eg. EFM32HG210F64), check compiler global definitions"
#endif

// check part features
#ifndef CMU_PRESENT
#error "error: part has no cmu"
#endif

#endif /* EM_DEVICE_H */


