#ifndef UTILS_H_
#define UTILS_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>
#include "types/sdkTypes.h"
#include "utils/stringUtils.h"
#include "maths/vector.h"
#include "maths/matrix.h"
#include "maths/quaternion.h"
#include "devices/device.h"
#include "platform/uart.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    std::string asBinStr(uint_t value, uint_t bits);
    uint_t binStrToValue(const std::string& str);
    bool_t getDictValue(const py::dict& d, const char* key, bool_t defaultValue);
    uint8_t getDictValue(const py::dict& d, const char* key, uint8_t defaultValue);
    uint16_t getDictValue(const py::dict& d, const char* key, uint16_t defaultValue);
    uint32_t getDictValue(const py::dict& d, const char* key, uint32_t defaultValue);
    uint_t getDictValue(const py::dict& d, const char* key, uint_t defaultValue);
    int32_t getDictValue(const py::dict& d, const char* key, int32_t defaultValue);
    real_t getDictValue(const py::dict& d, const char* key, real_t defaultValue);
    std::string getDictValue(const py::dict& d, const char* key, const std::string& defaultValue);
    IslSdk::Uart::Mode getDictValue(const py::dict& d, const char* key, IslSdk::Uart::Mode defaultValue);
    IslSdk::Uart::Parity getDictValue(const py::dict& d, const char* key, IslSdk::Uart::Parity defaultValue);
    IslSdk::Uart::StopBits getDictValue(const py::dict& d, const char* key, IslSdk::Uart::StopBits defaultValue);
    IslSdk::Math::Quaternion getDictValue(const py::dict& d, const char* key, const IslSdk::Math::Quaternion& defaultValue);
    IslSdk::Math::Vector3 getDictValue(const py::dict& d, const char* key, const IslSdk::Math::Vector3& defaultValue);
    IslSdk::Device::CustomStr getDictValue(const py::dict& d, const char* key, const IslSdk::Device::CustomStr& defaultValue);
    IslSdk::Device::PhyPortMode getDictValue(const py::dict& d, const char* key, IslSdk::Device::PhyPortMode defaultValue);
    IslSdk::Device::PhyMdixMode getDictValue(const py::dict& d, const char* key, IslSdk::Device::PhyMdixMode defaultValue);
}

//--------------------------------------------------------------------------------------------------
#endif