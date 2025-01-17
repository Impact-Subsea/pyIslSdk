#ifndef DEVICEBINDING_H_
#define DEVICEBINDING_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>
#include "devices/device.h"

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initDevice(py::module &m);
    py::dict customStrToDict(const IslSdk::Device::CustomStr& customStr);
}

//--------------------------------------------------------------------------------------------------
#endif