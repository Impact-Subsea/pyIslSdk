#ifndef DEVICEBINDING_H_
#define DEVICEBINDING_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initDevice(py::module &m);
}

//--------------------------------------------------------------------------------------------------
#endif