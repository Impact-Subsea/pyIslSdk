#ifndef SONARBINDING_H_
#define SONARBINDING_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initSonar(py::module &m);
}

//--------------------------------------------------------------------------------------------------
#endif