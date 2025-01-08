#ifndef PROTOCOLDEBUGGERBINDING_H_
#define PROTOCOLDEBUGGERBINDING_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initProtocolDebugger(py::module &m);
}

//--------------------------------------------------------------------------------------------------
#endif