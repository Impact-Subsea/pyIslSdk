#ifndef ISA500BINDING_H_
#define ISA500BINDING_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initIsa500(py::module &m);
}

//--------------------------------------------------------------------------------------------------
#endif