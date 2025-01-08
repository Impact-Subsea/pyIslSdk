#ifndef MULTIPCPBINDING_H_
#define MULTIPCPBINDING_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initMultiPcp(py::module &m);
}

//--------------------------------------------------------------------------------------------------
#endif