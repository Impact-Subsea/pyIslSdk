#ifndef TYPESBINDING_H_
#define TYPESBINDING_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initTypes(py::module &m);
}

//--------------------------------------------------------------------------------------------------
#endif