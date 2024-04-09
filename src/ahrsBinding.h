#ifndef AHRSBINDING_H_
#define AHRSBINDING_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initAhrs(py::module &m);
}

//--------------------------------------------------------------------------------------------------
#endif