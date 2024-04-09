#ifndef SYSPORTBINDINGS_H_
#define SYSPORTBINDINGS_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initSysPort(py::module &m);
}

//--------------------------------------------------------------------------------------------------
#endif