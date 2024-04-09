#ifndef LOGGINGBINDINGS_H_
#define LOGGINGBINDINGS_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initLogging(py::module &m);
}

//--------------------------------------------------------------------------------------------------
#endif