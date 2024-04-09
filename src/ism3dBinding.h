#ifndef ISM3DBINDING_H_
#define ISM3DBINDING_H_

//------------------------------------------ Includes ----------------------------------------------

#include <pybind11/pybind11.h>

//--------------------------------------- Class Definition -----------------------------------------

namespace PySdk
{
    namespace py = pybind11;

    void initIsm3d(py::module &m);
}

//--------------------------------------------------------------------------------------------------
#endif