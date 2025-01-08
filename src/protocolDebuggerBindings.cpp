//------------------------------------------ Includes ----------------------------------------------

#include "protocolDebuggerBindings.h"
#include "comms/protocolDebugger.h"

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
void PySdk::initProtocolDebugger(py::module &m)
{
    py::class_<ProtocolDebugger>(m, "ProtocolDebugger")
        .def(py::init<const std::string&>(), "name"_a)
        .def("monitor_port", &ProtocolDebugger::monitorPort, "port"_a)
        .def_readwrite("show_payload", &ProtocolDebugger::m_showPayload);
}
//--------------------------------------------------------------------------------------------------