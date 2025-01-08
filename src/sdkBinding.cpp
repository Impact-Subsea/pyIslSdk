//------------------------------------------ Includes ----------------------------------------------

#include "typesBinding.h"
#include "ahrsBinding.h"
#include "loggingBinding.h"
#include "sysPortBinding.h"
#include "deviceBinding.h"
#include "isa500Binding.h"
#include "isd4000Binding.h"
#include "ism3dBinding.h"
#include "sonarBinding.h"
#include "multiPcpBinding.h"
#include "protocolDebuggerBindings.h"
#include "sdk.h"
#include "utils/stringUtils.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
PYBIND11_MODULE(pyIslSdk, m)
{
    PySdk::initTypes(m);
    PySdk::initAhrs(m);
    PySdk::initLogging(m);
    PySdk::initSysPort(m);
    PySdk::initDevice(m);
    PySdk::initIsa500(m);
    PySdk::initIsd4000(m);
    PySdk::initIsm3d(m);
    PySdk::initSonar(m);
    PySdk::initMultiPcp(m);
    PySdk::initProtocolDebugger(m);
    
    m.attr("__version__") = "1.0.1";
    
    py::module SeaViewAppType = m.def_submodule("SeaViewAppType");
    SeaViewAppType.attr("isa500") = py::bytes("\x00\x00\x00\x00\x01\x00\x00\x00", 8);
    SeaViewAppType.attr("isd4000") = py::bytes("\x01\x00\x00\x00\x01\x00\x00\x00", 8);
    SeaViewAppType.attr("ism3d") = py::bytes("\x02\x00\x00\x00\x01\x00\x00\x00", 8);
    SeaViewAppType.attr("fmd") = py::bytes("\x03\x00\x00\x00\x01\x00\x00\x00", 8);
    SeaViewAppType.attr("sonar") = py::bytes("\x04\x00\x00\x00\x01\x00\x00\x00", 8);
    SeaViewAppType.attr("cam") = py::bytes("\x05\x00\x00\x00\x01\x00\x00\x00", 8);
    SeaViewAppType.attr("profiler") = py::bytes("\x06\x00\x00\x00\x01\x00\x00\x00", 8);

    m.def("ip_to_str", &StringUtils::ipToStr, "Convert an IP address to a string", "ip"_a);

    py::class_<Sdk>(m, "Sdk")
        .def(py::init<>())
        .def("run", &Sdk::run)
        .def_readonly("version", &Sdk::version)
        .def_readonly("ports", &Sdk::ports)
        .def_readonly("devices", &Sdk::devices);
    
    py::class_<DeviceMgr> deviceMgr(m, "DeviceMgr");
    deviceMgr.def_property_readonly("list", [](const DeviceMgr& self) { return self.deviceList; }, "The list of devices")
        .def("set_comms_timeouts", &DeviceMgr::setCommsTimeouts, "Set the comms timeouts", "host_timeout_ms"_a, "device_timeout_ms"_a, "tx_retries"_a)
        .def_readonly("on_new", &DeviceMgr::onNew)
        .def("find_by_id", &DeviceMgr::findById, "Find a device by id", "id"_a)
        .def("find_by_pn_sn", &DeviceMgr::findByPnSn, "Find a device by part number and serial number", "pn"_a, "sn"_a)
        .def("remove", &DeviceMgr::remove, "Remove a device", "device"_a);

    py::class_<Signal<const Device::SharedPtr&, const SysPort::SharedPtr&, const ConnectionMeta&>>(deviceMgr, "SignalOnNew")        // onNew type signal
        .def("connect", &Signal<const Device::SharedPtr&, const SysPort::SharedPtr&, const ConnectionMeta&>::pyConnect)
        .def("disconnect", &Signal<const Device::SharedPtr&, const SysPort::SharedPtr&, const ConnectionMeta&>::pyDisconnect);

    py::class_<SysPortMgr> sysPortMgr(m, "SysPortMgr");
    sysPortMgr.def_property_readonly("list", [](const SysPortMgr& self) { return self.sysPortList; }, "The list of ports")
        .def_readonly("on_new", &SysPortMgr::onNew)
        .def("find_by_id", &SysPortMgr::findById, "Find a port by id", "id"_a)
        .def("create_sol", [](SysPortMgr& self, const std::string& name, bool_t useTcp, bool_t useRFC2217, const std::string& ipAddress, uint16_t port) {
                                bool_t err = false;
                                self.createSol(name, useTcp, useRFC2217, StringUtils::toIp(ipAddress, err), port);
                                }, "Create a new Sol port", "name"_a, "use_tcp"_a, "use_RFC2217"_a, "ip_address"_a, "port"_a)
        .def("delete_sol", &SysPortMgr::deleteSolSysPort, "Delete a Sol port", "port"_a);

    py::class_<Signal<const SysPort::SharedPtr&>>(sysPortMgr, "SignalOnNew")        // onNew type signal
        .def("connect", &Signal<const SysPort::SharedPtr&>::pyConnect)
        .def("disconnect", &Signal<const SysPort::SharedPtr&>::pyDisconnect);
}
//--------------------------------------------------------------------------------------------------