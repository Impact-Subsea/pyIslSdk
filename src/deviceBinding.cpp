//------------------------------------------ Includes ----------------------------------------------

#include "deviceBinding.h"
#include "devices/device.h"
#include "logging/loggingDevice.h"
#include "comms/islHdlc.h"
#include <memory>

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
void PySdk::initDevice(py::module &m)
{
    py::class_<IslHdlc, std::shared_ptr<IslHdlc>> hdlc(m, "Hdlc");
    hdlc.def_readonly("id", &IslHdlc::id, "Id number of the port")
        .def_property_readonly("is_connected", [](const IslHdlc& self) { return self.isConnected; });

    py::class_<Device, LoggingDevice, IslHdlc, std::shared_ptr<Device>> device(m, "Device");
    device.def_property_readonly("info", [](const Device& self) { return self.info; }, "The device info")
        .def_property_readonly("connection", [](const Device& self) { return self.connection.get(); }, "The device's connection info")
        .def("connect", &Device::connect, "Connect to the device")
        .def("set_rediscovery_timeouts", &Device::setRediscoveryTimeouts, "Set the rediscovery timeouts", "search timeout ms"_a, "search count"_a)
        .def("set_comms_retries", &Device::setCommsRetries, "Set the comms retries", "retries"_a)
        .def("reset", &Device::reset, "Reset the device")
        .def("save_config", &Device::saveConfig, "Save the device config", "file name"_a)
        .def("bootloader_mode", &Device::bootloaderMode, "True if the device is in bootloader mode")
        .def_readonly("on_error", &Device::onError)
        .def_readonly("on_delete", &Device::onDelete)
        .def_readonly("on_connect", &Device::onConnect)
        .def_readonly("on_disconnect", &Device::onDisconnect)
        .def_readonly("on_port_added", &Device::onPortAdded)
        .def_readonly("on_port_changed", &Device::onPortChanged)
        .def_readonly("on_port_removed", &Device::onPortRemoved)
        .def_readonly("on_info_changed", &Device::onInfoChanged)
        .def_readonly("on_packet_count", &Device::onPacketCount)
        .def("__repr__", [](const Device& self) { return self.info.name() + " " + self.info.pnSnAsStr();});

    //----------------------------------------- Signals --------------------------------------------
    py::class_<Signal<Device&, const std::string&>>(device, "Signal_OnError")                        // onError type signal
        .def("connect", &Signal<Device&, const std::string&>::pyConnect, "Connect to the signal with a callback_function(Device device, string msg)")
        .def("disconnect", &Signal<Device&, const std::string&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&>>(device, "Signal_OnConnect")                                           // onDelete, onConnect, onDisconnect type signal
        .def("connect", &Signal<Device&>::pyConnect, "Connect to the signal with a callback_function(Device device)")
        .def("disconnect", &Signal<Device&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&, SysPort&, const ConnectionMeta&>>(device, "SignalOnPortAdded")                          // onPortAdded, onPortChanged type signal
        .def("connect", &Signal<Device&, SysPort&, const ConnectionMeta&>::pyConnect, "Connect to the signal with a callback_function(Device device, SysPort port, ConnectionMeta meta)")
        .def("disconnect", &Signal<Device&, SysPort&, const ConnectionMeta&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&, SysPort&>>(device, "Signal_OnPortRemoved")                                                       // onPortRemoved type signal
        .def("connect", &Signal<Device&, SysPort&>::pyConnect, "Connect to the signal with a callback_function(Device device, SysPort port)")
        .def("disconnect", &Signal<Device&, SysPort&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&, const Device::Info&>>(device, "Signal_OnInfoChanged")                                            // onInfoChanged type signal
        .def("connect", &Signal<Device&, const Device::Info&>::pyConnect, "Connect to the signal with a callback_function(Device device, DeviceInfo info)")
        .def("disconnect", &Signal<Device&, const Device::Info&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&, uint_t, uint_t, uint_t, uint_t>>(device, "Signal_OnPacketCount")                                         // onPacketCount type signal
        .def("connect", &Signal<Device&, uint_t, uint_t, uint_t, uint_t>::pyConnect, "Connect to the signal with a callback_function(Device device, uint txBytes, uint rxBytes, uint badPackets, uint missed)")
        .def("disconnect", &Signal<Device&, uint_t, uint_t, uint_t, uint_t>::pyDisconnect, "Disconnect from the signal");
    

    //------------------------------------------ Info ----------------------------------------------
    py::class_<Device::Info>(device, "Info")
        .def(py::init<>())
        .def_readwrite("pid", &Device::Info::pid)
        .def_readwrite("pn", &Device::Info::pn)
        .def_readwrite("sn", &Device::Info::sn)
        .def_readwrite("config", &Device::Info::config)
        .def_readwrite("mode", &Device::Info::mode)
        .def_readwrite("status", &Device::Info::status)
        .def_readwrite("firmware_build_num", &Device::Info::firmwareBuildNum)
        .def_readwrite("firmware_version_bcd", &Device::Info::firmwareVersionBcd)
        .def_readwrite("in_use", &Device::Info::inUse)
        .def("pn_sn_str", &Device::Info::pnSnAsStr);

    py::class_<Device::CustomStr>(device, "CustomStr")
        .def(py::init<>())
        .def(py::init<bool_t, const std::string&>())
        .def_readwrite("enable", &Device::CustomStr::enable)
        .def_readwrite("str", &Device::CustomStr::str);

    //----------------------------------------- Enums ----------------------------------------------
    py::enum_<Device::Pid>(device, "Pid")
        .value("Unknown", Device::Pid::Unknown)
        .value("Isa500v1", Device::Pid::Isa500v1)
        .value("Isd4000v1", Device::Pid::Isd4000v1)
        .value("Ism3dv1", Device::Pid::Ism3dv1)
        .value("Iss360v1", Device::Pid::Iss360v1)
        .value("Isa500", Device::Pid::Isa500)
        .value("Isd4000", Device::Pid::Isd4000)
        .value("Ism3d", Device::Pid::Ism3d)
        .value("Sonar", Device::Pid::Sonar)
        .value("Any", Device::Pid::Any);

    py::enum_<Device::UartMode>(device, "UartMode")
        .value("Rs232", Device::UartMode::Rs232)
        .value("Rs485", Device::UartMode::Rs485)
        .value("Rs485Terminated", Device::UartMode::Rs485Terminated)
        .value("Unknown", Device::UartMode::Unknown);

    py::enum_<Device::Parity>(device, "Parity")
        .value("Off", Device::Parity::None)
        .value("Odd", Device::Parity::Odd)
        .value("Even", Device::Parity::Even)
        .value("Mark", Device::Parity::Mark)
        .value("Space", Device::Parity::Space)
        .value("Unknown", Device::Parity::Unknown);

    py::enum_<Device::StopBits>(device, "StopBits")
        .value("One", Device::StopBits::One)
        .value("OneAndHalf", Device::StopBits::OneAndHalf)
        .value("Two", Device::StopBits::Two)
        .value("Unknown", Device::StopBits::Unknown);

    py::enum_<Device::PhyMdixMode>(device, "PhyMdixMode")
        .value("Normal", Device::PhyMdixMode::Normal)
        .value("Swapped", Device::PhyMdixMode::Swapped)
        .value("Auto", Device::PhyMdixMode::Auto)
        .value("Unknown", Device::PhyMdixMode::Unknown);

    py::enum_<Device::PhyPortMode>(device, "PhyPortMode")
        .value("Auto", Device::PhyPortMode::Auto)
        .value("Base10TxHalf", Device::PhyPortMode::Base10TxHalf)
        .value("Base10TxFull", Device::PhyPortMode::Base10TxFull)
        .value("Base100TxHalf", Device::PhyPortMode::Base100TxHalf)
        .value("Base100TxFull", Device::PhyPortMode::Base100TxFull)
        .value("Unknown", Device::PhyPortMode::Unknown);

    
}
//--------------------------------------------------------------------------------------------------