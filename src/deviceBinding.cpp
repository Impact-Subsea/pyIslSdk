//------------------------------------------ Includes ----------------------------------------------

#include "deviceBinding.h"
#include "logging/loggingDevice.h"
#include "comms/islHdlc.h"
#include "utils/stringUtils.h"
#include <memory>

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
void PySdk::initDevice(py::module &m)
{
    py::class_<IslHdlc, std::shared_ptr<IslHdlc>> hdlc(m, "Hdlc");
    hdlc.def_readonly("id", &IslHdlc::id, "Id number of the port")
        .def("disconnect", &IslHdlc::disconnect, "Disconnect the port");

    py::class_<Device, LoggingDevice, IslHdlc, std::shared_ptr<Device>> device(m, "Device");
    device.def_property_readonly("info", [](const Device& self) { return self.info; }, "The device info")
        .def_property_readonly("connection", [](const Device& self) { return self.connection.get(); }, "The device's connection info")
        .def("is_connected", &Device::isConnected, "True if the device is connected")
        .def("connect", &Device::connect, "Connect to the device")
        .def("set_rediscovery_timeouts", &Device::setRediscoveryTimeouts, "Set the rediscovery timeouts", "search_timeout ms"_a, "search_count"_a)
        .def("set_comms_retries", &Device::setCommsRetries, "Set the comms retries", "retries"_a)
        .def("reset", &Device::reset, "Reset the device")
        .def("save_config", &Device::saveConfig, "Save the device config", "file_name"_a)
        .def("get_config", &Device::getConfigAsString, "Get the xml device config as a string")
        .def("get_hardware_faults", &Device::getHardwareFaults, "Gets any hardware faults")
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
        .def_readonly("on_comms_timeout", &Device::onCommsTimeout)
        .def_readonly("on_xml_config", &Device::onXmlConfig)
        .def("__repr__", [](const Device& self) { return self.info.name() + " " + self.info.pnSnAsStr();});

    //----------------------------------------- Signals --------------------------------------------
    py::class_<Signal<Device&, const std::string&>>(device, "SignalOnError")                        // onError, onXmlConfig type signal
        .def("connect", &Signal<Device&, const std::string&>::pyConnect, "Connect to the signal with a callback_function(Device device, string msg)")
        .def("disconnect", &Signal<Device&, const std::string&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&>>(device, "SignalOnConnect")                                           // onDelete, onConnect, onDisconnect type signal
        .def("connect", &Signal<Device&>::pyConnect, "Connect to the signal with a callback_function(Device device)")
        .def("disconnect", &Signal<Device&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&, SysPort&, const ConnectionMeta&>>(device, "SignalOnPortAdded")                          // onPortAdded, onPortChanged type signal
        .def("connect", &Signal<Device&, SysPort&, const ConnectionMeta&>::pyConnect, "Connect to the signal with a callback_function(Device device, SysPort port, ConnectionMeta meta)")
        .def("disconnect", &Signal<Device&, SysPort&, const ConnectionMeta&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&, SysPort&>>(device, "SignalOnPortRemoved")                                                       // onPortRemoved type signal
        .def("connect", &Signal<Device&, SysPort&>::pyConnect, "Connect to the signal with a callback_function(Device device, SysPort port)")
        .def("disconnect", &Signal<Device&, SysPort&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&, const Device::Info&>>(device, "SignalOnInfoChanged")                                            // onInfoChanged type signal
        .def("connect", &Signal<Device&, const Device::Info&>::pyConnect, "Connect to the signal with a callback_function(Device device, DeviceInfo info)")
        .def("disconnect", &Signal<Device&, const Device::Info&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&, uint_t, uint_t, uint_t, uint_t>>(device, "SignalOnPacketCount")                                         // onPacketCount type signal
        .def("connect", &Signal<Device&, uint_t, uint_t, uint_t, uint_t>::pyConnect, "Connect to the signal with a callback_function(Device device, uint txBytes, uint rxBytes, uint badPackets, uint missed)")
        .def("disconnect", &Signal<Device&, uint_t, uint_t, uint_t, uint_t>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<Device&, bool_t>>(device, "SignalOnCommsTimeout")                        // onCommsTimeout type signal
        .def("connect", &Signal<Device&, bool_t>::pyConnect, "Connect to the signal with a callback_function(Device device, uint timeout)")
        .def("disconnect", &Signal<Device&, bool_t>::pyDisconnect, "Disconnect from the signal");

    

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
        .def_property_readonly("pn_sn", [](const Device::Info& self){ return StringUtils::pnSnToStr(self.pn, self.sn); }, "Gets the pn, sn as a string")
        .def_property_readonly("firmware_version", [](const Device::Info& self){ return StringUtils::bcdVersionToStr(self.firmwareVersionBcd); }, "Gets the firmware version as a string")
        .def_property_readonly("name", [](const Device::Info& self){ return StringUtils::pidToStr(self.pid); }, "Gets the device name as a string")
        .def("to_dict", [](const Device::Info& self)
        {
            py::dict pyDict;
            pyDict["pid"] = py::cast(self.pid).attr("name").cast<std::string>();
            pyDict["pn"] = self.pn;
            pyDict["sn"] = self.sn;
            pyDict["config"] = self.config;
            pyDict["mode"] = self.mode;
            pyDict["status"] = self.status;
            pyDict["firmware_build_num"] = self.firmwareBuildNum;
            pyDict["firmware_version_bcd"] = self.firmwareVersionBcd;
            pyDict["firmware_version"] = StringUtils::bcdVersionToStr(self.firmwareVersionBcd);
            pyDict["in_use"] = self.inUse;
            return pyDict;
        }, "Returns the device info as a dictionary");

    py::class_<Device::CustomStr>(device, "CustomStr")
        .def(py::init<>())
        .def(py::init<bool_t, const std::string&>())
        .def_readwrite("enable", &Device::CustomStr::enable)
        .def_readwrite("str", &Device::CustomStr::str)
        .def("to_dict", &PySdk::customStrToDict, "Returns the custom string as a dictionary");

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
        .value("MultiPcp", Device::Pid::MultiPcp)
        .value("Any", Device::Pid::Any);

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
py::dict PySdk::customStrToDict(const Device::CustomStr& customStr)
{
    py::dict pyDict;
    pyDict["enable"] = customStr.enable;
    pyDict["str"] = customStr.str;
    return pyDict;
}
//--------------------------------------------------------------------------------------------------
