//------------------------------------------ Includes ----------------------------------------------

#include "multiPcpBinding.h"
#include "devices/multiPcp.h"
#include "utils/stringUtils.h"
#include "utils.h"
#include <pybind11/stl.h>

using namespace pybind11::literals;
using namespace IslSdk;

MultiPcp::Settings dictToSettings(const py::dict& d, const MultiPcp::Settings& defualt);
PcpDevice::Settings dictToSettings(const py::dict& d, const PcpDevice::Settings& defualt);

//--------------------------------------------------------------------------------------------------
void PySdk::initMultiPcp(py::module &m)
{
    py::class_<MultiPcp, Device, std::shared_ptr<MultiPcp>> multiPcp(m, "MultiPcp");
    multiPcp.def("set_settings", &MultiPcp::setSettings, "Set the settings", "settings"_a, "save"_a)
        .def("set_settings", [](MultiPcp& self, const py::dict& dict, bool save) { 
            MultiPcp::Settings settings = dictToSettings(dict, self.settings);
            self.setSettings(settings, save);
        }, "Set the settings from a dictionary", "settings_dict"_a, "save"_a = false)   
        .def_readonly("on_settings_updated", &MultiPcp::onSettingsUpdated, "Signal emitted when the settings are updated")
        .def_property_readonly("settings", [](const MultiPcp& self) { return self.settings; }, "Get the settings")
        .def_property_readonly("pcp_devices", [](const MultiPcp& self) { return self.pcpDevices; }, "Get the pcp devices");

    py::class_<Signal<MultiPcp&, bool_t>>(multiPcp, "SignalOnSettingsUpdated")
        .def("connect", &Signal<MultiPcp&, bool_t>::pyConnect, "Connect to the signal with a callback_function(multiPcp, success)")
        .def("disconnect", &Signal<MultiPcp&, bool_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<MultiPcp::Settings> settings(multiPcp, "Settings");
    settings.def(py::init<>())
        .def(py::init([](const py::dict& d) { return dictToSettings(d, MultiPcp::Settings()); }))
        .def_property("ip_address", [](const MultiPcp::Settings& self) { return StringUtils::ipToStr(self.ipAddress);}, [](MultiPcp::Settings& self, const std::string& ipAddress) { bool_t err; self.ipAddress = StringUtils::toIp(ipAddress, err);}, "ip address")
        .def_property("netmask", [](const MultiPcp::Settings& self) { return StringUtils::ipToStr(self.netmask);}, [](MultiPcp::Settings& self, const std::string& netmask) { bool_t err; self.netmask = StringUtils::toIp(netmask, err);}, "netmask")
        .def_property("gateway", [](const MultiPcp::Settings& self) { return StringUtils::ipToStr(self.gateway);}, [](MultiPcp::Settings& self, const std::string& gateway) { bool_t err; self.gateway = StringUtils::toIp(gateway, err);}, "gateway")  
        .def_readwrite("port", &MultiPcp::Settings::port)
        .def_readwrite("use_dhcp", &MultiPcp::Settings::useDhcp)
        .def_readwrite("phy_port_mode", &MultiPcp::Settings::phyPortMode)
        .def_readwrite("phy_mdix_mode", &MultiPcp::Settings::phyMdixMode)
        .def("defaults", &MultiPcp::Settings::defaults)
        .def("to_dict", [](const MultiPcp::Settings& self)
        { 
            py::dict d;
            d["ip_address"] = StringUtils::ipToStr(self.ipAddress);
            d["netmask"] = StringUtils::ipToStr(self.netmask);
            d["gateway"] = StringUtils::ipToStr(self.gateway);
            d["port"] = self.port;
            d["use_dhcp"] = self.useDhcp;
            d["phy_port_mode"] = py::cast(self.phyPortMode).attr("name").cast<std::string>();
            d["phy_mdix_mode"] = py::cast(self.phyMdixMode).attr("name").cast<std::string>();
            return d;
        }, "Convert the settings to a dictionary");


    py::class_<PcpServices, std::shared_ptr<PcpServices>> pcpServices(m, "PcpServices");
    pcpServices.def("open", &PcpServices::open, "Open the port")
        .def("close", &PcpServices::close, "Close the port")
        .def("write", [](PcpServices& self, const std::vector<uint8_t>& data) { return self.write(data.data(), data.size()); }, "Write data to the port", "data"_a)
        .def("set_serial", py::overload_cast<uint32_t, uint8_t, Uart::Parity, Uart::StopBits>(&PcpServices::setSerial), "Set the serial port settings", "baudrate"_a, "data_bits"_a, "parity"_a, "stop_bits"_a)
        .def("set_serial", py::overload_cast<uint32_t>(&PcpServices::setSerial), "Set the serial port baudrate", "baudrate"_a)
        .def("set_power", &PcpServices::setPower, "Set the power", "on"_a)
        .def("set_mode", &PcpServices::setMode, "Set the mode", "mode"_a);
   

    py::class_<PcpDevice, PcpServices, std::shared_ptr<PcpDevice>> pcpDevice(m, "PcpDevice");
    pcpDevice.def("set_settings", &PcpDevice::setSettings, "Set the settings", "settings"_a, "save"_a)
        .def("set_settings", [](PcpDevice& self, const py::dict& dict, bool save) { 
            PcpDevice::Settings settings = dictToSettings(dict, self.settings);
            self.setSettings(settings, save);
        }, "Set the settings from a dictionary", "settings_dict"_a, "save"_a = false)  
        .def_readonly("on_power_stats", &PcpDevice::onPowerStats, "Signal emitted when the power stats are updated")
        .def_readonly("on_settings_updated", &PcpDevice::onSettingsUpdated, "Signal emitted when the settings are updated")
        .def_readonly("pcp", &PcpDevice::pcp, "Get the port")
        .def_property_readonly("settings", [](const PcpDevice& self) { return self.settings; }, "Get the settings")
        .def_property_readonly("id", [](const PcpDevice& self) { return self.id; }, "Get the id");

    py::class_<Signal<PcpDevice&, real_t, real_t>>(pcpDevice, "SignalOnPowerStats")
        .def("connect", &Signal<PcpDevice&, real_t, real_t>::pyConnect, "Connect to the signal with a callback_function(pcpDevice, power, stats)")
        .def("disconnect", &Signal<PcpDevice&, real_t, real_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<PcpDevice&, bool_t>>(pcpDevice, "SignalOnSettingsUpdated")
        .def("connect", &Signal<PcpDevice&, bool_t>::pyConnect, "Connect to the signal with a callback_function(pcpDevice, success)")
        .def("disconnect", &Signal<PcpDevice&, bool_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<PcpDevice::Settings> pcpMgrSettings(pcpDevice, "Settings");
    pcpMgrSettings.def(py::init<>())
        .def(py::init([](const py::dict& d) { return dictToSettings(d, PcpDevice::Settings()); }))
        .def_readwrite("power_on", &PcpDevice::Settings::powerOn)
        .def_readwrite("enabled", &PcpDevice::Settings::enabled)
        .def_readwrite("port_protocol", &PcpDevice::Settings::portProtocol)
        .def_readwrite("baudrate", &PcpDevice::Settings::baudrate)
        .def_readwrite("data_bits", &PcpDevice::Settings::dataBits)
        .def_readwrite("parity", &PcpDevice::Settings::parity)
        .def_readwrite("stop_bits", &PcpDevice::Settings::stopBits)
        .def("defaults", &PcpDevice::Settings::defaults)
        .def("to_dict", [](const PcpDevice::Settings& self)
        {
            py::dict d;
            d["power_on"] = self.powerOn;
            d["enabled"] = self.enabled;
            d["port_protocol"] = py::cast(self.portProtocol).attr("name").cast<std::string>();
            d["baudrate"] = self.baudrate;
            d["data_bits"] = self.dataBits;
            d["parity"] = py::cast(self.parity).attr("name").cast<std::string>();
            d["stop_bits"] = py::cast(self.stopBits).attr("name").cast<std::string>();
            return d;
        }, "Convert the settings to a dictionary");
}
//--------------------------------------------------------------------------------------------------
MultiPcp::Settings dictToSettings(const py::dict& d, const MultiPcp::Settings& defualt)
{
    bool_t err;
    MultiPcp::Settings settings;

    settings.ipAddress = StringUtils::toIp(PySdk::getDictValue(d, "ip_address", StringUtils::ipToStr(defualt.ipAddress)), err);
    settings.netmask = StringUtils::toIp(PySdk::getDictValue(d, "netmask", StringUtils::ipToStr(defualt.netmask)), err);
    settings.gateway = StringUtils::toIp(PySdk::getDictValue(d, "gateway", StringUtils::ipToStr(defualt.gateway)), err);
    settings.port = PySdk::getDictValue(d, "port", defualt.port);
    settings.useDhcp = PySdk::getDictValue(d, "use_dhcp", defualt.useDhcp);
    settings.phyPortMode = PySdk::getDictValue(d, "phy_port_mode", defualt.phyPortMode);
    settings.phyMdixMode = PySdk::getDictValue(d, "phy_mdix_mode", defualt.phyMdixMode);
    return settings;
}
//--------------------------------------------------------------------------------------------------
PcpDevice::Settings dictToSettings(const py::dict& d, const PcpDevice::Settings& defualt)
{
    PcpDevice::Settings settings;

    settings.powerOn = PySdk::getDictValue(d, "power_on", defualt.powerOn);
    settings.enabled = PySdk::getDictValue(d, "enabled", defualt.enabled);
    settings.portProtocol = PySdk::getDictValue(d, "port_protocol", defualt.portProtocol);
    settings.baudrate = PySdk::getDictValue(d, "baudrate", defualt.baudrate);
    settings.dataBits = PySdk::getDictValue(d, "data_bits", defualt.dataBits);
    settings.parity = PySdk::getDictValue(d, "parity", defualt.parity);
    settings.stopBits = PySdk::getDictValue(d, "stop_bits", defualt.stopBits);
    return settings;
}
//--------------------------------------------------------------------------------------------------
