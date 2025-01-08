//------------------------------------------ Includes ----------------------------------------------

#include "sysPortBinding.h"
#include "comms/ports/sysPort.h"
#include "comms/ports/uartPort.h"
#include "comms/ports/solPort.h"
#include "comms/ports/netPort.h"
#include "comms/ports/poweredComPort.h"
#include "utils/stringUtils.h"
#include "platform/uart.h"

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
py::dict sysPortToDict(const SysPort& self)
{
    py::dict d;
    d["name"] = self.name;
    d["class_type"] = py::cast(self.classType).attr("name").cast<std::string>();
    d["type"] = py::cast(self.type).attr("name").cast<std::string>();
    d["discovery_timeout_ms"] = self.discoveryTimeoutMs;
    d["is_open"] = self.isOpen;
    return d;
}

void PySdk::initSysPort(py::module &m)
{
    py::class_<SysPort, std::shared_ptr<SysPort>> sysPort(m, "SysPort");
    sysPort.def_readonly("id", &SysPort::id, "Id number of the port")
        .def_readonly("discovery_timeout_ms", &SysPort::discoveryTimeoutMs, "Discovery timeout in milliseconds")
        .def_readonly("name", &SysPort::name, "Name of the port")
        .def_readonly("class_type", &SysPort::classType, "Type of class")
        .def_readonly("type", &SysPort::type, "Type of the port")
        .def_property_readonly("is_open", [](const SysPort& self) { return self.isOpen; }, "True if the port is open")

        .def_readonly("on_error", &SysPort::onError, "Error event signal")
        .def_readonly("on_delete", &SysPort::onDelete)
        .def_readonly("on_open", &SysPort::onOpen)
        .def_readonly("on_close", &SysPort::onClose)
        .def_readonly("on_port_stats", &SysPort::onPortStats)
        .def_readonly("on_discovery_started", &SysPort::onDiscoveryStarted)
        .def_readonly("on_discovery_event", &SysPort::onDiscoveryEvent)
        .def_readonly("on_discovery_finished", &SysPort::onDiscoveryFinished)
        .def_readonly("on_rx_data", &SysPort::onRxData)
        .def_readonly("on_tx_data", &SysPort::onTxData)

        .def("stop_discovery", &SysPort::stopDiscovery, "Stop discovery")
        .def("is_discovering", &SysPort::isDiscovering, "True if the port is discovering")
        .def("open", &SysPort::open, "Open the port")
        .def("close", &SysPort::close, "Close the port")
        .def("write", [](SysPort& self, const std::vector<uint8_t>& data, const ConnectionMeta& meta) 
        {
            self.write(data.data(), data.size(), meta);
        }, "Write data to the port specifying baudrate or ip address and port", "data"_a, "meta"_a)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t>(&SysPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a=0xffff, "pn"_a=0xffff, "sn"_a=0xffff)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t, const ConnectionMeta&, uint_t, uint_t>(&SysPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a, "pn"_a, "sn"_a, "meta"_a, "timeout_ms"_a, "count"_a) 
        .def("discover_nmea_devices", &SysPort::discoverNmeaDevices, "Discover NMEA devices")
        .def("__repr__", [](const SysPort& self) { return self.name;})
        .def("to_dict", &sysPortToDict, "Return a dictionary with the port's properties");
    
    //----------------------------------------- Enums ----------------------------------------------
    py::enum_<SysPort::Type>(sysPort, "Type")
        .value("Serial", SysPort::Type::Serial)
        .value("Net", SysPort::Type::Net);

    py::enum_<SysPort::ClassType>(sysPort, "ClassType")
        .value("Serial", SysPort::ClassType::Serial)
        .value("Sol", SysPort::ClassType::Sol)
        .value("Net", SysPort::ClassType::Net)
        .value("Pcp", SysPort::ClassType::Pcp);

    //----------------------------------------- Signals --------------------------------------------
    py::class_<Signal<SysPort&, const std::string&>>(sysPort, "SignalOnError")                        // onError type signal
        .def("connect", &Signal<SysPort&, const std::string&>::pyConnect, "Connect to the signal with a callback_function(SysPort port, string msg)")
        .def("disconnect", &Signal<SysPort&, const std::string&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&>>(sysPort, "SignalOnDelete")                                           // onDelete, onClose type signal
        .def("connect", &Signal<SysPort&>::pyConnect, "Connect to the signal with a callback_function(SysPort port)")
        .def("disconnect", &Signal<SysPort&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, bool_t>>(sysPort, "SignalOnOpen")                                     // onOpen, type signal
        .def("connect", &Signal<SysPort&, bool_t>::pyConnect, "Connect to the signal with a callback_function(SysPort port, bool_t failed)")
        .def("disconnect", &Signal<SysPort&, bool_t>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, uint_t, uint_t, uint_t>>(sysPort, "SignalOnPortStats")                // onPortStats, type signal
        .def("connect", &Signal<SysPort&, uint_t, uint_t, uint_t>::pyConnect, "Connect to the signal with a callback_function(SysPort port, uint txBytes, uint rxBytes, uint badPackets)")
        .def("disconnect", &Signal<SysPort&, uint_t, uint_t, uint_t>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, AutoDiscovery::Type>>(sysPort, "SignalOnDiscoveryStarted")            // onDiscoveryStarted, type signal
        .def("connect", &Signal<SysPort&, AutoDiscovery::Type>::pyConnect, "Connect to the signal with a callback_function(SysPort port, AutoDiscovery::Type type)")
        .def("disconnect", &Signal<SysPort&, AutoDiscovery::Type>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, const ConnectionMeta&, AutoDiscovery::Type, uint_t>>(sysPort, "SignalOnDiscoveryEvent")      // onDiscoveryEvent, type signal
        .def("connect", &Signal<SysPort&, const ConnectionMeta&, AutoDiscovery::Type, uint_t>::pyConnect, "Connect to the signal with a callback_function(SysPort port, ConnectionMeta, meta, AutoDiscovery::Type type, uint_t discoveryCount)")
        .def("disconnect", &Signal<SysPort&, const ConnectionMeta&, AutoDiscovery::Type, uint_t>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, AutoDiscovery::Type, uint_t, bool_t>>(sysPort, "SignalOnDiscoveryFinished")      // onDiscoveryFinished, type signal
        .def("connect", &Signal<SysPort&, AutoDiscovery::Type, uint_t, bool_t>::pyConnect, "Connect to the signal with a callback_function(SysPort port, AutoDiscovery::Type type, uint_t discovery count, bool_t cancelled)")
        .def("disconnect", &Signal<SysPort&, AutoDiscovery::Type, uint_t, bool_t>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, const ConstBuffer&>>(sysPort, "SignalOnTxRxData")                   // onRxData, onTxData, type signal
        .def("connect", &Signal<SysPort&, const ConstBuffer&>::pyConnect, "Connect to the signal with a callback_function(SysPort port, const uint8_t* data, uint_t size)")
        .def("disconnect", &Signal<SysPort&, const ConstBuffer&>::pyDisconnect, "Disconnect from the signal");

    //---------------------------------------- UartPort --------------------------------------------
    py::class_<UartPort, SysPort, std::shared_ptr<UartPort>> uartPort(m, "UartPort");
    uartPort.def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t, uint32_t, uint_t>(&UartPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a, "pn"_a, "sn"_a, "baudrate"_a, "timeout_ms"_a) 
        .def("discover_nmea_devices", py::overload_cast<uint32_t, uint_t>(&UartPort::discoverNmeaDevices), "Discover NMEA devices", "baudrate"_a, "timeout_ms"_a)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t>(&UartPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a=0xffff, "pn"_a=0xffff, "sn"_a=0xffff);

    py::enum_<Uart::Mode>(uartPort, "Mode")
        .value("Rs232", Uart::Mode::Rs232)
        .value("Rs485", Uart::Mode::Rs485)
        .value("Rs485Terminated", Uart::Mode::Rs485Terminated)
        .value("Unknown", Uart::Mode::Unknown)
        .def("to_mode", [](const std::string& mode)
        {
            Uart::Mode m = StringUtils::toUartMode(mode);
            if (m == Uart::Mode::Unknown)
            {
                throw std::invalid_argument("Invalid mode");
            }
            return m;
        }, "Converts a string to a Uart::Mode enum");

    py::enum_<Uart::Parity>(uartPort, "Parity")
        .value("Off", Uart::Parity::None)
        .value("Odd", Uart::Parity::Odd)
        .value("Even", Uart::Parity::Even)
        .value("Mark", Uart::Parity::Mark)
        .value("Space", Uart::Parity::Space)
        .value("Unknown", Uart::Parity::Unknown)
        .def("to_mode", [](const std::string& parity)
        {
            Uart::Parity p = StringUtils::toUartParity(parity);
            if (p == Uart::Parity::Unknown)
            {
                throw std::invalid_argument("Invalid parity mode");
            }
            return p;
        }, "Converts a string to a Uart::Parity enum");

    py::enum_<Uart::StopBits>(uartPort, "StopBits")
        .value("One", Uart::StopBits::One)
        .value("OneAndHalf", Uart::StopBits::OneAndHalf)
        .value("Two", Uart::StopBits::Two)
        .value("Unknown", Uart::StopBits::Unknown)
        .def("to_mode", [](const std::string& stopBits)
        {
            Uart::StopBits s = StringUtils::toUartStopBits(stopBits);
            if (s == Uart::StopBits::Unknown)
            {
                throw std::invalid_argument("Invalid stop bits mode");
            }
            return s;
        }, "Converts a string to a Uart::StopBits enum");

    //---------------------------------------- SolPort ---------------------------------------------
    py::class_<SolPort, SysPort, std::shared_ptr<SolPort>>(m, "SolPort")
        .def("config", [](SolPort& self, bool_t isTcp, bool_t useTelnet, const std::string& ipAddress, uint16_t port)
        {
            bool_t err = false;
            self.config(isTcp, useTelnet, StringUtils::toIp(ipAddress, err), port);
        }, "Configures SOL", "is_tcp"_a, "use _rfc2217"_a, "ip_address"_a, "port"_a)
        .def("set_serial", &SolPort::setSerial, "Configures serial settings if useRFC2217 is true", "baudrate"_a, "data_bits"_a, "parity"_a, "stop_bits"_a)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t, uint32_t, uint_t>(&SolPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a, "pn"_a, "sn"_a, "baudrate"_a, "timeout_ms"_a)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t>(&SolPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a=0xffff, "pn"_a=0xffff, "sn"_a=0xffff)
        .def("discover_nmea_devices", py::overload_cast<uint32_t, uint_t>(&SolPort::discoverNmeaDevices), "Discover NMEA devices", "baudrate"_a, "timeout_ms"_a);
   
    //---------------------------------------- NetPort --------------------------------------------
    py::class_<NetPort, SysPort, std::shared_ptr<NetPort>>(m, "NetPort")
        .def("discover_isl_devices", [](NetPort& self, uint16_t pid, uint16_t pn, uint16_t sn, const std::string& ipAddress, uint16_t port, uint_t timeoutMs)
        {
            bool_t err = false;
            self.discoverIslDevices(pid, pn, sn, StringUtils::toIp(ipAddress, err), port, timeoutMs);
        }, "Discover Impact Subsea devices", "pid"_a=0xffff, "pn"_a=0xffff, "sn"_a=0xffff, "ip_address"_a, "port"_a=33005, "timeout ms"_a=1000);

    //----------------------------------- PoweredComPort ----------------------------------------
    py::class_<PoweredComPort, SysPort, std::shared_ptr<PoweredComPort>>(m, "PoweredComPort")
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t, uint32_t, uint_t>(&PoweredComPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a, "pn"_a, "sn"_a, "baudrate"_a, "timeout_ms"_a)
        .def("discover_nmea_devices", py::overload_cast<uint32_t, uint_t>(&PoweredComPort::discoverNmeaDevices), "Discover NMEA devices", "baudrate"_a, "timeout_ms"_a)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t>(&PoweredComPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a=0xffff, "pn"_a=0xffff, "sn"_a=0xffff)
        .def("set_serial", &PoweredComPort::setSerial, "Configures serial settings", "baudrate"_a, "data_bits"_a, "parity"_a, "stop_bits"_a)
        .def("set_auto_power_on_open", &PoweredComPort::setAutoPowerOnOpen, "The delay in milliseconds between power on and opening the port. A value of zero will disable auto power on.")
        .def("get_device_id", &PoweredComPort::getDeviceId, "Get the id of the device which created this port")
        .def("get_index", &PoweredComPort::getIndex, "Get the index of this port that the parent device assigned")
        .def("get_device_connection_meta", &PoweredComPort::getDeviceConnectionMeta, "Get the connection meta of the device which created this port")
        .def("to_dict", [](const PoweredComPort& self) -> py::dict
        {
            py::dict d = sysPortToDict(self);
            d["parent_id"] = self.getDeviceId();
            d["idx"] = self.getIndex();
            return d;
        }, "Return a dictionary with the port's properties");
}
//--------------------------------------------------------------------------------------------------