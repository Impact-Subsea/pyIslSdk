//------------------------------------------ Includes ----------------------------------------------

#include "sysPortBinding.h"
#include "comms/ports/sysPort.h"
#include "comms/ports/uartPort.h"
#include "comms/ports/solPort.h"
#include "comms/ports/netPort.h"
#include "utils/stringUtils.h"
#include "platform/uart.h"

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
void PySdk::initSysPort(py::module &m)
{
    py::class_<SysPort, std::shared_ptr<SysPort>> sysPort(m, "SysPort");
    sysPort.def_readonly("id", &SysPort::id, "Id number of the port")
        .def_readonly("discovery_timeout_ms", &SysPort::discoveryTimeoutMs, "Discovery timeout in milliseconds")
        .def_readonly("name", &SysPort::name, "Name of the port")
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
        .def("write", &SysPort::write, "Write data to the port specifying baudrate or ip address and port", "data"_a, "size"_a, "meta"_a)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t>(&SysPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a=0xffff, "pn"_a=0xffff, "sn"_a=0xffff)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t, const ConnectionMeta&, uint_t, uint_t>(&SysPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a, "pn"_a, "sn"_a, "meta"_a, "timeoutMs"_a, "count"_a) 
        .def("discover_nmea_devices", &SysPort::discoverNmeaDevices, "Discover NMEA devices")
        .def("__repr__", [](const SysPort& self) { return self.name;});
    
    //----------------------------------------- Enums ----------------------------------------------
    py::enum_<SysPort::Type>(sysPort, "Type")
        .value("Serial", SysPort::Type::Serial)
        .value("Sol", SysPort::Type::Sol)
        .value("Net", SysPort::Type::Net);

    //----------------------------------------- Signals --------------------------------------------
    py::class_<Signal<SysPort&, const std::string&>>(sysPort, "Signal_OnError")                        // onError type signal
        .def("connect", &Signal<SysPort&, const std::string&>::pyConnect, "Connect to the signal with a callback_function(SysPort port, string msg)")
        .def("disconnect", &Signal<SysPort&, const std::string&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&>>(sysPort, "Signal_OnDelete")                                           // onDelete, onClose type signal
        .def("connect", &Signal<SysPort&>::pyConnect, "Connect to the signal with a callback_function(SysPort port)")
        .def("disconnect", &Signal<SysPort&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, bool_t>>(sysPort, "Signal_OnOpen")                                     // onOpen, type signal
        .def("connect", &Signal<SysPort&, bool_t>::pyConnect, "Connect to the signal with a callback_function(SysPort port, bool_t failed)")
        .def("disconnect", &Signal<SysPort&, bool_t>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, uint_t, uint_t, uint_t>>(sysPort, "Signal_OnPortStats")                // onPortStats, type signal
        .def("connect", &Signal<SysPort&, uint_t, uint_t, uint_t>::pyConnect, "Connect to the signal with a callback_function(SysPort port, uint txBytes, uint rxBytes, uint badPackets)")
        .def("disconnect", &Signal<SysPort&, uint_t, uint_t, uint_t>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, AutoDiscovery::Type>>(sysPort, "Signal_OnDiscoveryStarted")            // onDiscoveryStarted, type signal
        .def("connect", &Signal<SysPort&, AutoDiscovery::Type>::pyConnect, "Connect to the signal with a callback_function(SysPort port, AutoDiscovery::Type type)")
        .def("disconnect", &Signal<SysPort&, AutoDiscovery::Type>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, const ConnectionMeta&, AutoDiscovery::Type, uint_t>>(sysPort, "Signal_OnDiscoveryEvent")      // onDiscoveryEvent, type signal
        .def("connect", &Signal<SysPort&, const ConnectionMeta&, AutoDiscovery::Type, uint_t>::pyConnect, "Connect to the signal with a callback_function(SysPort port, ConnectionMeta, meta, AutoDiscovery::Type type, uint_t discoveryCount)")
        .def("disconnect", &Signal<SysPort&, const ConnectionMeta&, AutoDiscovery::Type, uint_t>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, AutoDiscovery::Type, uint_t, bool_t>>(sysPort, "Signal_OnDiscoveryFinished")      // onDiscoveryFinished, type signal
        .def("connect", &Signal<SysPort&, AutoDiscovery::Type, uint_t, bool_t>::pyConnect, "Connect to the signal with a callback_function(SysPort port, AutoDiscovery::Type type, uint_t discovery count, bool_t cancelled)")
        .def("disconnect", &Signal<SysPort&, AutoDiscovery::Type, uint_t, bool_t>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<SysPort&, const ConstBuffer&>>(sysPort, "Signal_OnRxData")                   // onRxData, onTxData, type signal
        .def("connect", &Signal<SysPort&, const ConstBuffer&>::pyConnect, "Connect to the signal with a callback_function(SysPort port, const uint8_t* data, uint_t size)")
        .def("disconnect", &Signal<SysPort&, const ConstBuffer&>::pyDisconnect, "Disconnect from the signal");

    //---------------------------------------- UartPort --------------------------------------------
    py::class_<UartPort, SysPort, std::shared_ptr<UartPort>> uartPort(m, "UartPort");
    uartPort.def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t>(&SysPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a=0xffff, "pn"_a=0xffff, "sn"_a=0xffff)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t, uint32_t, uint_t>(&UartPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a, "pn"_a, "sn"_a, "baudrate"_a, "timeout ms"_a) 
        .def("discover_nmea_devices", py::overload_cast<uint32_t, uint_t>(&UartPort::discoverNmeaDevices), "Discover NMEA devices", "baudrate"_a, "timeout ms"_a);
    
    py::enum_<Uart::Parity>(uartPort, "Parity")
        .value("Off", Uart::Parity::None)
        .value("Odd", Uart::Parity::Odd)
        .value("Even", Uart::Parity::Even)
        .value("Mark", Uart::Parity::Mark)
        .value("Space", Uart::Parity::Space)
        .value("Unknown", Uart::Parity::Unknown);

    py::enum_<Uart::StopBits>(uartPort, "StopBits")
        .value("One", Uart::StopBits::One)
        .value("OneAndHalf", Uart::StopBits::OneAndHalf)
        .value("Two", Uart::StopBits::Two)
        .value("Unknown", Uart::StopBits::Unknown);

    //---------------------------------------- SolPort ---------------------------------------------
    py::class_<SolPort, SysPort, std::shared_ptr<SolPort>>(m, "SolPort")
        .def("config", [](SolPort& self, bool_t isTcp, bool_t useTelnet, const std::string& ipAddress, uint16_t port) {
                                bool_t err = false;
                                self.config(isTcp, useTelnet, StringUtils::toIp(ipAddress, err), port);
                                }, "Configures SOL", "isTcp"_a, "useRFC2217"_a, "ipAddress"_a, "port"_a)
        .def("set_serial", &SolPort::setSerial, "Configures serial settings if useRFC2217 is true", "baudrate"_a, "data bits"_a, "parity"_a, "stop bits"_a)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t>(&SysPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a=0xffff, "pn"_a=0xffff, "sn"_a=0xffff)
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t, uint32_t, uint_t>(&SolPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a, "pn"_a, "sn"_a, "baudrate"_a, "timeout ms"_a) 
        .def("discover_nmea_devices", py::overload_cast<uint32_t, uint_t>(&SolPort::discoverNmeaDevices), "Discover NMEA devices", "baudrate"_a, "timeout ms"_a);
   
    //---------------------------------------- NetPort --------------------------------------------
    py::class_<NetPort, SysPort, std::shared_ptr<NetPort>>(m, "NetPort")
        .def("discover_isl_devices", py::overload_cast<uint16_t, uint16_t, uint16_t>(&SysPort::discoverIslDevices), "Discover Impact Subsea devices", "pid"_a=0xffff, "pn"_a=0xffff, "sn"_a=0xffff)
        .def("discover_isl_devices", [](NetPort& self, uint16_t pid, uint16_t pn, uint16_t sn, const std::string& ipAddress, uint16_t port, uint_t timeoutMs) {
                                bool_t err = false;
                                self.discoverIslDevices(pid, pn, sn, StringUtils::toIp(ipAddress, err), port, timeoutMs);
                                }, "Discover Impact Subsea devices", "pid"_a, "pn"_a, "sn"_a, "ipV4"_a, "port"_a, "timeout ms"_a);
}
//--------------------------------------------------------------------------------------------------