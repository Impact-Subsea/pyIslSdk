//------------------------------------------ Includes ----------------------------------------------

#include "isa500Binding.h"
#include "devices/isa500.h"
#include <pybind11/stl.h>

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
void PySdk::initIsa500(py::module &m)
{
    py::class_<Isa500, Device, std::shared_ptr<Isa500>> isa500(m, "Isa500");
    isa500.def_readonly("ahrs", &Isa500::ahrs, "AHRS sensor")
        .def_readonly("gyro", &Isa500::gyro, "Gyro sensor")
        .def_readonly("accel", &Isa500::accel, "Accelerometer sensor")
        .def_readonly("mag", &Isa500::mag, "Magnetometer sensor")
        .def("set_sensor_rates", &Isa500::setSensorRates, "Set the sensor rates", "rate"_a)
        .def("set_settings", &Isa500::setSettings, "Set the settings", "settings"_a, "save"_a)
        .def("ping_now", &Isa500::pingNow, "Ping now")
        .def("set_echo_gram", &Isa500::setEchoGram, "Set the echo gram", "data point count"_a)
        .def("set_ping_script", &Isa500::setPingScript, "Set the ping script", "name"_a, "code"_a)
        .def("set_ahrs_script", &Isa500::setAhrsScript, "Set the ahrs script", "name"_a, "code"_a)
        .def("get_scripts", &Isa500::getScripts, "Get the scripts")
        .def("load_config", [](const Isa500& self, const std::string& filename) { 
            Device::Info info;
            Isa500::Settings settings;
            DeviceScript script0;
            DeviceScript script1;
            Isa500::AhrsCal cal;
            if (self.loadConfig(filename, &info, &settings, &script0, &script1, &cal))
            {
                return py::make_tuple(py::cast(info), py::cast(settings), py::cast(script0), py::cast(script1), py::cast(cal));
            }
            else
            {
                return py::make_tuple();
            }}, "Load the config")
        .def("has_ahrs", &Isa500::hasAhrs, "True if the device has the AHRS licence")
        .def("has_echo_gram", &Isa500::hasEchoGram, "True if the device has the echogram licence")
        .def("has_fmd", &Isa500::hasFmd, "True if the device has the FMD licence")
        .def("has_right_angle_transducer", &Isa500::hasRightAngleTransducer, "True if the device has a right angle transducer")
        .def("has_current_loop", &Isa500::hasCurrentLoop, "True if the device has current loop analogue output")
        .def_property_readonly("settings", [](const Isa500& self) { return self.settings; }, "Get the settings")
        .def_property_readonly("sensor_rates", [](const Isa500& self) { return self.sensorRates; }, "Get the sensor rates")
        .def_property_readonly("hard_coded_ping_output_strings", [](const Isa500& self) { return self.hardCodedPingOutputStrings; }, "Get the hard coded ping output strings")
        .def_property_readonly("hard_coded_ahrs_output_strings", [](const Isa500& self) { return self.hardCodedAhrsOutputStrings; }, "Get the hard coded AHRS output strings")
        .def_property_readonly("script_vars", [](const Isa500& self) { return self.scriptVars; }, "Get the script variables")
        .def_property_readonly("on_ping", [](const Isa500& self) { return self.onPing; }, "Get the ping script")
        .def_property_readonly("on_ahrs", [](const Isa500& self) { return self.onAhrs; }, "Get the AHRS script")
        .def_readonly("on_echo", &Isa500::onEcho, "Signal emitted when new echo data is available")
        .def_readonly("on_echogram_data", &Isa500::onEchogramData, "Signal emitted when new echogram data is available")
        .def_readonly("on_temperature", &Isa500::onTemperature, "Signal emitted when new temperature data is available")
        .def_readonly("on_voltage", &Isa500::onVoltage, "Signal emitted when new voltage data is available")
        .def_readonly("on_trigger", &Isa500::onTrigger, "Signal emitted when a trigger event occurs")
        .def_readonly("on_script_data_received", &Isa500::onScriptDataReceived, "Signal emitted when new script data is available")
        .def_readonly("on_settings_updated", &Isa500::onSettingsUpdated, "Signal emitted when settings have updated");
        

    py::class_<Signal<Isa500&, uint64_t, uint_t, uint_t, const std::vector<Isa500::Echo>&>>(isa500, "Signal_onEcho")
        .def("connect", &Signal<Isa500&, uint64_t, uint_t, uint_t, const std::vector<Isa500::Echo>&>::pyConnect, "Connect to the signal with a callback_function(isa500, time us, selected echo index, total echo count, echos)")
        .def("disconnect", &Signal<Isa500&, uint64_t, uint_t, uint_t, const std::vector<Isa500::Echo>&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isa500&, const std::vector<uint8_t>&>>(isa500, "Signal_onEchogramData")
        .def("connect", &Signal<Isa500&, const std::vector<uint8_t>&>::pyConnect, "Connect to the signal with a callback_function(isa500, echogram)")
        .def("disconnect", &Signal<Isa500&, const std::vector<uint8_t>&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isa500&, real_t>>(isa500, "Signal_onTemperature")
        .def("connect", &Signal<Isa500&, real_t>::pyConnect, "Connect to the signal with a callback_function(isa500, temperature)")
        .def("disconnect", &Signal<Isa500&, real_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isa500&, bool_t>>(isa500, "Signal_onTrigger")
        .def("connect", &Signal<Isa500&, bool_t>::pyConnect, "Connect to the signal with a callback_function(isa500, edge)")
        .def("disconnect", &Signal<Isa500&, bool_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isa500&>>(isa500, "Signal_onScriptDataReceived")
        .def("connect", &Signal<Isa500&>::pyConnect, "Connect to the signal with a callback_function(isa500)")
        .def("disconnect", &Signal<Isa500&>::pyDisconnect, "Disconnect a callback function from the signal");

    

    py::class_<Isa500::AhrsCal>(isa500, "AhrsCal")
        .def(py::init<>())
        .def_readwrite("gyro_bias", &Isa500::AhrsCal::gyroBias, "Gyro bias corrections in degress per second")
        .def_readwrite("accel_bias", &Isa500::AhrsCal::accelBias, "Accel bias corrections in G")
        .def_readwrite("mag_bias", &Isa500::AhrsCal::magBias, "Mag bias corrections in uT")
        .def_readwrite("accel_transform", &Isa500::AhrsCal::accelTransform, "Transformation matrix for accelerometer")
        .def_readwrite("mag_transform", &Isa500::AhrsCal::magTransform, "Transformation matrix for magnetometer");

    py::class_<Isa500::SensorRates>(isa500, "SensorRates")
        .def(py::init<>())
        .def_readwrite("ping", &Isa500::SensorRates::ping, "Interval in milliseconds between pings. Zero means no pings.")
        .def_readwrite("ahrs", &Isa500::SensorRates::ahrs, "Interval in milliseconds between AHRS data. Zero means no AHRS data.")
        .def_readwrite("gyro", &Isa500::SensorRates::gyro, "Interval in milliseconds between gyro data. Zero means no gyro data.")
        .def_readwrite("accel", &Isa500::SensorRates::accel, "Interval in milliseconds between accelerometer data. Zero means no accelerometer data.")
        .def_readwrite("mag", &Isa500::SensorRates::mag, "Interval in milliseconds between magnetometer data. Zero means no magnetometer data.")
        .def_readwrite("temperature", &Isa500::SensorRates::temperature, "Interval in milliseconds between temperature data. Zero means no temperature data.")
        .def_readwrite("voltage", &Isa500::SensorRates::voltage, "Interval in milliseconds between voltage data. Zero means no voltage data.");

    py::class_<Isa500::Echo>(isa500, "Echo")
        .def(py::init<>())
        .def_readwrite("total_tof", &Isa500::Echo::totalTof, "Total time of flight in seconds to the target and back")
        .def_readwrite("correlation", &Isa500::Echo::correlation, "How well the received echo correlates 0 to 1")
        .def_readwrite("signal_energy", &Isa500::Echo::signalEnergy, "Normalised energy level of the echo 0 to 1");

    py::class_<Isa500::Settings> isa500Settings(isa500, "Settings");
    isa500Settings.def(py::init<>())
        .def_readwrite("uart_mode", &Isa500::Settings::uartMode, "Serial port mode")
        .def_readwrite("baudrate", &Isa500::Settings::baudrate, "Serial port baudrate")
        .def_readwrite("parity", &Isa500::Settings::parity, "Serial parity")
        .def_readwrite("data_bits", &Isa500::Settings::dataBits, "Serial word length 5 to 8 bits")
        .def_readwrite("stop_bits", &Isa500::Settings::stopBits, "Serial stop bits")
        .def_readwrite("ahrs_mode", &Isa500::Settings::ahrsMode, "If bit zero is 1 use inertial mode. 0 is mag slave mode")
        .def_readwrite("orientation_offset", &Isa500::Settings::orientationOffset, "Heading, pitch and roll offsets (or down and forward vectors) expressed as a quaternion")
        .def_readwrite("heading_offset_rad", &Isa500::Settings::headingOffsetRad, "Offset in radians to add to the heading. Typically use for magnetic declination")
        .def_readwrite("turns_about", &Isa500::Settings::turnsAbout, "A vector representing the axis which turn are measured about")
        .def_readwrite("turns_about_earth_frame", &Isa500::Settings::turnsAboutEarthFrame, "If true the \"turnsAbout\" vector is referenced to the earth frame. False is sensor frame")
        .def_readwrite("clr_turn", &Isa500::Settings::clrTurn, "The turns clearing string")
        .def_readwrite("set_heading2_mag", &Isa500::Settings::setHeading2Mag, "A string to set the heading to magnetometer heading")
        .def_readwrite("multi_echo_limit", &Isa500::Settings::multiEchoLimit, "Sets the maximum multi echo limit, range is 0 to 100")
        .def_readwrite("frequency", &Isa500::Settings::frequency, "Frequency of operation in Hertz from 50000 to 700000")
        .def_readwrite("tx_pulse_width_us", &Isa500::Settings::txPulseWidthUs, "Length of the transmit pulse in microseconds ranging from 0 to 500")
        .def_readwrite("tx_pulse_amplitude", &Isa500::Settings::txPulseAmplitude, "Amplitude of the transmit pulse as a percentage ranging from 0 to 100")
        .def_readwrite("echo_analyse_mode", &Isa500::Settings::echoAnalyseMode, "Selects which echo to report back as the chosen one")
        .def_readwrite("xc_threashold_low", &Isa500::Settings::xcThreasholdLow, "When the return cross correlated signal level drops below this value the end of an echo pulse is realised. Value ranges from 0 to 1 default is 0.4")
        .def_readwrite("xc_threashold_high", &Isa500::Settings::xcThreasholdHigh, "When the return cross correlated signal level rises above this value the start of an echo pulse is realised. Value ranges from 0 to 1 default is 0.5")
        .def_readwrite("energy_threashold", &Isa500::Settings::energyThreashold, "Minimum enery an echo must have to be reported. Range is 0 to 1")
        .def_readwrite("speed_of_sound", &Isa500::Settings::speedOfSound, "Speed of sound in metres per second")
        .def_readwrite("min_range", &Isa500::Settings::minRange, "Minimum range in metres. Distance is one way, transducer to target")
        .def_readwrite("max_range", &Isa500::Settings::maxRange, "Maximum range in metres. Upper limit is 300, distance is one way, transducer to target")
        .def_readwrite("distance_offset", &Isa500::Settings::distanceOffset, "Offset + or - in metres to add to the final reading")
        .def_readwrite("use_tilt_correction", &Isa500::Settings::useTiltCorrection, "If true the echo range to target will be trigonometrically corrected for pitch and roll")
        .def_readwrite("use_max_value_on_no_return", &Isa500::Settings::useMaxValueOnNoReturn, "If no echo is detected then use the maximum range value as the reading for outputted strings")
        .def_readwrite("ana_mode", &Isa500::Settings::anaMode, "Mode of the analogue output")
        .def_readwrite("a_out_min_range", &Isa500::Settings::aOutMinRange, "Value in metres. \"aOutMinRange\" and \"aOutMinVal\" define a point. e.g 3 metres = 3 volt")
        .def_readwrite("a_out_max_range", &Isa500::Settings::aOutMaxRange, "Value in meteres. \"aOutMaxRange\" and \"aOutMaxVal\" define another point. e.g 10 metres = 10 volt. These 2 points define a straight line which relates range to output value")
        .def_readwrite("a_out_min_val", &Isa500::Settings::aOutMinVal, "Volts or milliamps depending on mode")
        .def_readwrite("a_out_max_val", &Isa500::Settings::aOutMaxVal, "Volts or milliamps depending on mode")
        .def_readwrite("ping_str", &Isa500::Settings::pingStr, "Custom interrogation string")
        .def_readwrite("ahrs_str", &Isa500::Settings::pingStr, "Custom interrogation string")
        .def("defaults", &Isa500::Settings::defaults, "Sets the default values");

    py::enum_<Isa500::AnalogueOutMode>(isa500, "AnalogueOutMode")
        .value("Off", Isa500::AnalogueOutMode::None)
        .value("Voltage", Isa500::AnalogueOutMode::Voltage)
        .value("Current", Isa500::AnalogueOutMode::Current);

    py::enum_<Isa500::EchoAnalyseMode>(isa500, "EchoAnalyseMode")
        .value("First", Isa500::EchoAnalyseMode::First)
        .value("Strongest", Isa500::EchoAnalyseMode::Strongest)
        .value("Tracking", Isa500::EchoAnalyseMode::Tracking);

    py::class_<Isa500::Settings::StrOutputSetup>(isa500Settings, "StrOutputSetup")
        .def_readwrite("str_id", &Isa500::Settings::StrOutputSetup::strId)
        .def_readwrite("interval_enabled", &Isa500::Settings::StrOutputSetup::intervalEnabled)
        .def_readwrite("interval_ms", &Isa500::Settings::StrOutputSetup::intervalMs)
        .def_readwrite("trigger_enabled", &Isa500::Settings::StrOutputSetup::triggerEnabled)
        .def_readwrite("trigger_edge", &Isa500::Settings::StrOutputSetup::triggerEdge)
        .def_readwrite("interrogation", &Isa500::Settings::StrOutputSetup::interrogation);
}
//--------------------------------------------------------------------------------------------------