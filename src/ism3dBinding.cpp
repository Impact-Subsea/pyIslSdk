//------------------------------------------ Includes ----------------------------------------------

#include "ism3dBinding.h"
#include "devices/ism3d.h"
#include <pybind11/stl.h>

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
void PySdk::initIsm3d(py::module &m)
{
    py::class_<Ism3d, Device, std::shared_ptr<Ism3d>> ism3d(m, "Ism3d");
    ism3d.def(py::init<const Device::Info&>())
        .def_readonly("ahrs", &Ism3d::ahrs, "Class to manage AHRS data")
        .def_readonly("gyro", &Ism3d::gyro, "Class to manage gyro data")
        .def_readonly("gyro_sec", &Ism3d::gyroSec, "Class to manage backup gyro data")
        .def_readonly("accel", &Ism3d::accel, "Class to manage accelerometer data")
        .def_readonly("accel_sec", &Ism3d::accelSec, "Class to manage backup accelerometer data")
        .def_readonly("mag", &Ism3d::mag, "Class to manage magnetometer data")
        .def_property_readonly("settings", [](const Ism3d& self) { return self.settings; }, "The settings")
        .def_property_readonly("sensor_rates", [](const Ism3d& self) { return self.sensorRates; }, "The sensor rates")
        .def_property_readonly("hard_coded_ahrs_output_strings", [](const Ism3d& self) { return self.hardCodedAhrsOutputStrings; }, "The hard coded AHRS output strings")
        .def_property_readonly("script_vars", [](const Ism3d& self) { return self.scriptVars; }, "The script variables")
        .def_property_readonly("on_ahrs", [](const Ism3d& self) { return self.onAhrs; }, "The AHRS data signal")
        .def("set_sensor_rates", &Ism3d::setSensorRates, "Set the sensor rates", "sensors"_a)
        .def("set_settings", &Ism3d::setSettings, "Set the settings", "settings"_a, "save"_a)
        .def("set_ahrs_script", &Ism3d::setAhrsScript, "Set the AHRS script", "name"_a, "code"_a)
        .def("get_scripts", &Ism3d::getScripts, "Get the scripts")
        .def("save_config", &Ism3d::saveConfig, "Save the configuration", "fileName"_a)
        .def_static("load_config", [](const Ism3d& self, const std::string& filename)
        {
            Device::Info info;
            Ism3d::Settings settings;
            DeviceScript script;
            Ism3d::AhrsCal cal;
            if (self.loadConfig(filename, &info, &settings, &script, &cal))
            {
                return py::make_tuple(py::cast(info), py::cast(settings), py::cast(script), py::cast(cal));
            }
            else
            {
                return py::make_tuple();
            }
        }, "Load the configuration")
        .def_readonly("on_script_data_received", &Ism3d::onScriptDataReceived)
        .def_readonly("on_settings_updated", &Ism3d::onSettingsUpdated);

    py::class_<Signal<Ism3d&>>(ism3d, "Signal_onScriptDataReceived")
        .def("connect", &Signal<Ism3d&>::pyConnect, "Connect to the signal with a callback_function(ism3d)")
        .def("disconnect", &Signal<Ism3d&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Ism3d&, bool_t>>(ism3d, "Signal_onSettingsUpdated")
        .def("connect", &Signal<Ism3d&, bool_t>::pyConnect, "Connect to the signal with a callback_function(ism3d, success)")
        .def("disconnect", &Signal<Ism3d&, bool_t>::pyDisconnect, "Disconnect a callback function from the signal");
    

    py::class_<Ism3d::Settings> ism3dSettings(ism3d, "Settings");
    ism3dSettings.def(py::init<>())
        .def_readwrite("uart_mode", &Ism3d::Settings::uartMode, "The serial port mode")
        .def_readwrite("baudrate", &Ism3d::Settings::baudrate, "The serial port baudrate")
        .def_readwrite("parity", &Ism3d::Settings::parity, "The serial port parity")
        .def_readwrite("data_bits", &Ism3d::Settings::dataBits, "The serial port word length")
        .def_readwrite("stop_bits", &Ism3d::Settings::stopBits, "The serial port stop bits")
        .def_readwrite("ahrs_mode", &Ism3d::Settings::ahrsMode, "The AHRS mode")
        .def_readwrite("orientation_offset", &Ism3d::Settings::orientationOffset, "The orientation offset")
        .def_readwrite("heading_offset_rad", &Ism3d::Settings::headingOffsetRad, "The heading offset in radians")
        .def_readwrite("turns_about", &Ism3d::Settings::turnsAbout, "The turns about vector")
        .def_readwrite("turns_about_earth_frame", &Ism3d::Settings::turnsAboutEarthFrame, "The turns about earth frame")
        .def_readwrite("clr_turn", &Ism3d::Settings::clrTurn, "The clear turns string")
        .def_readwrite("set_heading2_mag", &Ism3d::Settings::setHeading2Mag, "The set heading to mag string")
        .def_readwrite("ahrs_str", &Ism3d::Settings::ahrsStr, "The AHRS string")
        .def("defaults", &Ism3d::Settings::defaults, "Set the settings to defaults");

    py::class_<Ism3d::Settings::StrOutputSetup>(ism3dSettings, "StrOutputSetup")
        .def_readwrite("str_id", &Ism3d::Settings::StrOutputSetup::strId)
        .def_readwrite("interval_enabled", &Ism3d::Settings::StrOutputSetup::intervalEnabled)
        .def_readwrite("interval_ms", &Ism3d::Settings::StrOutputSetup::intervalMs)
        .def_readwrite("interrogation", &Ism3d::Settings::StrOutputSetup::interrogation);

    py::class_<Ism3d::SensorRates>(ism3d, "SensorRates")
        .def(py::init<>())
        .def_readwrite("ahrs", &Ism3d::SensorRates::ahrs, "The AHRS interval")
        .def_readwrite("gyro", &Ism3d::SensorRates::gyro, "The gyro interval")
        .def_readwrite("accel", &Ism3d::SensorRates::accel, "The accel interval")
        .def_readwrite("mag", &Ism3d::SensorRates::mag, "The mag interval");

    py::class_<Ism3d::AhrsCal>(ism3d, "AhrsCal")
        .def(py::init<>())
        .def_readwrite("gyro_bias", &Ism3d::AhrsCal::gyroBias, "The gyro bias")
        .def_readwrite("accel_bias", &Ism3d::AhrsCal::accelBias, "The accel bias")
        .def_readwrite("mag_bias", &Ism3d::AhrsCal::magBias, "The mag bias")
        .def_readwrite("accel_transform", &Ism3d::AhrsCal::accelTransform, "The accel transform")
        .def_readwrite("mag_transform", &Ism3d::AhrsCal::magTransform, "The mag transform")
        .def_readwrite("gyro_bias_sec", &Ism3d::AhrsCal::gyroBiasSec, "The gyro bias sec")
        .def_readwrite("accel_bias_sec", &Ism3d::AhrsCal::accelBiasSec, "The accel bias sec")
        .def_readwrite("accel_transform_sec", &Ism3d::AhrsCal::accelTransformSec, "The accel transform sec");
}
//--------------------------------------------------------------------------------------------------