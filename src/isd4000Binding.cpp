//------------------------------------------ Includes ----------------------------------------------

#include "isd4000Binding.h"
#include "devices/isd4000.h"
#include <pybind11/stl.h>

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
void PySdk::initIsd4000(py::module &m)
{
    py::class_<Isd4000, Device, std::shared_ptr<Isd4000>> isd4000(m, "Isd4000");
    isd4000.def_readonly("ahrs", &Isd4000::ahrs, "AHRS sensor")
        .def_readonly("gyro", &Isd4000::gyro, "Gyro sensor")
        .def_readonly("accel", &Isd4000::accel, "Accelerometer sensor")
        .def_readonly("mag", &Isd4000::mag, "Magnetometer sensor")
        .def("set_sensor_rates", &Isd4000::setSensorRates, "Set the sensor rates", "rate"_a)
        .def("set_settings", &Isd4000::setSettings, "Set the settings", "settings"_a, "save"_a)
        .def("set_depth_script", &Isd4000::setDepthScript, "Set the depth script", "name"_a, "code"_a)
        .def("set_ahrs_script", &Isd4000::setAhrsScript, "Set the ahrs script", "name"_a, "code"_a)
        .def("get_scripts", &Isd4000::getScripts, "Get the scripts")
        .def("set_pressure_cal_cert", &Isd4000::setPressureCalCert, "Set the pressure calibration", "cert"_a)
        .def("set_temperature_cal_cert", &Isd4000::setTemperatureCalCert, "Set the temperature calibration", "cert"_a)
        .def("get_cal", &Isd4000::getCal, "Get the cal", "pressure"_a = false, "temperature"_a = false)
        .def("pressure_cal_valid", &Isd4000::pressureCalValid, "True if the pressure calibration is valid")
        .def("temperature_cal_valid", &Isd4000::temperatureCalValid, "True if the temperature calibration is valid")
        .def("measure_now", &Isd4000::measureNow, "Trigger a measurement")
        .def("load_config", [](const Isd4000& self, const std::string& filename)
        { 
            Device::Info info;
            Isd4000::Settings settings;
            DeviceScript script0;
            DeviceScript script1;
            Isd4000::AhrsCal cal;
            Isd4000::PressureCal pCal;
            Isd4000::TemperatureCal tCal;
            if (self.loadConfig(filename, &info, &settings, &script0, &script1, &cal, &pCal, &tCal))
            {
                return py::make_tuple(py::cast(info), py::cast(settings), py::cast(script0), py::cast(script1), py::cast(cal), py::cast(pCal), py::cast(tCal));
            }
            else
            {
                return py::make_tuple();
            }
        }, "Load the config")
        .def("has_ahrs", &Isd4000::hasAhrs, "True if the device has the AHRS licence")
        .def("max_pressure_rating_bar", &Isd4000::maxPressureRatingBar, "The maximum pressure rating in Bar")
        .def_property_readonly("settings", [](const Isd4000& self) { return self.settings; }, "Get the settings")
        .def_property_readonly("sensor_rates", [](const Isd4000& self) { return self.sensorRates; }, "Get the sensor rates")
        .def_property_readonly("hard_coded_depth_output_strings", [](const Isd4000& self) { return self.hardCodedDepthOutputStrings; }, "Get the hard coded depth output strings")
        .def_property_readonly("hard_coded_ahrs_output_strings", [](const Isd4000& self) { return self.hardCodedAhrsOutputStrings; }, "Get the hard coded ahrs output strings")
        .def_property_readonly("script_vars", [](const Isd4000& self) { return self.scriptVars; }, "Get the script variables")
        .def_property_readonly("on_depth", [](const Isd4000& self) { return self.onDepth; }, "Get the depth script")
        .def_property_readonly("on_ahrs", [](const Isd4000& self) { return self.onAhrs; }, "Get the ahrs script")
        .def_property_readonly("pressure_cal", [](const Isd4000& self) { return self.pressureCal; }, "Get the pressure calibration")
        .def_property_readonly("temperature_cal", [](const Isd4000& self) { return self.temperatureCal; }, "Get the temperature calibration")
        .def_readonly("on_pressure", &Isd4000::onPressure, "Signal emitted when new pressure data is available")
        .def_readonly("on_temperature", &Isd4000::onTemperature, "Signal emitted when new temperature data is available")
        .def_readonly("on_script_data_received", &Isd4000::onScriptDataReceived, "Signal emitted when new script data is available")
        .def_readonly("on_settings_updated", &Isd4000::onSettingsUpdated, "Signal emitted when the settings have been updated")
        .def_readonly("on_pressure_cal_cert", &Isd4000::onPressureCalCert, "Signal emitted when the pressure calibration has been received")
        .def_readonly("on_temperature_cal_cert", &Isd4000::onTemperatureCalCert, "Signal emitted when the temperature calibration has been received");


    py::class_<Signal<Isd4000&, uint64_t, real_t, real_t, real_t>>(isd4000, "Signal_onPressure")
        .def("connect", &Signal<Isd4000&, uint64_t, real_t, real_t, real_t>::pyConnect, "Connect to the signal with a callback_function(isd4000, timeUs, pressure, depth, pressureRaw)")
        .def("disconnect", &Signal<Isd4000&, uint64_t, real_t, real_t, real_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isd4000&, real_t, real_t>>(isd4000, "Signal_onTemperature")
        .def("connect", &Signal<Isd4000&, real_t, real_t>::pyConnect, "Connect to the signal with a callback_function(isd4000, temperature, temperatureRaw)")
        .def("disconnect", &Signal<Isd4000&, real_t, real_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isd4000&>>(isd4000, "Signal_onScriptDataReceived")
        .def("connect", &Signal<Isd4000&>::pyConnect, "Connect to the signal with a callback_function(isd4000)")
        .def("disconnect", &Signal<Isd4000&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isd4000&, bool_t>>(isd4000, "Signal_onSettingsUpdated")
        .def("connect", &Signal<Isd4000&, bool_t>::pyConnect, "Connect to the signal with a callback_function(isd4000, success)")
        .def("disconnect", &Signal<Isd4000&, bool_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isd4000&, const Isd4000::PressureCal&>>(isd4000, "Signal_onPressureCalCert")
        .def("connect", &Signal<Isd4000&, const Isd4000::PressureCal&>::pyConnect, "Connect to the signal with a callback_function(isd4000, cal)")
        .def("disconnect", &Signal<Isd4000&, const Isd4000::PressureCal&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isd4000&, const Isd4000::TemperatureCal&>>(isd4000, "Signal_onTemperatureCalCert")
        .def("connect", &Signal<Isd4000&, const Isd4000::TemperatureCal&>::pyConnect, "Connect to the signal with a callback_function(isd4000, cal)")
        .def("disconnect", &Signal<Isd4000&, const Isd4000::TemperatureCal&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Isd4000::Settings> isd4000Settings(isd4000, "Settings");
    isd4000Settings.def(py::init<>())
        .def_readwrite("uart_mode", &Isd4000::Settings::uartMode, "The serial port mode")
        .def_readwrite("baudrate", &Isd4000::Settings::baudrate, "The serial port baudrate")
        .def_readwrite("parity", &Isd4000::Settings::parity, "The serial port parity")
        .def_readwrite("data_bits", &Isd4000::Settings::dataBits, "The serial port data bits")
        .def_readwrite("stop_bits", &Isd4000::Settings::stopBits, "The serial port stop bits")
        .def_readwrite("ahrs_mode", &Isd4000::Settings::ahrsMode, "The ahrs mode")
        .def_readwrite("orientation_offset", &Isd4000::Settings::orientationOffset, "The orientation offset")
        .def_readwrite("heading_offset_rad", &Isd4000::Settings::headingOffsetRad, "The heading offset in radians")
        .def_readwrite("turns_about", &Isd4000::Settings::turnsAbout, "The turns about vector")
        .def_readwrite("turns_about_earth_frame", &Isd4000::Settings::turnsAboutEarthFrame, "True if the turns about vector is in the earth frame")
        .def_readwrite("clr_turn", &Isd4000::Settings::clrTurn, "The clear turns string")
        .def_readwrite("set_heading_2_mag", &Isd4000::Settings::setHeading2Mag, "The set heading to mag string")
        .def_readwrite("filter_pressure", &Isd4000::Settings::filterPressure, "True if the pressure is filtered")
        .def_readwrite("depth_offset", &Isd4000::Settings::depthOffset, "The depth offset")
        .def_readwrite("pressure_offset", &Isd4000::Settings::pressureOffset, "The pressure offset")
        .def_readwrite("latitude", &Isd4000::Settings::latitude, "The latitude")
        .def_readwrite("tare_str", &Isd4000::Settings::tareStr, "The tare string")
        .def_readwrite("untare_str", &Isd4000::Settings::unTareStr, "The untare string")
        .def_readwrite("depth_str", &Isd4000::Settings::depthStr, "The depth string")
        .def_readwrite("ahrs_str", &Isd4000::Settings::ahrsStr, "The ahrs string")
        .def("defaults", &Isd4000::Settings::defaults, "Set the settings to defaults");

    py::class_<Isd4000::Settings::StrOutputSetup>(isd4000Settings, "StrOutputSetup")
        .def_readwrite("str_id", &Isd4000::Settings::StrOutputSetup::strId)
        .def_readwrite("interval_enabled", &Isd4000::Settings::StrOutputSetup::intervalEnabled)
        .def_readwrite("interval_ms", &Isd4000::Settings::StrOutputSetup::intervalMs)
        .def_readwrite("interrogation", &Isd4000::Settings::StrOutputSetup::interrogation);

    py::class_<Isd4000::SensorRates>(isd4000, "SensorRates")
        .def(py::init<>())
        .def_readwrite("pressure", &Isd4000::SensorRates::pressure, "The pressure rate")
        .def_readwrite("ahrs", &Isd4000::SensorRates::ahrs, "The ahrs rate")
        .def_readwrite("gyro", &Isd4000::SensorRates::gyro, "The gyro rate")
        .def_readwrite("accel", &Isd4000::SensorRates::accel, "The accel rate")
        .def_readwrite("mag", &Isd4000::SensorRates::mag, "The mag rate")
        .def_readwrite("temperature", &Isd4000::SensorRates::temperature, "The temperature rate");

    py::class_<Isd4000::CalCert>(isd4000, "CalCert")
        .def(py::init<>())
        .def_readwrite("year", &Isd4000::CalCert::year, "The year")
        .def_readwrite("month", &Isd4000::CalCert::month, "The month")
        .def_readwrite("day", &Isd4000::CalCert::day, "The day")
        .def_readwrite("cal_points_length", &Isd4000::CalCert::calPointsLength, "The cal points length")
        .def_readwrite("verify_points_length", &Isd4000::CalCert::verifyPointsLength, "The verify points length")
        .def_readwrite("cal_points", &Isd4000::CalCert::calPoints, "The cal points")
        .def_readwrite("verify_points", &Isd4000::CalCert::verifyPoints, "The verify points")
        .def_readwrite("number", &Isd4000::CalCert::number, "The number")
        .def_readwrite("organisation", &Isd4000::CalCert::organisation, "The organisation")
        .def_readwrite("person", &Isd4000::CalCert::person, "The person")
        .def_readwrite("equipment", &Isd4000::CalCert::equipment, "The equipment")
        .def_readwrite("equipment_sn", &Isd4000::CalCert::equipmentSn, "The equipment serial number")
        .def_readwrite("notes", &Isd4000::CalCert::notes, "The notes");

    py::class_<Isd4000::AhrsCal>(isd4000, "AhrsCal")
        .def(py::init<>())
        .def_readwrite("gyro_bias", &Isd4000::AhrsCal::gyroBias, "The gyro bias")
        .def_readwrite("accel_bias", &Isd4000::AhrsCal::accelBias, "The accel bias")
        .def_readwrite("mag_bias", &Isd4000::AhrsCal::magBias, "The mag bias")
        .def_readwrite("accel_transform", &Isd4000::AhrsCal::accelTransform, "The accel transform")
        .def_readwrite("mag_transform", &Isd4000::AhrsCal::magTransform, "The mag transform");

    py::class_<Isd4000::PressureCal>(isd4000, "PressureCal")
        .def(py::init<>())
        .def_readwrite("state", &Isd4000::PressureCal::state, "The state")
        .def_readwrite("cal", &Isd4000::PressureCal::cal, "The cal");

    py::class_<Isd4000::TemperatureCal>(isd4000, "TemperatureCal")
        .def(py::init<>())
        .def_readwrite("state", &Isd4000::TemperatureCal::state, "The state")
        .def_readwrite("cal", &Isd4000::TemperatureCal::cal, "The cal")
        .def_readwrite("adc_offset", &Isd4000::TemperatureCal::adcOffset, "The adc offset");

    py::class_<Isd4000::PressureSenorInfo>(isd4000, "PressureSenorInfo") 
        .def_readonly("min_pressure", &Isd4000::PressureSenorInfo::minPressure, "The minimum pressure")
        .def_readonly("max_pressure", &Isd4000::PressureSenorInfo::maxPressure, "The maximum pressure");
}
//--------------------------------------------------------------------------------------------------