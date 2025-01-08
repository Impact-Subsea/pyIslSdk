//------------------------------------------ Includes ----------------------------------------------

#include "isd4000Binding.h"
#include "deviceBinding.h"
#include "devices/isd4000.h"
#include "utils.h"
#include <pybind11/stl.h>

using namespace pybind11::literals;
using namespace IslSdk;

py::dict strOutputSetupToDict(const Isd4000::Settings::StrOutputSetup& setup);
py::dict calCertToDict(const Isd4000::CalCert& cal);
py::list pointArrayToList(const std::array<Point, 10>& points, uint_t length);
Isd4000::Settings dictToSettings(const py::dict& d, const Isd4000::Settings& defualt);
Isd4000::Settings::StrOutputSetup getFromDict(const py::dict& d, const char* key, const Isd4000::Settings::StrOutputSetup& defaultValue);

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
        .def("set_settings", [](Isd4000& self, const py::dict& dict, bool save) { 
            Isd4000::Settings settings = dictToSettings(dict, self.settings);
            self.setSettings(settings, save);
        }, "Set the settings from a dictionary", "settings_dict"_a, "save"_a = false)
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
        .def_property_readonly("depth_script", [](const Isd4000& self) { return self.onDepth; }, "Get the depth script")
        .def_property_readonly("ahrs_script", [](const Isd4000& self) { return self.onAhrs; }, "Get the ahrs script")
        .def_property_readonly("pressure_cal", [](const Isd4000& self) { return self.pressureCal; }, "Get the pressure calibration")
        .def_property_readonly("temperature_cal", [](const Isd4000& self) { return self.temperatureCal; }, "Get the temperature calibration")
        .def_readonly("on_pressure", &Isd4000::onPressure, "Signal emitted when new pressure data is available")
        .def_readonly("on_temperature", &Isd4000::onTemperature, "Signal emitted when new temperature data is available")
        .def_readonly("on_script_data_received", &Isd4000::onScriptDataReceived, "Signal emitted when new script data is available")
        .def_readonly("on_settings_updated", &Isd4000::onSettingsUpdated, "Signal emitted when the settings have been updated")
        .def_readonly("on_pressure_cal_cert", &Isd4000::onPressureCalCert, "Signal emitted when the pressure calibration has been received")
        .def_readonly("on_temperature_cal_cert", &Isd4000::onTemperatureCalCert, "Signal emitted when the temperature calibration has been received");


    py::class_<Signal<Isd4000&, uint64_t, real_t, real_t, real_t>>(isd4000, "SignalOnPressure")
        .def("connect", &Signal<Isd4000&, uint64_t, real_t, real_t, real_t>::pyConnect, "Connect to the signal with a callback_function(isd4000, timeUs, pressure, depth, pressureRaw)")
        .def("disconnect", &Signal<Isd4000&, uint64_t, real_t, real_t, real_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isd4000&, real_t, real_t>>(isd4000, "SignalOnTemperature")
        .def("connect", &Signal<Isd4000&, real_t, real_t>::pyConnect, "Connect to the signal with a callback_function(isd4000, temperature, temperatureRaw)")
        .def("disconnect", &Signal<Isd4000&, real_t, real_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isd4000&>>(isd4000, "SignalOnScriptDataReceived")
        .def("connect", &Signal<Isd4000&>::pyConnect, "Connect to the signal with a callback_function(isd4000)")
        .def("disconnect", &Signal<Isd4000&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isd4000&, bool_t>>(isd4000, "SignalOnSettingsUpdated")
        .def("connect", &Signal<Isd4000&, bool_t>::pyConnect, "Connect to the signal with a callback_function(isd4000, success)")
        .def("disconnect", &Signal<Isd4000&, bool_t>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isd4000&, const Isd4000::PressureCal&>>(isd4000, "SignalOnPressureCalCert")
        .def("connect", &Signal<Isd4000&, const Isd4000::PressureCal&>::pyConnect, "Connect to the signal with a callback_function(isd4000, cal)")
        .def("disconnect", &Signal<Isd4000&, const Isd4000::PressureCal&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Isd4000&, const Isd4000::TemperatureCal&>>(isd4000, "SignalOnTemperatureCalCert")
        .def("connect", &Signal<Isd4000&, const Isd4000::TemperatureCal&>::pyConnect, "Connect to the signal with a callback_function(isd4000, cal)")
        .def("disconnect", &Signal<Isd4000&, const Isd4000::TemperatureCal&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Isd4000::Settings> isd4000Settings(isd4000, "Settings");
    isd4000Settings.def(py::init<>())
        .def(py::init([](const py::dict& d) { return dictToSettings(d, Isd4000::Settings()); }))
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
        .def("defaults", &Isd4000::Settings::defaults, "Set the settings to defaults")
        .def("to_dict", [](const Isd4000::Settings& self)
        {
            py::dict d;
            d["uart_mode"] = py::cast(self.uartMode).attr("name").cast<std::string>();
            d["baudrate"] = self.baudrate;
            d["parity"] = py::cast(self.parity).attr("name").cast<std::string>();
            d["data_bits"] = self.dataBits;
            d["stop_bits"] = py::cast(self.stopBits).attr("name").cast<std::string>();
            d["ahrs_mode"] = self.ahrsMode;
            d["orientation_offset"] = py::make_tuple(self.orientationOffset.w, self.orientationOffset.x, self.orientationOffset.y, self.orientationOffset.z);
            d["heading_offset_rad"] = self.headingOffsetRad;
            d["turns_about"] = py::make_tuple(self.turnsAbout.x, self.turnsAbout.y, self.turnsAbout.z);
            d["turns_about_earth_frame"] = self.turnsAboutEarthFrame;
            d["clr_turn"] = PySdk::customStrToDict(self.clrTurn);
            d["set_heading_2_mag"] = PySdk::customStrToDict(self.setHeading2Mag);
            d["filter_pressure"] = self.filterPressure;
            d["depth_offset"] = self.depthOffset;
            d["pressure_offset"] = self.pressureOffset;
            d["latitude"] = self.latitude;
            d["tare_str"] = PySdk::customStrToDict(self.tareStr);
            d["untare_str"] = PySdk::customStrToDict(self.unTareStr);
            d["depth_str"] = strOutputSetupToDict(self.depthStr);
            d["ahrs_str"] = strOutputSetupToDict(self.ahrsStr);
            return d;
        }, "Convert to dictionary");

    py::class_<Isd4000::Settings::StrOutputSetup>(isd4000Settings, "StrOutputSetup")
        .def_readwrite("str_id", &Isd4000::Settings::StrOutputSetup::strId)
        .def_readwrite("interval_enabled", &Isd4000::Settings::StrOutputSetup::intervalEnabled)
        .def_readwrite("interval_ms", &Isd4000::Settings::StrOutputSetup::intervalMs)
        .def_readwrite("interrogation", &Isd4000::Settings::StrOutputSetup::interrogation)
        .def("to_dict", &strOutputSetupToDict, "Convert to a dictionary");

    py::class_<Isd4000::SensorRates>(isd4000, "SensorRates")
        .def(py::init<>())
        .def_readwrite("pressure", &Isd4000::SensorRates::pressure, "The pressure rate")
        .def_readwrite("ahrs", &Isd4000::SensorRates::ahrs, "The ahrs rate")
        .def_readwrite("gyro", &Isd4000::SensorRates::gyro, "The gyro rate")
        .def_readwrite("accel", &Isd4000::SensorRates::accel, "The accel rate")
        .def_readwrite("mag", &Isd4000::SensorRates::mag, "The mag rate")
        .def_readwrite("temperature", &Isd4000::SensorRates::temperature, "The temperature rate")
        .def("to_dict", [](const Isd4000::SensorRates& self)
        {
            py::dict d;
            d["pressure"] = self.pressure;
            d["ahrs"] = self.ahrs;
            d["gyro"] = self.gyro;
            d["accel"] = self.accel;
            d["mag"] = self.mag;
            d["temperature"] = self.temperature;
            return d;
        }, "Convert to dictionary");

    py::class_<Isd4000::CalCert>(isd4000, "CalCert")
        .def(py::init<>())
        .def_readwrite("year", &Isd4000::CalCert::year, "The year")
        .def_readwrite("month", &Isd4000::CalCert::month, "The month")
        .def_readwrite("day", &Isd4000::CalCert::day, "The day")
        .def_property("cal_points", [](const Isd4000::CalCert& self) -> py::list {
            py::list pyList;
            for (uint_t i = 0; i < self.calPointsLength; i++) {
                pyList.append(self.calPoints[i]);
            }
            return pyList;
        }, [](Isd4000::CalCert &self, py::iterable pyIterable) {
            self.calPointsLength = 0;
            for (auto item : pyIterable) {
                self.calPoints[self.calPointsLength] = item.cast<Point>();
                self.calPointsLength++;
            }
        }, "The calibration points")
        .def_property("verify_points", [](const Isd4000::CalCert& self) -> py::list {
            py::list pyList;
            for (uint_t i = 0; i < self.verifyPointsLength; i++) {
                pyList.append(self.verifyPoints[i]);
            }
            return pyList;
        }, [](Isd4000::CalCert &self, py::iterable pyIterable) {
            self.verifyPointsLength = 0;
            for (auto item : pyIterable) {
                self.verifyPoints[self.verifyPointsLength] = item.cast<Point>();
                self.verifyPointsLength++;
            }
        }, "The verify points")
        .def_readwrite("number", &Isd4000::CalCert::number, "The number")
        .def_readwrite("organisation", &Isd4000::CalCert::organisation, "The organisation")
        .def_readwrite("person", &Isd4000::CalCert::person, "The person")
        .def_readwrite("equipment", &Isd4000::CalCert::equipment, "The equipment")
        .def_readwrite("equipment_sn", &Isd4000::CalCert::equipmentSn, "The equipment serial number")
        .def_readwrite("notes", &Isd4000::CalCert::notes, "The notes")
        .def("to_dict", &calCertToDict, "Convert to dictionary");

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
        .def_readwrite("cal", &Isd4000::PressureCal::cal, "The cal")
        .def("to_dict", [](const Isd4000::PressureCal& self)
        {
            py::dict d;
            d["state"] = py::cast(self.state).attr("name").cast<std::string>();
            d["cal"] = calCertToDict(self.cal);
            return d;
        }, "Convert to dictionary");

    py::class_<Isd4000::TemperatureCal>(isd4000, "TemperatureCal")
        .def(py::init<>())
        .def_readwrite("state", &Isd4000::TemperatureCal::state, "The state")
        .def_readwrite("cal", &Isd4000::TemperatureCal::cal, "The cal")
        .def_readwrite("adc_offset", &Isd4000::TemperatureCal::adcOffset, "The adc offset")
        .def("to_dict", [](const Isd4000::TemperatureCal& self)
        {
            py::dict d;
            d["state"] = py::cast(self.state).attr("name").cast<std::string>();
            d["cal"] = calCertToDict(self.cal);
            d["adc_offset"] = self.adcOffset;
            return d;
        }, "Convert to dictionary");
}
//--------------------------------------------------------------------------------------------------
py::dict strOutputSetupToDict(const Isd4000::Settings::StrOutputSetup& setup)
{
    py::dict d;
    d["str_id"] = setup.strId;
    d["interval_enabled"] = setup.intervalEnabled;
    d["interval_ms"] = setup.intervalMs;
    d["interrogation"] = PySdk::customStrToDict(setup.interrogation);
    return d;
}
//--------------------------------------------------------------------------------------------------
py::dict calCertToDict(const Isd4000::CalCert& cal)
{
    py::dict d;
    d["year"] = cal.year;
    d["month"] = cal.month;
    d["day"] = cal.day;
    d["cal_points"] = pointArrayToList(cal.calPoints, cal.calPointsLength);
    d["verify_points"] = pointArrayToList(cal.verifyPoints, cal.verifyPointsLength);
    d["number"] = cal.number;
    d["organisation"] = cal.organisation;
    d["person"] = cal.person;
    d["equipment"] = cal.equipment;
    d["equipment_sn"] = cal.equipmentSn;
    d["notes"] = cal.notes;
    return d;
}
//--------------------------------------------------------------------------------------------------
py::list pointArrayToList(const std::array<Point, 10>& points, uint_t length)
{
    py::list pyList;
    for (uint_t i = 0; i < length; i++)
    {
        pyList.append(points[i]);
    }
    return pyList;
}
//--------------------------------------------------------------------------------------------------
Isd4000::Settings dictToSettings(const py::dict& d, const Isd4000::Settings& defualt) 
{
    Isd4000::Settings settings;
    settings.uartMode = PySdk::getDictValue(d, "uart_mode", defualt.uartMode);
    settings.baudrate = PySdk::getDictValue(d, "baudrate", defualt.baudrate);
    settings.parity = PySdk::getDictValue(d, "parity", defualt.parity);
    settings.dataBits = PySdk::getDictValue(d, "data_bits", defualt.dataBits);
    settings.stopBits = PySdk::getDictValue(d, "stop_bits", defualt.stopBits);
    settings.ahrsMode = PySdk::getDictValue(d, "ahrs_mode", defualt.ahrsMode);
    settings.orientationOffset = PySdk::getDictValue(d, "orientation_offset", defualt.orientationOffset);
    settings.headingOffsetRad = PySdk::getDictValue(d, "heading_offset_rad", defualt.headingOffsetRad);
    settings.turnsAbout = PySdk::getDictValue(d, "turns_about", defualt.turnsAbout);
    settings.turnsAboutEarthFrame = PySdk::getDictValue(d, "turns_about_earth_frame", defualt.turnsAboutEarthFrame);
    settings.clrTurn = PySdk::getDictValue(d, "clr_turn", defualt.clrTurn);
    settings.setHeading2Mag = PySdk::getDictValue(d, "set_heading2_mag", defualt.setHeading2Mag);
    settings.filterPressure = PySdk::getDictValue(d, "filter_pressure", defualt.filterPressure);
    settings.depthOffset = PySdk::getDictValue(d, "depth_offset", defualt.depthOffset);
    settings.pressureOffset = PySdk::getDictValue(d, "pressure_offset", defualt.pressureOffset);
    settings.latitude = PySdk::getDictValue(d, "latitude", defualt.latitude);
    settings.tareStr = PySdk::getDictValue(d, "tare_str", defualt.tareStr);
    settings.unTareStr = PySdk::getDictValue(d, "untare_str", defualt.unTareStr);
    settings.depthStr = getFromDict(d, "depth_str", defualt.depthStr);
    settings.ahrsStr = getFromDict(d, "ahrs_str", defualt.ahrsStr);
    return settings;
}
//--------------------------------------------------------------------------------------------------
Isd4000::Settings::StrOutputSetup getFromDict(const py::dict& d, const char* key, const Isd4000::Settings::StrOutputSetup& defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::dict>(d[key]))
        {
            py::dict dict = d[key].cast<py::dict>();
            Isd4000::Settings::StrOutputSetup setup;

            if (dict.contains("str_id"))
            {
                if (py::isinstance<py::int_>(dict["str_id"]))
                {
                    setup.strId = dict["str_id"].cast<uint8_t>();
                }
                else
                {
                    throw py::type_error("Type mismatch: Expected int for key[" + std::string(key) + "][str_id], got " + std::string(py::str(d[key].get_type())));
                }
            }
            else
            {
                setup.strId = defaultValue.strId;
            }

            if (dict.contains("interval_enabled"))
            {
                if (py::isinstance<py::bool_>(dict["interval_enabled"]))
                {
                    setup.intervalEnabled = dict["interval_enabled"].cast<bool_t>();
                }
                else
                {
                    throw py::type_error("Type mismatch: Expected bool for key[" + std::string(key) + "][interval_enabled], got " + std::string(py::str(d[key].get_type())));
                }
            }
            else
            {
                setup.intervalEnabled = defaultValue.intervalEnabled;
            }

            if (dict.contains("interval_ms"))
            {
                if (py::isinstance<py::int_>(dict["interval_ms"]))
                {
                    setup.intervalMs = dict["interval_ms"].cast<uint32_t>();
                }
                else
                {
                    throw py::type_error("Type mismatch: Expected int for key[" + std::string(key) + "][interval_ms], got " + std::string(py::str(d[key].get_type())));
                }
            }
            else
            {
                setup.intervalMs = defaultValue.intervalMs;
            }
        }
        else
        {
            throw py::type_error("Type mismatch: Expected dict for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------