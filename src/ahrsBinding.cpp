//------------------------------------------ Includes ----------------------------------------------

#include "ahrsBinding.h"
#include "devices/ahrs.h"

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
void PySdk::initAhrs(py::module &m)
{
    py::class_<GyroSensor> gyroSensor(m, "GyroSensor");
    gyroSensor.def_property_readonly("id", [](const GyroSensor& self) { return self.deviceId; }, "The device ID of the gyro sensor. This is the same as the device ID that owns the gyro sensor.")
        .def_property_readonly("sensor_number", [](const GyroSensor& self) { return self.sensorNumber; }, "The sensor number of the gyro sensor.")
        .def_property_readonly("bias", [](const GyroSensor& self) { return self.bias; }, "The bias values for the gyro sensor.")
        .def("update_cal_values", &GyroSensor::updateCalValues, "Update the calibration values within this class.", "bias"_a)
        .def("auto_cal", &GyroSensor::autoCal, "Automatically determine and set the calibration values for the gyro sensor. The sensor can be in any orintation when this is called, but it must be completely still.")
        .def("set_cal", &GyroSensor::setCal, "Set the calibration values for the gyro sensor.", "bias"_a)
        .def_readonly("on_data", &GyroSensor::onData, "Signal emitted when new data is available.")
        .def_readonly("on_cal_change", &GyroSensor::onCalChange, "Signal emitted when the calibration values change.");

    py::class_<Signal<GyroSensor&, const Vector3&>>(gyroSensor, "Signal_onData")
        .def("connect", &Signal<GyroSensor&, const Vector3&>::pyConnect, "Connect to the signal with a callback_function(GyroSensor& gyro, const Vector3& vector)")
        .def("disconnect", &Signal<GyroSensor&, const Vector3&>::pyDisconnect, "Disconnect from the signal");

    //----------------------------------------------------------------------------------------------
    py::class_<AccelSensor> accelSensor(m, "AccelSensor");
    accelSensor.def_property_readonly("id", [](const AccelSensor& self) { return self.deviceId; }, "The device ID of the accelerometer sensor. This is the same as the device ID that owns the accelerometer sensor.")
        .def_property_readonly("sensor_number", [](const AccelSensor& self) { return self.sensorNumber; }, "The sensor number of the accelerometer sensor.")
        .def_property_readonly("bias", [](const AccelSensor& self) { return self.bias; }, "The bias values for the accelerometer sensor.")
        .def_property_readonly("transform", [](const AccelSensor& self) { return self.transform; }, "The transform matrix for the accelerometer sensor.")
        .def("update_cal_values", &AccelSensor::updateCalValues, "Update the calibration values within this class.", "bias"_a, "transform"_a)
        .def("set_cal", &AccelSensor::setCal, "Set the calibration values for the accelerometer sensor.", "bias"_a, "transform"_a)
        .def("start_cal", &AccelSensor::startCal, "Start the calibration process for the accelerometer sensor.", "samplesPerAverage"_a, "maxVariationG"_a, "degFromCardinal"_a)
        .def("stop_cal", &AccelSensor::stopCal, "Stop the calibration process for the accelerometer sensor.", "cancel"_a)
        .def_readonly("on_data", &AccelSensor::onData)
        .def_readonly("on_cal_change", &AccelSensor::onCalChange)
        .def_readonly("on_cal_progress", &AccelSensor::onCalProgress);

    py::class_<Signal<AccelSensor&, const Vector3&>>(accelSensor, "Signal_onData")
        .def("connect", &Signal<AccelSensor&, const Vector3&>::pyConnect, "Connect to the signal with a callback_function(AccelSensor& accel, const Vector3& vector)")
        .def("disconnect", &Signal<AccelSensor&, const Vector3&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<AccelSensor&, const Vector3&, const Matrix3x3&>>(accelSensor, "Signal_onCalChange")
        .def("connect", &Signal<AccelSensor&, const Vector3&, const Matrix3x3&>::pyConnect, "Connect to the signal with a callback_function(AccelSensor& accel, const Vector3& bias, const Matrix3x3& transform)")
        .def("disconnect", &Signal<AccelSensor&, const Vector3&, const Matrix3x3&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<AccelSensor&, Vector3::Axis, const Vector3&, uint_t>>(accelSensor, "Signal_onCalProgress")
        .def("connect", &Signal<AccelSensor&, Vector3::Axis, const Vector3&, uint_t>::pyConnect, "Connect to the signal with a callback_function(AccelSensor& accel, Vector3::Axis axis, const Vector3& vector, uint_t progress)")
        .def("disconnect", &Signal<AccelSensor&, Vector3::Axis, const Vector3&, uint_t>::pyDisconnect, "Disconnect from the signal");

    //----------------------------------------------------------------------------------------------
    py::class_<MagSensor> magSensor(m, "MagSensor");
    magSensor.def_property_readonly("id", [](const MagSensor& self) { return self.deviceId; }, "The device ID of the mag sensor. This is the same as the device ID that owns the mag sensor.")
        .def_property_readonly("sensor_number", [](const MagSensor& self) { return self.sensorNumber; }, "The sensor number of the mag sensor.")
        .def_property_readonly("bias", [](const MagSensor& self) { return self.bias; }, "The bias values for the mag sensor.")
        .def_property_readonly("transform", [](const MagSensor& self) { return self.transform; }, "The transform matrix for the mag sensor.")
        .def("update_cal_values", &MagSensor::updateCalValues, "Update the calibration values within this class.", "bias"_a, "transform"_a)
        .def("set_cal", &MagSensor::setCal, "Set the calibration values for the mag sensor.", "bias"_a, "transform"_a)
        .def("start_cal", &MagSensor::startCal, "Start the calibration process for the mag sensor.", "maxSample"_a=500)
        .def("stop_cal", &MagSensor::stopCal, "Stop the calibration process for the mag sensor.", "cancel"_a, "biasCorrection"_a=nullptr, "transformCorrection"_a=nullptr)
        .def_readonly("on_data", &MagSensor::onData, "Signal emitted when new data is available.")
        .def_readonly("on_cal_change", &MagSensor::onCalChange, "Signal emitted when the calibration values change.")
        .def_readonly("on_cal_progress", &MagSensor::onCalProgress, "Signal emitted when the calibration progress changes.");

    py::class_<Signal<MagSensor&, const Vector3&>>(magSensor, "Signal_onData")
        .def("connect", &Signal<MagSensor&, const Vector3&>::pyConnect, "Connect to the signal with a callback_function(MagSensor& mag, const Vector3& vector)")
        .def("disconnect", &Signal<MagSensor&, const Vector3&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<MagSensor&, const Vector3&, const Matrix3x3&>>(magSensor, "Signal_onCalChange")
        .def("connect", &Signal<MagSensor&, const Vector3&, const Matrix3x3&>::pyConnect, "Connect to the signal with a callback_function(MagSensor& mag, const Vector3& bias, const Matrix3x3& transform)")
        .def("disconnect", &Signal<MagSensor&, const Vector3&, const Matrix3x3&>::pyDisconnect, "Disconnect from the signal");

    py::class_<Signal<MagSensor&, const Vector3&, uint_t, real_t>>(magSensor, "Signal_onCalProgress")
        .def("connect", &Signal<MagSensor&, const Vector3&, uint_t, real_t>::pyConnect, "Connect to the signal with a callback_function(MagSensor& mag, const Vector3& vector, uint_t index, real_t quality)")
        .def("disconnect", &Signal<MagSensor&, const Vector3&, uint_t, real_t>::pyDisconnect, "Disconnect from the signal");

    //----------------------------------------------------------------------------------------------
    py::class_<Ahrs> ahrs(m, "Ahrs");
    ahrs.def_property_readonly("id", [](const Ahrs& self) { return self.deviceId; }, "The device ID of the AHRS. This is the same as the device ID that owns the AHRS.")
        .def("set_heading", &Ahrs::setHeading, "Set the heading of the AHRS.", "heading_rad"_a)
        .def("set_heading_to_mag", &Ahrs::setHeadingToMag, "Set the heading of the AHRS to the magnetic heading.")
        .def("clear_turns_count", &Ahrs::clearTurnsCount, "Clear the turn count of the AHRS.")
        .def_readonly("on_data", &Ahrs::onData, "Signal emitted when new data is available.");

    py::class_<Signal<Ahrs&, uint64_t, const Quaternion&, real_t, real_t>>(ahrs, "Signal_onData")
        .def("connect", &Signal<Ahrs&, uint64_t, const Quaternion&, real_t, real_t>::pyConnect, "Connect to the signal with a callback_function(Ahrs& ahrs, uint64_t timestampUs, const Quaternion& quaternion, real_t magneticHeading, real_t turnCount)")
        .def("disconnect", &Signal<Ahrs&, uint64_t, const Quaternion&, real_t, real_t>::pyDisconnect, "Disconnect from the signal");
}
//--------------------------------------------------------------------------------------------------