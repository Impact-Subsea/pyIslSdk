//------------------------------------------ Includes ----------------------------------------------

#include "typesBinding.h"
#include "types/sdkTypes.h"
#include "maths/vector3.h"
#include "maths/matrix3x3.h"
#include "maths/quaternion.h"
#include "utils/stringUtils.h"

#include <pybind11/stl.h>
#include <pybind11/operators.h>

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------

void PySdk::initTypes(py::module &m)
{
    py::class_<ConstBuffer>(m, "ConstArray", py::buffer_protocol())
        .def_readonly("size", &ConstBuffer::size, "The size")
        .def_buffer([](ConstBuffer& self) -> py::buffer_info 
        {
            return py::buffer_info(
                const_cast<uint8_t*>(self.data),
                sizeof(uint8_t),
                py::format_descriptor<uint8_t>::format(),
                1,
                { self.size},
                { sizeof(uint8_t) },
                true
            );
        })
        .def_property_readonly("__array_interface__", [](ConstBuffer& self) -> py::dict
        {
            py::dict d;
            d["shape"] = self.size;
            d["typestr"] = py::format_descriptor<uint8_t>::format(),
            d["data"] = py::make_tuple( reinterpret_cast<size_t>(self.data), true);
            d["strides"] = sizeof(uint8_t);
            d["version"] = 3;
            return d;
        }, "Return a NumPy array with __array_interface__ support")
        .def("to_bytes", [](ConstBuffer& self) -> py::bytes
        {
            return py::bytes(reinterpret_cast<const char*>(self.data), self.size);
        }, "Return the image as a bytes object");

    py::class_<ConnectionMeta>(m, "ConnectionMeta")
        .def(py::init<uint32_t>(), "Constructor with baudrate")
        .def(py::init<uint32_t, uint16_t>(), "Constructor with ipAddress and port")
        .def(py::init<uint32_t, uint32_t, uint16_t>(), "Constructor with baudrate, ipAddress, and port")
        .def_readwrite("baudrate", &ConnectionMeta::baudrate, "baudrate")
        .def_property("ip_address", [](const ConnectionMeta& self) { return StringUtils::ipToStr(self.ipAddress);}, [](ConnectionMeta& self, const std::string& ipAddress) { bool_t err; self.ipAddress = StringUtils::toIp(ipAddress, err);}, "ip address")
        .def_readwrite("port", &ConnectionMeta::port, "port")
        .def("is_different", &ConnectionMeta::isDifferent, "Check if two ConnectionMeta objects are different")
        .def("__repr__", [](const ConnectionMeta& self) { 
            if (self.baudrate)
                return "Baudrate " + StringUtils::uintToStr(self.baudrate);
            else
                return "IP " + StringUtils::ipToStr(self.ipAddress) + ":" + StringUtils::uintToStr(self.port); });
        
    py::class_<AutoDiscovery> autoDiscovery(m, "AutoDiscovery");

    py::enum_<AutoDiscovery::Type>(autoDiscovery, "Type")
        .value("Isl", AutoDiscovery::Type::Isl)
        .value("Nmea", AutoDiscovery::Type::Nmea);

    py::class_<Connection>(m, "Connection")
        .def_readonly("port", &Connection::sysPort, "The port")
        .def_readonly("meta", &Connection::meta, "The connection meta");

    py::class_<DeviceScript>(m, "DeviceScript")
        .def(py::init<>())
        .def(py::init<const std::string&, const std::string&>())
        .def_readonly("state", &DeviceScript::state)
        .def_readonly("name", &DeviceScript::name)
        .def_readonly("code", &DeviceScript::code)
         .def("__repr__", [](const DeviceScript& self) { 
            return "name:" + self.name + " code:" + self.code;});
    
    py::enum_<DataState>(m, "DataState")
        .value("Invalid", DataState::Invalid)
        .value("Pending", DataState::Pending)
        .value("Valid", DataState::Valid);

    py::class_<ScriptVars> scriptVars(m, "ScriptVars");
        scriptVars.def_readonly("state", &ScriptVars::state)
        .def_readonly("vars", &ScriptVars::vars);

    py::class_<ScriptVars::Var>(scriptVars, "Var")
        .def_readonly("type", &ScriptVars::Var::type)
        .def_readonly("name", &ScriptVars::Var::name)
        .def_readonly("description", &ScriptVars::Var::description)
        .def("__repr__", [](const ScriptVars::Var& self) { 
            return "name:" + self.name + " description:" + self.description;});

    py::enum_<ScriptVars::Var::Type>(scriptVars, "Type")
        .value("Byte", ScriptVars::Var::Type::Byte)
        .value("Int", ScriptVars::Var::Type::Int)
        .value("Real", ScriptVars::Var::Type::Real)
        .value("ByteArray", ScriptVars::Var::Type::ByteArray)
        .value("IntArray", ScriptVars::Var::Type::IntArray)
        .value("RealArray", ScriptVars::Var::Type::RealArray);

    //------------------------------------------ Maths ----------------------------------------------

    py::class_<Point>(m, "Point")
        .def(py::init<>())
        .def(py::init<real_t, real_t>())
        .def_readwrite("x", &Point::x)
        .def_readwrite("y", &Point::y);

    py::class_<Vector3> vector3(m, "Vector3");
    vector3.def(py::init<>())
        .def(py::init<real_t, real_t, real_t>())
        .def_readwrite("x", &Vector3::x)
        .def_readwrite("y", &Vector3::y)
        .def_readwrite("z", &Vector3::z)
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * real_t())
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= real_t())
        .def("zero", &Vector3::zero)
        .def("magnitude", &Vector3::magnitude)
        .def("magnitude_sq", &Vector3::magnitudeSq)
        .def("normalise", &Vector3::normalise)
        //.def("dot", py::overload_cast<const Vector3&>(&Vector3::dot))
        //.def("cross", py::overload_cast<const Vector3&>(&Vector3::cross))
        .def("find_closest_cardinal_axis", &Vector3::findClosestCardinalAxis)
        .def_static("get_vector_from_axis", &Vector3::getVectorFromAxis)
        .def("__repr__", [](const Vector3& self) { 
            return "x:" + StringUtils::realToStr(self.x, 0, 4) + " y:" + StringUtils::realToStr(self.y, 0, 4) + " z:" + 
            StringUtils::realToStr(self.z, 0, 4);});

    py::enum_<Vector3::Axis>(vector3, "Axis")
        .value("xPlus", Vector3::Axis::xPlus)
        .value("xMinus", Vector3::Axis::xMinus)
        .value("yPlus", Vector3::Axis::yPlus)
        .value("yMinus", Vector3::Axis::yMinus)
        .value("zPlus", Vector3::Axis::zPlus)
        .value("zMinus", Vector3::Axis::zMinus);

    py::class_<Matrix3x3>(m, "Matrix3x3")
        .def(py::init<>())
        .def(py::init<real_t, real_t, real_t>())
        .def(py::init<const Vector3&, const Vector3&>())
        .def_readwrite("m", &Matrix3x3::m)
        .def("identity", &Matrix3x3::identity, "Set the matrix to identity")
        .def("transpose", &Matrix3x3::transpose, "Transpose the matrix")
        .def(py::self * py::self)
        .def(py::self * Vector3())
        .def("__repr__", [](const Matrix3x3& self) {
            return StringUtils::realToStr(self.m[0][0], 0, 5, 0, true) + " " + StringUtils::realToStr(self.m[0][1], 0, 5, 0, true) + " " + StringUtils::realToStr(self.m[0][2], 0, 5, 0, true) + "\n" +
                   StringUtils::realToStr(self.m[1][0], 0, 5, 0, true) + " " + StringUtils::realToStr(self.m[1][1], 0, 5, 0, true) + " " + StringUtils::realToStr(self.m[1][2], 0, 5, 0, true) + "\n" +
                   StringUtils::realToStr(self.m[2][0], 0, 5, 0, true) + " " + StringUtils::realToStr(self.m[2][1], 0, 5, 0, true) + " " + StringUtils::realToStr(self.m[2][2], 0, 5, 0, true); });

    py::class_<EulerAngles>(m, "EulerAngles")
        .def(py::init<>())
        .def("rad_to_deg", &EulerAngles::radToDeg, "Convert radians to degrees")
        .def_readwrite("heading", &EulerAngles::heading, "heading")
        .def_readwrite("pitch", &EulerAngles::pitch, "pitch")
        .def_readwrite("roll", &EulerAngles::roll, "roll")
        .def("__repr__", [](const EulerAngles& self) { 
            return "heading:" + StringUtils::realToStr(self.heading) + " pitch:" + StringUtils::realToStr(self.pitch) + " roll:" + 
            StringUtils::realToStr(self.roll);}, "Convert to string");

    py::class_<Quaternion>(m, "Quaternion")
        .def(py::init<>())
        .def(py::init<real_t, real_t, real_t, real_t>())
        .def(py::init<const Vector3&, real_t>())
        .def(py::init<const Vector3&>())
        .def(py::init<const Matrix3x3&>())
        .def_readwrite("w", &Quaternion::w)
        .def_readwrite("x", &Quaternion::x)
        .def_readwrite("y", &Quaternion::y)
        .def_readwrite("z", &Quaternion::z)
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self * Vector3())
        .def(py::self += py::self)
        .def(py::self *= py::self)
        .def("conjugate", &Quaternion::conjugate, "Conjugate of the quaternion")
        .def("normalise", &Quaternion::normalise, "Normalise the quaternion")
        .def("magnitude", &Quaternion::magnitude, "Magnitude of the quaternion")
        .def("to_matrix", &Quaternion::toMatrix, "Convert to a matrix")
        .def("to_axis_angle", &Quaternion::toAxisAngle, "Convert to an axis angle")
        .def("to_euler_angles", &Quaternion::toEulerAngles, "Convert to radian euler angles", "heading_offset_rad"_a=0)
        .def("angle_between", &Quaternion::angleBetween, "Angle between two quaternions", "q"_a, "axis"_a, "earth_frame"_a)
        .def("get_rotation_about", &Quaternion::getRotationAbout, "Get rotation about an axis", "q"_a, "axis"_a)
        .def("get_down_and_euler_offsets", &Quaternion::getDownAndEulerOffsets, "Get down and euler offsets", "down"_a, "euler_angles"_a)
        .def("__repr__", [](const Quaternion& self) { 
            return "w:" + StringUtils::realToStr(self.w, 0, 4) + " x:" + StringUtils::realToStr(self.x, 0, 4) + " y:" + 
            StringUtils::realToStr(self.y, 0, 4) + " z:" + StringUtils::realToStr(self.z, 0, 4);});
}
//--------------------------------------------------------------------------------------------------