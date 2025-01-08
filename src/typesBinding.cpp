//------------------------------------------ Includes ----------------------------------------------

#include "typesBinding.h"
#include "types/sdkTypes.h"
#include "utils/stringUtils.h"
#include "maths/vector.h"
#include "maths/matrix.h"
#include "maths/quaternion.h"
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
        }, "Return as a bytes object");

    py::class_<ConnectionMeta>(m, "ConnectionMeta")
        .def(py::init<uint32_t>(), "Constructor with baudrate")
        .def(py::init( [](const std::string& ipString, uint16_t port) {
            bool_t err;
            return new ConnectionMeta(StringUtils::toIp(ipString, err), port);
        }), "Constructor with ip address and port")
        .def_readwrite("baudrate", &ConnectionMeta::baudrate, "baudrate")
        .def_property("ip_address", [](const ConnectionMeta& self) { return StringUtils::ipToStr(self.ipAddress);}, [](ConnectionMeta& self, const std::string& ipAddress) { bool_t err; self.ipAddress = StringUtils::toIp(ipAddress, err);}, "ip address")
        .def_property("ip_address_int", [](const ConnectionMeta& self) { return self.ipAddress;}, [](ConnectionMeta& self, uint32_t ipAddress) { self.ipAddress = ipAddress;}, "ip address")
        .def_readwrite("port", &ConnectionMeta::port, "port")
        .def("is_different", &ConnectionMeta::isDifferent, "Check if two ConnectionMeta objects are different")
        .def("__repr__", [](const ConnectionMeta& self) { 
            if (self.baudrate)
                return "Baudrate " + StringUtils::toStr(self.baudrate);
            else
                return "IP " + StringUtils::ipToStr(self.ipAddress) + ":" + StringUtils::toStr(self.port); })
        .def("to_dict", [](const ConnectionMeta& self) -> py::dict
        {
            py::dict pyDict;
            if (self.baudrate)
                pyDict["baudrate"] = self.baudrate;
            else
            {
                pyDict["ip_address"] = StringUtils::ipToStr(self.ipAddress);
                pyDict["port"] = self.port;
            }
            return pyDict;
        }, "Convert to a dictionary");
        
    py::class_<AutoDiscovery> autoDiscovery(m, "AutoDiscovery");

    py::enum_<AutoDiscovery::Type>(autoDiscovery, "Type")
        .value("Isl", AutoDiscovery::Type::Isl)
        .value("Nmea", AutoDiscovery::Type::Nmea);

    py::class_<Connection>(m, "Connection")
        .def_readonly("port", &Connection::sysPort, "The port")
        .def_readonly("meta", &Connection::meta, "The connection meta")
        .def("to_dict", [](const Connection& self) -> py::dict
        {
            py::dict pyDict;
            pyDict["port"] = self.sysPort->id;
            pyDict["meta"] = py::cast(self.meta).attr("to_dict")();
            return pyDict;
        }, "Convert to a dictionary");

    py::class_<DeviceScript>(m, "DeviceScript")
        .def(py::init<>())
        .def(py::init<const std::string&, const std::string&>())
        .def_readonly("state", &DeviceScript::state)
        .def_readonly("name", &DeviceScript::name)
        .def_readonly("code", &DeviceScript::code)
        .def("__repr__", [](const DeviceScript& self) { 
            return "name:" + self.name + " code:" + self.code;})
        .def("to_dict", [](const DeviceScript& self) -> py::dict
        {
            py::dict pyDict;
            pyDict["name"] = self.name;
            pyDict["code"] = self.code;
            return pyDict;
        }, "Convert to a dictionary");
    
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


    py::class_<Math::Vector3> vector3(m, "Vector3");
    vector3.def(py::init<>())
        .def(py::init<real_t, real_t, real_t>())
        .def(py::init( [](const py::list& list)
        {
            if (py::len(list) != 3) throw py::value_error("Invalid list size");
            return new Math::Vector3(list[0].cast<real_t>(), list[1].cast<real_t>(), list[2].cast<real_t>());
        }), "Constructor with a list")
        .def_readwrite("x", &Math::Vector3::x)
        .def_readwrite("y", &Math::Vector3::y)
        .def_readwrite("z", &Math::Vector3::z)
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * real_t())
        .def(py::self += py::self)
        .def(py::self -= py::self)
        .def(py::self *= real_t())
        .def("zero", &Math::Vector3::zero)
        .def("magnitude", &Math::Vector3::magnitude)
        .def("magnitude_sq", &Math::Vector3::magnitudeSq)
        .def("normalise", &Math::Vector3::normalise)
        .def("dot", &Math::Vector3::dot, "Dot product", "v"_a)
        .def("cross", &Math::Vector3::cross, "Cross product", "v"_a)
        .def("find_closest_cardinal_axis", &Math::Vector3::findClosestCardinalAxis)
        .def_static("get_vector_from_axis", &Math::Vector3::getVectorFromAxis)
        .def("__repr__", [](const Math::Vector3& self) { 
            return "x:" + StringUtils::toStr(self.x, 0, 4) + " y:" + StringUtils::toStr(self.y, 0, 4) + " z:" + 
            StringUtils::toStr(self.z, 0, 4);});

    py::enum_<Math::Vector3::Axis>(vector3, "Axis")
        .value("xPlus", Math::Vector3::Axis::xPlus)
        .value("xMinus", Math::Vector3::Axis::xMinus)
        .value("yPlus", Math::Vector3::Axis::yPlus)
        .value("yMinus", Math::Vector3::Axis::yMinus)
        .value("zPlus", Math::Vector3::Axis::zPlus)
        .value("zMinus", Math::Vector3::Axis::zMinus);

    py::class_<Math::Matrix3x3>(m, "Matrix3x3")
        .def(py::init<>())
        .def(py::init( [](const py::list& list)
        {
            if (py::len(list) == 3)
            {
                const py::list& r0 = list[0];
                const py::list& r1 = list[1];
                const py::list& r2 = list[2];
                return new Math::Matrix3x3(r0[0].cast<real_t>(), r0[1].cast<real_t>(), r0[2].cast<real_t>(),
                                            r1[0].cast<real_t>(), r1[1].cast<real_t>(), r1[2].cast<real_t>(),
                                            r2[0].cast<real_t>(), r2[1].cast<real_t>(), r2[2].cast<real_t>());
            }
            else if (py::len(list) == 9)
            {
                    return new Math::Matrix3x3(list[0].cast<real_t>(), list[1].cast<real_t>(), list[2].cast<real_t>(),
                                            list[3].cast<real_t>(), list[4].cast<real_t>(), list[5].cast<real_t>(),
                                            list[6].cast<real_t>(), list[7].cast<real_t>(), list[8].cast<real_t>());
            }
            throw py::value_error("Invalid list size");
        }), "Constructor with a list")
        .def(py::init<real_t, real_t, real_t>(), "Initalise a matrix from Eular angles" , "heading"_a, "pitch"_a, "roll"_a)
        .def("__getitem__", [](Math::Matrix3x3& self, std::pair<uint_t, uint_t> index) {
            if (index.first >= 3 || index.second >= 3) throw py::index_error();
            return self[index.first][index.second];
        }, "Get an element", "index"_a)
        .def("__setitem__", [](Math::Matrix3x3& self, std::pair<uint_t, uint_t> index, real_t value) {
            if (index.first >= 3 || index.second >= 3) throw py::index_error();
            self[index.first][index.second] = value;
        }, "Set an element", "index"_a, "value"_a)
        .def("set_identity", &Math::Matrix3x3::setIdentity, "Set the matrix to identity")
        .def("transpose", &Math::Matrix3x3::transpose, "Transpose the matrix")
        .def(py::self * py::self)
        .def(py::self * Math::Vector3())
        .def("to_list", [](const Math::Matrix3x3& self)
        {
            return std::vector<std::vector<real_t>>{ {self[0][0], self[0][1], self[0][2]},
                                                     {self[1][0], self[1][1], self[1][2]},
                                                     {self[2][0], self[2][1], self[2][2]} }; 
        }, "Convert to a list")
        .def("__repr__", [](const Math::Matrix3x3& self) {
            return StringUtils::toStr(self[0][0], 0, 5, 0, true) + " " + StringUtils::toStr(self[0][1], 0, 5, 0, true) + " " + StringUtils::toStr(self[0][2], 0, 5, 0, true) + "\n" +
                   StringUtils::toStr(self[1][0], 0, 5, 0, true) + " " + StringUtils::toStr(self[1][1], 0, 5, 0, true) + " " + StringUtils::toStr(self[1][2], 0, 5, 0, true) + "\n" +
                   StringUtils::toStr(self[2][0], 0, 5, 0, true) + " " + StringUtils::toStr(self[2][1], 0, 5, 0, true) + " " + StringUtils::toStr(self[2][2], 0, 5, 0, true); });

    py::class_<Math::EulerAngles>(m, "EulerAngles")
        .def(py::init<>())
        .def("rad_to_deg", &Math::EulerAngles::radToDeg, "Convert radians to degrees", "rad_a")
        .def_readwrite("heading", &Math::EulerAngles::heading, "heading")
        .def_readwrite("pitch", &Math::EulerAngles::pitch, "pitch")
        .def_readwrite("roll", &Math::EulerAngles::roll, "roll")
        .def("__repr__", [](const Math::EulerAngles& self) { 
            return "heading:" + StringUtils::toStr(self.heading) + " pitch:" + StringUtils::toStr(self.pitch) + " roll:" + 
            StringUtils::toStr(self.roll);}, "Convert to string");

    py::class_<Math::Quaternion>(m, "Quaternion")
        .def(py::init<>())
        .def(py::init<real_t, real_t, real_t, real_t>())
        .def(py::init<const Math::Vector3&, real_t>())
        .def(py::init<const Math::Vector3&>())
        .def(py::init<const Math::Matrix3x3&>())
        .def(py::init( [](const py::list& list)
        {
            if (py::len(list) != 4) throw py::value_error("Invalid list size");
            return new Math::Quaternion(list[0].cast<real_t>(), list[1].cast<real_t>(), list[2].cast<real_t>(), list[3].cast<real_t>());
        }), "Constructor with a list")
        .def_readwrite("w", &Math::Quaternion::w)
        .def_readwrite("x", &Math::Quaternion::x)
        .def_readwrite("y", &Math::Quaternion::y)
        .def_readwrite("z", &Math::Quaternion::z)
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self * Math::Vector3())
        .def(py::self += py::self)
        .def(py::self *= py::self)
        .def("conjugate", &Math::Quaternion::conjugate, "Conjugate of the quaternion")
        .def("normalise", &Math::Quaternion::normalise, "Normalise the quaternion")
        .def("magnitude", &Math::Quaternion::magnitude, "Magnitude of the quaternion")
        .def("to_matrix", &Math::Quaternion::toMatrix, "Convert to a matrix")
        .def("to_euler_angles", &Math::Quaternion::toEulerAngles, "Convert to radian euler angles", "heading_offset_rad"_a=0)
        .def("__repr__", [](const Math::Quaternion& self) { 
            return "w:" + StringUtils::toStr(self.w, 0, 4) + " x:" + StringUtils::toStr(self.x, 0, 4) + " y:" + 
            StringUtils::toStr(self.y, 0, 4) + " z:" + StringUtils::toStr(self.z, 0, 4);});
}
//--------------------------------------------------------------------------------------------------