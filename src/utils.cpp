//------------------------------------------ Includes ----------------------------------------------

#include "utils.h"
#include <pybind11/stl.h>

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
std::string PySdk::asBinStr(uint_t value, uint_t bits)
{
    std::string binStr = "";
    for (int_t i = bits-1; i >= 0 ; i--)
    {
        binStr += value & (static_cast<uint_t>(1) << i) ? "1" : "0";
    }
    return binStr;
}
//--------------------------------------------------------------------------------------------------
uint_t PySdk::binStrToValue(const std::string& str)
{
    uint32_t values = 0;

    if (str.length() <= 32)
    {
        values <<= 1;
        for (uint_t i = 0; i < str.length(); i++)
        {
            if (str[i] == '1')
            {
                values |= 0x01;
            }    
        }
    }
    return values;
}
//--------------------------------------------------------------------------------------------------
bool_t PySdk::getDictValue(const py::dict& d, const char* key, bool_t defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::bool_>(d[key]))
        {
            return d[key].cast<bool_t>();
        }
        else
        {
            throw py::type_error("Type mismatch: Expected bool_t for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
uint8_t PySdk::getDictValue(const py::dict& d, const char* key, uint8_t defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::int_>(d[key]))
        {
            return d[key].cast<uint8_t>();
        }
        else
        {
            throw py::type_error("Type mismatch: Expected int for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
uint16_t PySdk::getDictValue(const py::dict& d, const char* key, uint16_t defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::int_>(d[key]))
        {
            return d[key].cast<uint16_t>();
        }
        else
        {
            throw py::type_error("Type mismatch: Expected int for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
uint32_t PySdk::getDictValue(const py::dict& d, const char* key, uint32_t defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::int_>(d[key]))
        {
            return d[key].cast<uint32_t>();
        }
        else
        {
            throw py::type_error("Type mismatch: Expected int for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
uint_t PySdk::getDictValue(const py::dict& d, const char* key, uint_t defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::int_>(d[key]))
        {
            return d[key].cast<uint_t>();
        }
        else
        {
            throw py::type_error("Type mismatch: Expected int for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
int32_t PySdk::getDictValue(const py::dict& d, const char* key, int32_t defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::int_>(d[key]))
        {
            return d[key].cast<int32_t>();
        }
        else
        {
            throw py::type_error("Type mismatch: Expected int for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
real_t PySdk::getDictValue(const py::dict& d, const char* key, real_t defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::float_>(d[key]) || py::isinstance<py::int_>(d[key]))
        {
            return d[key].cast<real_t>();
        }
        else
        {
            throw py::type_error("Type mismatch: Expected float for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
std::string PySdk::getDictValue(const py::dict& d, const char* key, const std::string& defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::str>(d[key]))
        {
            return d[key].cast<std::string>();
        }
        else
        {
            throw py::type_error("Type mismatch: Expected string for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
Uart::Mode PySdk::getDictValue(const py::dict& d, const char* key, Uart::Mode defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::str>(d[key]))
        {
            Uart::Mode mode = StringUtils::toUartMode(d[key].cast<std::string>());
            if (mode == Uart::Mode::Unknown)
            {
                throw py::value_error("Invalid value: " + d[key].cast<std::string>() +" for key " + std::string(key));
            }
            return mode;
        }
        else
        {
            throw py::type_error("Type mismatch: Expected string for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
Uart::Parity PySdk::getDictValue(const py::dict& d, const char* key, Uart::Parity defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::str>(d[key]))
        {
            Uart::Parity parity = StringUtils::toUartParity(d[key].cast<std::string>());
            if (parity == Uart::Parity::Unknown)
            {
                throw py::value_error("Invalid value: " + d[key].cast<std::string>() +" for key " + std::string(key));
            }
            return parity;
        }
        else
        {
            throw py::type_error("Type mismatch: Expected string for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
   
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
Uart::StopBits PySdk::getDictValue(const py::dict& d, const char* key, Uart::StopBits defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::str>(d[key]))
        {
            Uart::StopBits stopBits = StringUtils::toUartStopBits(d[key].cast<std::string>());
            if (stopBits == Uart::StopBits::Unknown)
            {
                throw py::value_error("Invalid value: " + d[key].cast<std::string>() +" for key " + std::string(key));
            }
            return stopBits;
        }
        else if (py::isinstance<py::int_>(d[key]) || py::isinstance<py::float_>(d[key]))
        {
            if (d[key].cast<int_t>() == 1)
            {
                return Uart::StopBits::One;
            }
            else if (d[key].cast<int_t>() == 2)
            {
                return Uart::StopBits::Two;
            }
            else if (((d[key].cast<real_t>() - 1.5) < 0.001) && ((d[key].cast<real_t>() - 1.5) > -0.001))
            {
                return Uart::StopBits::OneAndHalf;
            }
            else
            {
                throw py::value_error("Invalid value for key " + std::string(key));
            }
        }
        else
        {
            throw py::type_error("Type mismatch: Expected string, int or float for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
Math::Quaternion PySdk::getDictValue(const py::dict& d, const char* key, const Math::Quaternion& defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::tuple>(d[key]))
        {
            py::tuple t = d[key].cast<py::tuple>();
            if (t.size() == 4)
            {
                return Math::Quaternion(t[0].cast<real_t>(), t[1].cast<real_t>(), t[2].cast<real_t>(), t[3].cast<real_t>());
            }
            else
            {
                throw py::type_error("Type mismatch: Expected tuple of size 4 for key " + std::string(key));
            }
        }
        else if (py::isinstance<py::list>(d[key]))
        {
            py::list l = d[key].cast<py::list>();
            if (l.size() == 4)
            {
                return Math::Quaternion(l[0].cast<real_t>(), l[1].cast<real_t>(), l[2].cast<real_t>(), l[3].cast<real_t>());
            }
            else
            {
                throw py::type_error("Type mismatch: Expected list of size 4 for key " + std::string(key));
            }
        }
        else
        {
            throw py::type_error("Type mismatch: Expected tuple for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
Math::Vector3 PySdk::getDictValue(const py::dict& d, const char* key, const Math::Vector3& defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::tuple>(d[key]))
        {
            py::tuple t = d[key].cast<py::tuple>();
            if (t.size() == 3)
            {
                return Math::Vector3(t[0].cast<real_t>(), t[1].cast<real_t>(), t[2].cast<real_t>());
            }
            else
            {
                throw py::type_error("Type mismatch: Expected tuple of size 3 for key " + std::string(key));
            }
        }
        else if (py::isinstance<py::list>(d[key]))
        {
            py::list l = d[key].cast<py::list>();
            if (l.size() == 3)
            {
                return Math::Vector3(l[0].cast<real_t>(), l[1].cast<real_t>(), l[2].cast<real_t>());
            }
            else
            {
                throw py::type_error("Type mismatch: Expected list of size 3 for key " + std::string(key));
            }
        }
        else
        {
            throw py::type_error("Type mismatch: Expected tuple for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
Device::CustomStr PySdk::getDictValue(const py::dict& d, const char* key, const Device::CustomStr& defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::dict>(d[key]))
        {
            py::dict dict = d[key].cast<py::dict>();
            Device::CustomStr customStr;
            if (dict.contains("enable"))
            {
                if (py::isinstance<py::bool_>(dict["enable"]))
                {
                    customStr.enable = dict["enable"].cast<bool_t>();
                }
                else
                {
                    throw py::type_error("Type mismatch: Expected bool for key[" + std::string(key) + "][enable], got " + std::string(py::str(d[key].get_type())));
                }
            }
            else
            {
                customStr.enable = defaultValue.enable;
            }

            if (dict.contains("str"))
            {
                if (py::isinstance<py::str>(dict["str"]))
                {
                    customStr.str = dict["str"].cast<std::string>();
                }
                else
                {
                    throw py::type_error("Type mismatch: Expected string for key[" + std::string(key) + "][str], got " + std::string(py::str(d[key].get_type())));
                }
            }
            else
            {
                customStr.str = defaultValue.str;
            }
            return customStr;
        }
        else
        {
            throw py::type_error("Type mismatch: Expected dict for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
Device::PhyPortMode PySdk::getDictValue(const py::dict& d, const char* key, Device::PhyPortMode defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::str>(d[key]))
        {
            Device::PhyPortMode portMode = StringUtils::toPhyPortMode(d[key].cast<std::string>());
            if (portMode != Device::PhyPortMode::Unknown)
            {
                return portMode;
            }
        }
        else
        {
            throw py::type_error("Type mismatch: Expected string for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------
Device::PhyMdixMode PySdk::getDictValue(const py::dict& d, const char* key, Device::PhyMdixMode defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::str>(d[key]))
        {
            Device::PhyMdixMode mdixMode = StringUtils::toPhyMdixMode(d[key].cast<std::string>());
            if (mdixMode != Device::PhyMdixMode::Unknown)
            {
                return mdixMode;
            }
        }
        else
        {
            throw py::type_error("Type mismatch: Expected string for key " + std::string(key) + ", got " + std::string(py::str(d[key].get_type())));
        }
    }
    return defaultValue;
}
//--------------------------------------------------------------------------------------------------