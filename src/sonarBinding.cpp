//------------------------------------------ Includes ----------------------------------------------

#include "sonarBinding.h"
#include "deviceBinding.h"
#include "devices/sonar.h"
#include "helpers/sonarImage.h"
#include "helpers/palette.h"
#include "helpers/sonarDataStore.h"
#include "utils/stringUtils.h"
#include "files/bmpFile.h"
#include "utils.h"
#include <pybind11/stl.h>

using namespace pybind11::literals;
using namespace IslSdk;

Sonar::System dictToSystemSettings(const py::dict& d, const Sonar::System& defualt);
Sonar::Acoustic dictToAcousticSettings(const py::dict& d, const Sonar::Acoustic& defualt);
Sonar::Setup dictToSetupSettings(const py::dict& d, const Sonar::Setup& defualt);
Sonar::System::GatingMode getFromDict(const py::dict& d, const char* key, Sonar::System::GatingMode defaultValue);
Sonar::Setup::EchoMode getFromDict(const py::dict& d, const char* key, Sonar::Setup::EchoMode defaultValue);

//--------------------------------------------------------------------------------------------------
class Buf2D
{
public:
    std::vector<uint32_t> buf;
    uint_t width;
    uint_t height;
    Buf2D(uint_t width, uint_t height) : buf(width * height), width(width), height(height) {}
};

void PySdk::initSonar(py::module &m)
{
    py::class_<Buf2D>(m, "Buf2D", py::buffer_protocol())
        .def(py::init<uint_t, uint_t>(), "width"_a, "height"_a)
        .def_buffer([](Buf2D& self) -> py::buffer_info 
        {
            return py::buffer_info(
                self.buf.data(),
                sizeof(uint32_t),
                py::format_descriptor<uint32_t>::format(),
                2,
                { self.height, self.width },
                { self.width * sizeof(uint32_t), sizeof(uint32_t) },
                true
            );
        })
        .def_property_readonly("__array_interface__", [](Buf2D& self) -> py::dict
        {
            py::dict d;
            d["shape"] = py::make_tuple(self.height, self.width);
            d["typestr"] = py::format_descriptor<uint32_t>::format(),
            d["data"] = py::make_tuple( reinterpret_cast<size_t>(self.buf.data()), true);
            d["strides"] = py::make_tuple(self.width * sizeof(uint32_t), sizeof(uint32_t));
            d["version"] = 3;
            return d;
        }, "Return a NumPy array with __array_interface__ support")
        .def("to_bytes", [](Buf2D& self) -> py::bytes
        {
            return py::bytes(reinterpret_cast<const char*>(self.buf.data()), self.buf.size() * sizeof(uint32_t));
        }, "Return the image as a bytes object");

    py::class_<Palette>(m, "Palette")
        .def(py::init<>())
        .def("set_to_default", &Palette::setToDefault, "Set the palette to the default")
        .def("set", [](Palette& self, const std::vector<std::tuple<uint32_t, uint16_t>>& gradients, uint32_t nullColour)
        {
            std::vector<Palette::GradientValue> g;
            g.reserve(gradients.size());
            for (const auto& gradient : gradients)
            {
                g.push_back({ std::get<0>(gradient), std::get<1>(gradient) });
            }
            self.set(g, nullColour);
        }, "Set the palette", "gradient"_a, "null_colour"_a)
        .def("render", [](Palette& self, uint_t width, uint_t height, bool_t horizontal)
        {
            Buf2D* buf = new Buf2D(width, height);
            self.render(buf->buf.data(), width, height, horizontal);
            return py::cast(buf, py::return_value_policy::take_ownership);
        }, "Render the palette", "width"_a, "height"_a, "horizontal"_a = true)
        .def("save_bmp", [] (Palette& self, const std::string& fileName, uint_t width, uint_t height, bool_t horizontal)
        {
            Buf2D buf2D = {width, height};
            self.render(buf2D.buf.data(), width, height, horizontal);
            BmpFile::save(fileName, &buf2D.buf[0], 32, buf2D.width, buf2D.height);
        }, "Save the image to a BMP file", "file_name"_a, "width"_a, "height"_a, "horizontal"_a = true);

    py::class_<SonarImage>(m, "SonarImage", py::buffer_protocol())
        .def(py::init<>())
        .def(py::init<int32_t, int32_t, bool_t, bool_t>(), "width"_a, "height"_a, "use_4_bpp"_a = true, "use_biliner_interpolation"_a = true)
        .def_buffer([](SonarImage& self) -> py::buffer_info 
        {
            return py::buffer_info(
                const_cast<uint8_t*>(self.buf.data()),
                sizeof(uint8_t),
                self.bpp == 2 ? py::format_descriptor<uint16_t>::format() : py::format_descriptor<uint32_t>::format(),
                2,
                { self.height, self.width },
                { static_cast<uint32_t>(self.width * self.bpp), static_cast<uint32_t>(self.bpp)},
                true
            );
        })
        .def_property_readonly("__array_interface__", [](SonarImage& self) -> py::dict
        {
            py::dict d;
            d["shape"] = py::make_tuple(self.height, self.width);  // Shape
            d["typestr"] = self.bpp == 2 ? py::format_descriptor<uint16_t>::format() : py::format_descriptor<uint32_t>::format(),
            d["data"] = py::make_tuple( reinterpret_cast<size_t>(self.buf.data()), true);
            d["strides"] = py::make_tuple(self.width * self.bpp, self.bpp);
            d["version"] = 3;
            return d;
        }, "Return a NumPy array with __array_interface__ support")
        .def("to_bytes", [](SonarImage& self) -> py::bytes
        {
            return py::bytes(reinterpret_cast<const char*>(self.buf.data()), self.buf.size());
        }, "Return the image as a bytes object")
        .def_property_readonly("width", [](const SonarImage& self) { return self.width; })
        .def_property_readonly("height",[](const SonarImage& self) { return self.height; })
        .def_property_readonly("bpp", [](const SonarImage& self) { return self.bpp; })
        .def_readwrite("use_biliner_interpolation", &SonarImage::useBilinerInterpolation)
        .def("set_buffer", &SonarImage::setBuffer, "Set the buffer", "width"_a, "height"_a, "use_4_bpp"_a)
        .def("set_sector_area", &SonarImage::setSectorArea, "Set the sector area", "min_range_mm"_a, "max_range_mm"_a, "sector_start"_a, "sector_size"_a)
        .def("render", &SonarImage::render, "Render the image", "data"_a, "palette"_a, "redraw"_a = false)
        .def("render_16_bit", &SonarImage::render16Bit, "Render the image", "data"_a, "redraw"_a = false)
        .def("render_texture", &SonarImage::renderTexture, "Render the image", "data"_a, "palette"_a, "redraw"_a = false)
        .def("render_texture_16_bit", &SonarImage::renderTexture16Bit, "Render the image", "data"_a, "redraw"_a = false)
        .def("save_bmp", [] (const SonarImage& self, const std::string& fileName)
        {
            BmpFile::save(fileName, reinterpret_cast<const uint32_t*>(&self.buf[0]), 32, self.width, self.height);
        }, "Save the image to a BMP file", "file_name"_a);

    py::class_<SonarDataStore>(m, "SonarDataStore")
        .def(py::init<>())
        .def("add", &SonarDataStore::add, "Add ping data", "ping"_a, "blank_range_mm"_a = 0)
        .def("clear", &SonarDataStore::clear, "Clear ping data", "start_angle"_a = 0, "angle_size"_a = Sonar::maxAngle);;

    py::class_<Sonar, Device, std::shared_ptr<Sonar>> sonar(m, "Sonar");
    sonar.def(py::init<const Device::Info&>())
        .def_readonly_static("max_angle", &Sonar::maxAngle, "Get the maximum angle")
        .def_property_readonly("settings", [](const Sonar& self) { return self.settings; })
        .def_property_readonly("sensor_rates", [](const Sonar& self) { return self.sensorRates; })
        .def_property_readonly("tvg_points", [](const Sonar& self) { return self.tvgPoints; })
        .def_property_readonly("mac_address", [](const Sonar& self) { return StringUtils::macAddressToStr(self.macAddress); })
        .def_readonly("ahrs", &Sonar::ahrs, "AHRS sensor")
        .def_readonly("gyro", &Sonar::gyro, "Gyro sensor")
        .def_readonly("accel", &Sonar::accel, "Accelerometer sensor")
        .def_readonly("mag", &Sonar::mag, "Magnetometer sensor")
        .def_readonly("on_settings_updated", &Sonar::onSettingsUpdated)
        .def_readonly("on_head_indexes_acquired", &Sonar::onHeadIndexesAcquired)
        .def_readonly("on_ping_data", &Sonar::onPingData)
        .def_readonly("on_echo_data", &Sonar::onEchoData)
        .def_readonly("on_pwr_and_temp", &Sonar::onPwrAndTemp)
        .def_readonly("on_motor_slip", &Sonar::onMotorSlip)
        .def_readonly("on_motor_move_complete", &Sonar::onMotorMoveComplete)
        .def("set_sensor_rates", &Sonar::setSensorRates, "Set the sensor rates", "sensors"_a)
        .def("set_system_settings", &Sonar::setSystemSettings, "Set the system settings", "settings"_a, "save"_a = false)
        .def("set_system_settings", [](Sonar& self, const py::dict& dict, bool save) { 
            Sonar::System settings = dictToSystemSettings(dict, self.settings.system);
            self.setSystemSettings(settings, save);
        }, "Set the system settings from a dictionary", "settings_dict"_a, "save"_a = false)
        .def("set_acoustic_settings", &Sonar::setAcousticSettings, "Set the acoustic settings", "settings"_a, "save"_a = false)
        .def("set_acoustic_settings", [](Sonar& self, const py::dict& dict, bool save) { 
            Sonar::Acoustic settings = dictToAcousticSettings(dict, self.settings.acoustic);
            self.setAcousticSettings(settings, save);
        }, "Set the acoustic settings from a dictionary", "settings_dict"_a, "save"_a = false)
        .def("set_setup_settings", &Sonar::setSetupSettings, "Set the setup settings", "settings"_a, "save"_a = false)
        .def("set_setup_settings", [](Sonar& self, const py::dict& dict, bool save) { 
            Sonar::Setup settings = dictToSetupSettings(dict, self.settings.setup);
            self.setSetupSettings(settings, save);
        }, "Set the setup settings from a dictionary", "settings_dict"_a, "save"_a = false)
        .def("check_head_idx", &Sonar::acquireHeadIdx, "Reacquires the transducer position and reports any errors", "full_sync"_a = false)
        .def("start_scanning", &Sonar::startScanning, "Start scanning")
        .def("stop_scanning", &Sonar::stopScanning, "Stop scanning")
        .def("move_head", &Sonar::moveHead, "Move the head", "angle"_a, "relative"_a = false)
        .def("test_pattern", &Sonar::testPattern, "Test pattern", "enable"_a)
        .def("set_tvg", &Sonar::setTvg, "Set the TVG curve", "points"_a)
        .def_static("get_default_tvg", &Sonar::getDefaultTvg, "Get the default TVG curve")
        .def("load_config", [](const Sonar& self, const std::string& filename)
        { 
            Device::Info info;
            Sonar::Settings settings;
            Sonar::AhrsCal cal;
            std::array<Point, 9> tvgPoints;
            if (self.loadConfig(filename, &info, &settings, &cal, &tvgPoints))
            {
                return py::make_tuple(py::cast(info), py::cast(settings), py::cast(cal), py::cast(tvgPoints));
            }
            else
            {
                return py::make_tuple();
            }
        }, "Load the config")
        .def("has_ahrs", &Sonar::hasAhrs, "Has AHRS")
        .def("get_type", &Sonar::getType, "Gets the type of sonar");

    py::enum_<Sonar::Type>(sonar, "SonarType")
        .value("Iss360HD", Sonar::Type::Iss360HD)
        .value("Iss360", Sonar::Type::Iss360)
        .value("Isp360Profiler", Sonar::Type::Isp360Profiler)
        .value("Unknown", Sonar::Type::Unknown);
       
    //----------------------------------------- Signals --------------------------------------------
    
    py::class_<Signal<Sonar&, bool_t, Sonar::Settings::Type>>(sonar, "SignalOnSettingsUpdated")
        .def("connect", &Signal<Sonar&, bool_t, Sonar::Settings::Type>::pyConnect, "Connect to the signal with a callback_function(sonar, success, type)")
        .def("disconnect", &Signal<Sonar&, bool_t, Sonar::Settings::Type>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Sonar&, const Sonar::HeadIndexes&>>(sonar, "SignalOnHeadIndexesAcquired")
        .def("connect", &Signal<Sonar&, const Sonar::HeadIndexes&>::pyConnect, "Connect to the signal with a callback_function(sonar, homeInfo)")
        .def("disconnect", &Signal<Sonar&, const Sonar::HeadIndexes&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Sonar&, const Sonar::Ping&>>(sonar, "SignalOnPingData")
        .def("connect", &Signal<Sonar&, const Sonar::Ping&>::pyConnect, "Connect to the signal with a callback_function(sonar, ping)")
        .def("disconnect", &Signal<Sonar&, const Sonar::Ping&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Sonar&, const Sonar::Echos&>>(sonar, "SignalOnEchoData")
        .def("connect", &Signal<Sonar&, const Sonar::Echos&>::pyConnect, "Connect to the signal with a callback_function(sonar, echos)")
        .def("disconnect", &Signal<Sonar&, const Sonar::Echos&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Sonar&, const Sonar::CpuPowerTemp&>>(sonar, "SignalOnPwrAndTemp")
        .def("connect", &Signal<Sonar&, const Sonar::CpuPowerTemp&>::pyConnect, "Connect to the signal with a callback_function(sonar, status)")
        .def("disconnect", &Signal<Sonar&, const Sonar::CpuPowerTemp&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Sonar&>>(sonar, "SignalOnMotorSlip")
        .def("connect", &Signal<Sonar&>::pyConnect, "Connect to the signal with a callback_function(sonar, slip)")
        .def("disconnect", &Signal<Sonar&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Sonar&, bool_t>>(sonar, "SignalOnMotorMoveComplete")
        .def("connect", &Signal<Sonar&, bool_t>::pyConnect, "Connect to the signal with a callback_function(sonar, success)")
        .def("disconnect", &Signal<Sonar&, bool_t>::pyDisconnect, "Disconnect a callback function from the signal");

    
    //----------------------------------------- Structs --------------------------------------------

    py::class_<Sonar::Sector>(sonar, "Sector")
        .def(py::init<>())
        .def_readwrite("start", &Sonar::Sector::start, "Start angle of the sector")
        .def_readwrite("size", &Sonar::Sector::size, "Size of the sector");

    py::class_<Sonar::System> system(sonar, "System");
    system.def(py::init<>())
        .def(py::init([](const py::dict& d) { return dictToSystemSettings(d, Sonar::System()); }))
        .def_readwrite("uart_mode", &Sonar::System::uartMode, "Serial port mode")
        .def_readwrite("baudrate", &Sonar::System::baudrate, "Serial port baudrate")
        .def_property("ip_address", [](const Sonar::System& self) { return StringUtils::ipToStr(self.ipAddress);}, [](Sonar::System& self, const std::string& ipAddress) { bool_t err; self.ipAddress = StringUtils::toIp(ipAddress, err);}, "ip address")
        .def_property("netmask", [](const Sonar::System& self) { return StringUtils::ipToStr(self.netmask);}, [](Sonar::System& self, const std::string& netmask) { bool_t err; self.netmask = StringUtils::toIp(netmask, err);}, "netmask")
        .def_property("gateway", [](const Sonar::System& self) { return StringUtils::ipToStr(self.gateway);}, [](Sonar::System& self, const std::string& gateway) { bool_t err; self.gateway = StringUtils::toIp(gateway, err);}, "gateway")
        .def_readwrite("port", &Sonar::System::port, "Port the device listens on and transmits from")
        .def_readwrite("phy_port_mode", &Sonar::System::phyPortMode, "Ethernet connection speed anf duplex mode")
        .def_readwrite("phy_mdix_mode", &Sonar::System::phyMdixMode, "Ethernet TX/RX swapping mode")
        .def_readwrite("use_dhcp", &Sonar::System::useDhcp, "If true device will request an IP address from the DHCP server")
        .def_readwrite("invert_head_direction", &Sonar::System::invertHeadDirection, "If true the head direction is swapped")
        .def_readwrite("ahrs_mode", &Sonar::System::ahrsMode, "If bit zero is 1 use inertial mode. 0 is mag slave mode")
        .def_readwrite("orientation_offset", &Sonar::System::orientationOffset, "Heading, pitch and roll offsets (or down and forward vectors) expressed as a quaternion")
        .def_readwrite("heading_offset_rad", &Sonar::System::headingOffsetRad, "Offset in radians to add to the heading. Typically use for magnetic declination")
        .def_readwrite("turns_about", &Sonar::System::turnsAbout, "A vector representing the axis which turn are measured about")
        .def_readwrite("turns_about_earth_frame", &Sonar::System::turnsAboutEarthFrame, "If true the \"turnAbout\" vector is referenced to the earth frame. False is sensor frame")
        .def_readwrite("use_xc_norm", &Sonar::System::useXcNorm, "Output normalised cross correlation data instead of unnormalised. Normalised data represents the quality of the echo rather than the strength of the echo")
        .def_readwrite("data_8_bit", &Sonar::System::data8Bit, "true = 8-bit data, false = 16-bit data")
        .def_readwrite("gating_mode", &Sonar::System::gatingMode, "The mode of gating to use. Off, On, RollCompensation")
        .def_readwrite("gating_angle", &Sonar::System::gatingAngle, "The gating angle to a line perpendicular to the sector center in degrees")
        .def_readwrite("speed_of_sound", &Sonar::System::speedOfSound, "Speed of sound in meters per second. limits 1000 to 2500")
        .def("defaults", &Sonar::System::defaults, "Set the default settings")
        .def("to_dict", [](const Sonar::System& self)
        {
            py::dict d;
            d["uart_mode"] = py::cast(self.uartMode).attr("name").cast<std::string>();
            d["baudrate"] = self.baudrate;
            d["ip_address"] = StringUtils::ipToStr(self.ipAddress);
            d["netmask"] = StringUtils::ipToStr(self.netmask);
            d["gateway"] = StringUtils::ipToStr(self.gateway);
            d["port"] = self.port;
            d["phy_port_mode"] = py::cast(self.phyPortMode).attr("name").cast<std::string>();
            d["phy_mdix_mode"] = py::cast(self.phyMdixMode).attr("name").cast<std::string>();
            d["use_dhcp"] = self.useDhcp;
            d["invert_head_direction"] = self.invertHeadDirection;
            d["ahrs_mode"] = self.ahrsMode;
            d["orientation_offset"] = py::make_tuple(self.orientationOffset.w, self.orientationOffset.x, self.orientationOffset.y, self.orientationOffset.z);
            d["heading_offset_rad"] = self.headingOffsetRad;
            d["turns_about"] = py::make_tuple(self.turnsAbout.x, self.turnsAbout.y, self.turnsAbout.z);
            d["turns_about_earth_frame"] = self.turnsAboutEarthFrame;
            d["use_xc_norm"] = self.useXcNorm;
            d["data_8_bit"] = self.data8Bit;
            d["gating_mode"] = py::cast(self.gatingMode).attr("name").cast<std::string>();
            d["gating_angle"] = self.gatingAngle;
            d["speed_of_sound"] = self.speedOfSound;
            return d;
        }, "Return the settings as a dictionary");

    py::enum_<Sonar::System::GatingMode>(system, "GatingMode")
        .value("Off", Sonar::System::GatingMode::Off)
        .value("On", Sonar::System::GatingMode::On)
        .value("RollComp", Sonar::System::GatingMode::RollComp);

    py::class_<Sonar::Acoustic> acoustic(sonar, "Acoustic");
    acoustic.def(py::init<>())
        .def(py::init([](const py::dict& d) { return dictToAcousticSettings(d, Sonar::Acoustic()); }))
        .def_readwrite("tx_start_frequency", &Sonar::Acoustic::txStartFrequency, "Transmit pulse start frequency in hertz")
        .def_readwrite("tx_end_frequency", &Sonar::Acoustic::txEndFrequency, "Transmit pulse end frequency in hertz")
        .def_readwrite("tx_pulse_width_us", &Sonar::Acoustic::txPulseWidthUs, "Transmit pulse length in micro seconds")
        .def_readwrite("tx_pulse_amplitude", &Sonar::Acoustic::txPulseAmplitude, "Transmit pulse amplitude as a percent 0% to 100%")
        .def_readwrite("high_sample_rate", &Sonar::Acoustic::highSampleRate, "If true the ADC sample rate is 5 MHz else it's 2.5 MHz")
        .def_property("psk_code", [](const Sonar::Acoustic& self) 
        { 
            return PySdk::asBinStr(self.pskCode, self.pskLength);
        }, 
        [](Sonar::Acoustic& self, const std::string& code) 
        { 
            if (code.length() <= 32)
            {
                self.pskCode = static_cast<uint32_t>(PySdk::binStrToValue(code));
                self.pskLength = static_cast<uint8_t>(code.length());
            }
        }, "PSK modulation code")
        .def("defaults", &Sonar::Acoustic::defaults, "Set the default settings")
        .def("to_dict", [](const Sonar::Acoustic& self)
        {
            py::dict d;
            d["tx_start_frequency"] = self.txStartFrequency;
            d["tx_end_frequency"] = self.txEndFrequency;
            d["tx_pulse_width_us"] = self.txPulseWidthUs;
            d["tx_pulse_amplitude"] = self.txPulseAmplitude;
            d["high_sample_rate"] = self.highSampleRate;
            d["psk_code"] = PySdk::asBinStr(self.pskCode, self.pskLength);
            return d;
        }, "Return the settings as a dictionary");

    py::class_<Sonar::Setup> setup(sonar, "Setup");
    setup.def(py::init<>())
        .def(py::init([](const py::dict& d) { return dictToSetupSettings(d, Sonar::Setup()); }))
        .def_readwrite("step_size", &Sonar::Setup::stepSize, "Angle the tranducer head should move between pings in units of 12800th. Positive values turn clockwise, negative anticlockwise. limits -6399 to 6399")
        .def_readwrite("sector_start", &Sonar::Setup::sectorStart, "Start angle of the sector. limmts 0 to 12799")
        .def_readwrite("sector_size", &Sonar::Setup::sectorSize, "Size of the sector. limits 0 to 12800")
        .def_readwrite("flyback_mode", &Sonar::Setup::flybackMode, "If true the transducer head returns back to either the sectorStart position when stepSize is positive, or ssectorStart + sectorSize when stepSize is negative")
        .def_readwrite("image_data_point", &Sonar::Setup::imageDataPoint, "Number of data points per ping between the range set by minRangeMm and maxRangeMm. limits 20 to 4096")
        .def_readwrite("min_range_mm", &Sonar::Setup::minRangeMm, "Start listening for echos after this range in millimeters")
        .def_readwrite("max_range_mm", &Sonar::Setup::maxRangeMm, "Listen for echos up until this range in millimeters")
        .def_readwrite("profiler_min_range_mm", &Sonar::Setup::profilerMinRangeMm, "Applies to profiling mode only. Start listening for echos after this range in millimeters")
        .def_readwrite("profiler_max_range_mm", &Sonar::Setup::profilerMaxRangeMm, "Applies to profiling mode only. Listen for echos up until this range in millimeters")  
        .def_readwrite("digital_gain", &Sonar::Setup::digitalGain, "Digital gain for the image data as a simple multiplier factor. limits 1 to 1000")
        .def_readwrite("echo_mode", &Sonar::Setup::echoMode, "Applies to profiling mode only. Selects which echo to report back as the chosen one when profiling")
        .def_readwrite("xc_threashold_low", &Sonar::Setup::xcThreasholdLow, "Applies to profiling mode only. Sets a lower limit on the quality of the return pulse. This ensures resilience to false echos. Value ranges from 0 to 1")
        .def_readwrite("xc_threashold_high", &Sonar::Setup::xcThreasholdHigh, "Applies to profiling mode only. When the return signal level drops bellow this value the end of an echo pulse is realised. Value ranges from 0 to 1")
        .def_readwrite("energy_threashold", &Sonar::Setup::energyThreashold, "Applies to profiling mode only. Minimum enery an echo must have to be reported. Range is 0 to 1")
        .def("defaults", &Sonar::Setup::defaults, "Set the default settings")
        .def("to_dict", [](const Sonar::Setup& self)
        {
            py::dict d;    
            d["step_size"] = self.stepSize;
            d["sector_start"] = self.sectorStart;
            d["sector_size"] = self.sectorSize;
            d["flyback_mode"] = self.flybackMode;
            d["image_data_point"] = self.imageDataPoint;
            d["min_range_mm"] = self.minRangeMm;
            d["max_range_mm"] = self.maxRangeMm;
            d["profiler_min_range_mm"] = self.profilerMinRangeMm;
            d["profiler_max_range_mm"] = self.profilerMaxRangeMm;
            d["digital_gain"] = self.digitalGain;
            d["echo_mode"] = py::cast(self.echoMode).attr("name").cast<std::string>();
            d["xc_threashold_low"] = self.xcThreasholdLow;
            d["xc_threashold_high"] = self.xcThreasholdHigh;
            d["energy_threashold"] = self.energyThreashold;            
            return d;
        }, "Return the settings as a dictionary");

    py::enum_<Sonar::Setup::EchoMode>(setup, "EchoMode")
        .value("First", Sonar::Setup::EchoMode::First)
        .value("Strongest", Sonar::Setup::EchoMode::Strongest)
        .value("All", Sonar::Setup::EchoMode::All);

    py::class_<Sonar::Settings> settings(sonar, "Settings");
    settings.def(py::init<>())
        .def_readwrite("system", &Sonar::Settings::system, "System settings")
        .def_readwrite("acoustic", &Sonar::Settings::acoustic, "Acoustic settings")
        .def_readwrite("setup", &Sonar::Settings::setup, "Setup settings")
        .def("defaults", &Sonar::Settings::defaults, "Set the default settings")
        .def("to_dict", [](const Sonar::Settings& self)
        {
            py::dict d;
            d["system"] = py::cast(self.system).attr("to_dict")();
            d["acoustic"] = py::cast(self.acoustic).attr("to_dict")();
            d["setup"] = py::cast(self.setup).attr("to_dict")();
            return d;
        }, "Return the settings as a dictionary");

    py::enum_<Sonar::Settings::Type>(settings, "Type")
        .value("System", Sonar::Settings::Type::System)
        .value("Acoustic", Sonar::Settings::Type::Acoustic)
        .value("Setup", Sonar::Settings::Type::Setup);

    py::class_<Sonar::SensorRates>(sonar, "SensorRates")
        .def(py::init<>())
        .def_readwrite("ahrs", &Sonar::SensorRates::ahrs, "Interval in milliseconds between AHRS data. Zero means no AHRS data")
        .def_readwrite("gyro", &Sonar::SensorRates::gyro, "Interval in milliseconds between gyro data. Zero means no gyro data")
        .def_readwrite("accel", &Sonar::SensorRates::accel, "Interval in milliseconds between accelerometer data. Zero means no accelerometer data")
        .def_readwrite("mag", &Sonar::SensorRates::mag, "Interval in milliseconds between magnetometer data. Zero means no magnetometer data")
        .def_readwrite("voltage_and_temp", &Sonar::SensorRates::voltageAndTemp, "Interval in milliseconds between system voltage and temperature data")
        .def("to_dict", [](const Sonar::SensorRates& self)
        {
            py::dict d;
            d["ahrs"] = self.ahrs;
            d["gyro"] = self.gyro;
            d["accel"] = self.accel;
            d["mag"] = self.mag;
            d["voltage_and_temp"] = self.voltageAndTemp;
            return d;
        }, "Return the settings as a dictionary");

    py::class_<Sonar::AhrsCal>(sonar, "AhrsCal")
        .def(py::init<>())
        .def_readwrite("gyro_bias", &Sonar::AhrsCal::gyroBias, "Gyro bias corrections in degress per second")
        .def_readwrite("accel_bias", &Sonar::AhrsCal::accelBias, "Accel bias corrections in G")
        .def_readwrite("mag_bias", &Sonar::AhrsCal::magBias, "Mag bias corrections in uT")
        .def_readwrite("accel_transform", &Sonar::AhrsCal::accelTransform, "Transformation matrix for accelerometer")
        .def_readwrite("mag_transform", &Sonar::AhrsCal::magTransform, "Transformation matrix for magnetometer");

    py::class_<Sonar::HeadIndexes> headIndexes(sonar, "HeadIndexes");
    headIndexes.def(py::init<>())
        .def_readwrite("state", &Sonar::HeadIndexes::state, "The state of the head indexes")
        .def_readwrite("slippage", &Sonar::HeadIndexes::slippage, "The amount of slippage since the last acquisition or bootup in units of 12800th. 360 degrees = a value of 12800.")
        .def_readwrite("std_deviation", &Sonar::HeadIndexes::stdDeviation, "The standard deviation of the indexes in units of 12800th")
        .def_readwrite("hysteresis_correction", &Sonar::HeadIndexes::hysteresisCorrection, "The amount of hysteresis correction applied to the indexes in units of 12800th")
        .def_readwrite("width_correction", &Sonar::HeadIndexes::widthCorrection, "The amount of width correction applied to the indexes in units of 12800th")
        .def_readwrite("indexes", &Sonar::HeadIndexes::indexes, "indexes of the markers")
        .def("to_dict", [](const Sonar::HeadIndexes& self)
        {
            py::dict d;
            py::list indexes;

            for (const Sonar::HeadIndexes::Index& index : self.indexes)
            {
                indexes.append(py::cast(index).attr("to_dict")());
            }
            d["state"] = py::cast(self.state).attr("name").cast<std::string>();
            d["slippage"] = self.slippage;
            d["std_deviation"] = self.stdDeviation;
            d["hysteresis_correction"] = self.hysteresisCorrection;
            d["width_correction"] = self.widthCorrection;
            d["indexes"] = indexes;
            return d;
        }, "Return the settings as a dictionary");

    py::enum_<Sonar::HeadIndexes::State>(headIndexes, "State")
        .value("Ok", Sonar::HeadIndexes::State::Ok)
        .value("ErrRotaion", Sonar::HeadIndexes::State::ErrRotaion)
        .value("ErrIdxCount", Sonar::HeadIndexes::State::ErrIdxCount)
        .value("ErrNoMatch", Sonar::HeadIndexes::State::ErrNoMatch);

    py::class_<Sonar::HeadIndexes::Index>(headIndexes, "HeadIndex")
        .def(py::init<>())
        .def_readwrite("idx", &Sonar::HeadIndexes::Index::idx, "The position of the index. (Units of 12800th)")
        .def_readwrite("level", &Sonar::HeadIndexes::Index::level, "The level of the index. True = low to high transition, false = high to low transition.")
        .def_readwrite("dir", &Sonar::HeadIndexes::Index::dir, "The direction the head was traveling when the index was aquired. True = clockwise, false = anticlockwise.")
        .def("to_dict", [](const Sonar::HeadIndexes::Index& self)
        {
            py::dict d;
            d["idx"] = self.idx;
            d["level"] = self.level;
            d["dir"] = self.dir;
            return d;
        }, "Return the settings as a dictionary");

    py::class_<Sonar::Ping>(sonar, "Ping", py::buffer_protocol())
        .def(py::init<>())
        .def_readwrite("angle", &Sonar::Ping::angle, "Angle the data was aquired at in units of 12800th. 360 degrees = a value of 12800")
        .def_readwrite("step_size", &Sonar::Ping::stepSize, "The step size setting at the time this data was aquired")
        .def_readwrite("min_range_mm", &Sonar::Ping::minRangeMm, "Start distance of the data in millimeters, data[0] is aquired at this range")
        .def_readwrite("max_range_mm", &Sonar::Ping::maxRangeMm, "Final distance of the data in millimeters, data[data.size()-1] is aquired at this range")
        .def_buffer([](Sonar::Ping& self) -> py::buffer_info 
        {
            return py::buffer_info(
                self.data.data(),
                sizeof(uint16_t),
                py::format_descriptor<uint16_t>::format(),
                1,
                { static_cast<uint32_t>(self.data.size()) },
                { sizeof(uint16_t) },
                true
            );
        })
        .def("to_dict", [](const Sonar::Ping& self)
        {
            py::dict d;
            d["angle"] = self.angle;
            d["step_size"] = self.stepSize;
            d["min_range_mm"] = self.minRangeMm;
            d["max_range_mm"] = self.maxRangeMm;
            d["data"] = self.data;
            return d;
        }, "Return the settings as a dictionary");

    py::class_<Sonar::Echos> echos(sonar, "Echos");
    echos.def(py::init<>())
        .def_readwrite("time_us", &Sonar::Echos::timeUs, "Time in microseconds of the start of the ping")
        .def_readwrite("angle", &Sonar::Echos::angle, "Angle the data was aquired at in units of 12800th. 360 degrees = a value of 12800")
        .def_readwrite("data", &Sonar::Echos::data, "Array of echos. Each echo represents a single target")
        .def("to_dict", [](const Sonar::Echos& self)
        {
            py::dict d;
            d["time_us"] = self.timeUs;
            d["angle"] = self.angle;
            py::list echoes;
            for (auto& echo : self.data)
            {
                echoes.append(py::cast(echo).attr("to_dict")());
            }
            d["data"] = echoes;
            return d;
        }, "Return the settings as a dictionary");

    py::class_<Sonar::Echos::Echo> (echos, "Echo")
        .def(py::init<>())
        .def_readwrite("total_tof", &Sonar::Echos::Echo::totalTof, "Total time of flight in seconds to the target and back")
        .def_readwrite("correlation", &Sonar::Echos::Echo::correlation, "How well the received echo correlates 0 to 1")
        .def_readwrite("signal_energy", &Sonar::Echos::Echo::signalEnergy, "Normalised energy level of the echo 0 to 1")
        .def("to_dict", [](const Sonar::Echos::Echo& self)
        {
            py::dict d;
            d["total_tof"] = self.totalTof;
            d["correlation"] = self.correlation;
            d["signal_energy"] = self.signalEnergy;
            return d;
        }, "Return the settings as a dictionary");

    py::class_<Sonar::CpuPowerTemp>(sonar, "CpuPowerTemp")
        .def(py::init<>())
        .def_readwrite("core1_v0", &Sonar::CpuPowerTemp::core1V0, "CPU Core voltage, should be 1V")
        .def_readwrite("aux1_v8", &Sonar::CpuPowerTemp::aux1V8, "Auxillary voltage, should be 1.8V")
        .def_readwrite("ddr1_v35", &Sonar::CpuPowerTemp::ddr1V35, "DDR voltage, should be 1.35V")
        .def_readwrite("cpu_temperature", &Sonar::CpuPowerTemp::cpuTemperature, "CPU temperature in degrees C")
        .def_readwrite("aux_temperature", &Sonar::CpuPowerTemp::auxTemperature, "Auxillary temperature in degrees C")
        .def("__repr__", [](const Sonar::CpuPowerTemp& self) {
            return "1.0V: " + StringUtils::toStr(self.core1V0) + "\n" + 
                   "1.8V: " + StringUtils::toStr(self.aux1V8) + "\n" + 
                   "1.35V: " + StringUtils::toStr(self.ddr1V35) + "\n" +
                   "CPU Temp: " + StringUtils::toStr(self.cpuTemperature) + "\n" +
                   "Aux Temp: " + StringUtils::toStr(self.auxTemperature);
        })
        .def("to_dict", [](const Sonar::CpuPowerTemp& self)
        {
            py::dict d;
            d["core1_v0"] = self.core1V0;
            d["aux1_v8"] = self.aux1V8;
            d["ddr1_v35"] = self.ddr1V35;
            d["cpu_temperature"] = self.cpuTemperature;
            d["aux_temperature"] = self.auxTemperature;
            return d;
        }, "Return the settings as a dictionary");

    
}
//--------------------------------------------------------------------------------------------------
Sonar::System dictToSystemSettings(const py::dict& d, const Sonar::System& defualt)
{
    bool_t err = false;
    Sonar::System settings;

    settings.uartMode = PySdk::getDictValue(d, "uart_mode", defualt.uartMode);
    settings.baudrate = PySdk::getDictValue(d, "baudrate", defualt.baudrate);
    settings.ipAddress = StringUtils::toIp(PySdk::getDictValue(d, "ip_address", StringUtils::ipToStr(defualt.ipAddress)), err);
    settings.netmask = StringUtils::toIp(PySdk::getDictValue(d, "netmask", StringUtils::ipToStr(defualt.netmask)), err);
    settings.gateway = StringUtils::toIp(PySdk::getDictValue(d, "gateway", StringUtils::ipToStr(defualt.gateway)), err);
    settings.port = PySdk::getDictValue(d, "port", defualt.port);
    settings.phyPortMode = PySdk::getDictValue(d, "phy_port_mode", defualt.phyPortMode);
    settings.phyMdixMode = PySdk::getDictValue(d, "phy_mdix_mode", defualt.phyMdixMode);
    settings.useDhcp = PySdk::getDictValue(d, "use_dhcp", defualt.useDhcp);
    settings.invertHeadDirection = PySdk::getDictValue(d, "invert_head_direction", defualt.invertHeadDirection);
    settings.ahrsMode = PySdk::getDictValue(d, "ahrs_mode", defualt.ahrsMode);
    settings.orientationOffset = PySdk::getDictValue(d, "orientation_offset", defualt.orientationOffset);
    settings.headingOffsetRad = PySdk::getDictValue(d, "heading_offset_rad", defualt.headingOffsetRad);
    settings.turnsAbout = PySdk::getDictValue(d, "turns_about", defualt.turnsAbout);
    settings.turnsAboutEarthFrame = PySdk::getDictValue(d, "turns_about_earth_frame", defualt.turnsAboutEarthFrame);
    settings.useXcNorm = PySdk::getDictValue(d, "use_xc_norm", defualt.useXcNorm);
    settings.data8Bit = PySdk::getDictValue(d, "data_8_bit", defualt.data8Bit);
    settings.gatingMode = getFromDict(d, "gating_mode", defualt.gatingMode);
    settings.gatingAngle = PySdk::getDictValue(d, "gating_angle", defualt.gatingAngle);
    settings.speedOfSound = PySdk::getDictValue(d, "speed_of_sound", defualt.speedOfSound);  
    return settings;
}
//--------------------------------------------------------------------------------------------------
Sonar::Acoustic dictToAcousticSettings(const py::dict& d, const Sonar::Acoustic& defualt)
{
    Sonar::Acoustic settings;

    settings.txStartFrequency = PySdk::getDictValue(d, "tx_start_frequency", defualt.txStartFrequency);
    settings.txEndFrequency = PySdk::getDictValue(d, "tx_end_frequency", defualt.txEndFrequency);
    settings.txPulseWidthUs = PySdk::getDictValue(d, "tx_pulse_width_us", defualt.txPulseWidthUs);
    settings.txPulseAmplitude = PySdk::getDictValue(d, "tx_pulse_amplitude", defualt.txPulseAmplitude);
    settings.highSampleRate = PySdk::getDictValue(d, "high_sample_rate", defualt.highSampleRate);
    std::string code = PySdk::getDictValue(d, "psk_code", PySdk::asBinStr(defualt.pskCode, defualt.pskLength));
    settings.pskCode = static_cast<uint32_t>(PySdk::binStrToValue(code));
    settings.pskLength = static_cast<uint8_t>(code.length());
    return settings;
}
//--------------------------------------------------------------------------------------------------
Sonar::Setup dictToSetupSettings(const py::dict& d, const Sonar::Setup& defualt)
{
    Sonar::Setup settings;
    settings.stepSize = PySdk::getDictValue(d, "step_size", defualt.stepSize);
    settings.sectorStart = PySdk::getDictValue(d, "sector_start", defualt.sectorStart);
    settings.sectorSize = PySdk::getDictValue(d, "sector_size", defualt.sectorSize);
    settings.flybackMode = PySdk::getDictValue(d, "flyback_mode", defualt.flybackMode);
    settings.imageDataPoint = PySdk::getDictValue(d, "image_data_point", defualt.imageDataPoint);
    settings.minRangeMm = PySdk::getDictValue(d, "min_range_mm", defualt.minRangeMm);
    settings.maxRangeMm = PySdk::getDictValue(d, "max_range_mm", defualt.maxRangeMm);
    settings.profilerMinRangeMm = PySdk::getDictValue(d, "profiler_min_range_mm", defualt.profilerMinRangeMm);
    settings.profilerMaxRangeMm = PySdk::getDictValue(d, "profiler_max_range_mm", defualt.profilerMaxRangeMm);
    settings.digitalGain = PySdk::getDictValue(d, "digital_gain", defualt.digitalGain);
    settings.echoMode = getFromDict(d, "echo_mode", defualt.echoMode);
    settings.xcThreasholdLow = PySdk::getDictValue(d, "xc_threashold_low", defualt.xcThreasholdLow);
    settings.xcThreasholdHigh = PySdk::getDictValue(d, "xc_threashold_high", defualt.xcThreasholdHigh);
    settings.energyThreashold = PySdk::getDictValue(d, "energy_threashold", defualt.energyThreashold);
    return settings;
}
//--------------------------------------------------------------------------------------------------
Sonar::System::GatingMode getFromDict(const py::dict& d, const char* key, Sonar::System::GatingMode defaultValue)
{
    if (d.contains(key))
    {
        if (py::isinstance<py::str>(d[key]))
        {
            std::string str = d[key].cast<std::string>();
            if (StringUtils::compareNoCase(str, "off"))
            {
                return Sonar::System::GatingMode::Off;
            }
            else if (StringUtils::compareNoCase(str, "on"))
            {
                return Sonar::System::GatingMode::On;
            }
            else if (StringUtils::compareNoCase(str, "rollcomp"))
            {
                return Sonar::System::GatingMode::RollComp;
            }
            else
            {
                throw py::value_error("Invalid value: " + d[key].cast<std::string>() +" for key " + std::string(key));
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
Sonar::Setup::EchoMode getFromDict(const py::dict& d, const char* key, Sonar::Setup::EchoMode defaultValue)
{
     if (d.contains(key))
    {
        if (py::isinstance<py::str>(d[key]))
        {
            std::string str = d[key].cast<std::string>();
            if (StringUtils::compareNoCase(str, "first"))
            {
                return Sonar::Setup::EchoMode::First;
            }
            else if (StringUtils::compareNoCase(str, "strongest"))
            {
                return Sonar::Setup::EchoMode::Strongest;
            }
            else if (StringUtils::compareNoCase(str, "all"))
            {
                return Sonar::Setup::EchoMode::All;
            }
            else
            {
                throw py::value_error("Invalid value: " + d[key].cast<std::string>() +" for key " + std::string(key));
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