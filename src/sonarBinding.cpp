//------------------------------------------ Includes ----------------------------------------------

#include "sonarBinding.h"
#include "devices/sonar.h"
#include "helpers/sonarImage.h"
#include "helpers/palette.h"
#include "helpers/sonarDataStore.h"
#include "utils/stringUtils.h"
#include "files/bmpFile.h"
#include <pybind11/stl.h>

using namespace pybind11::literals;
using namespace IslSdk;

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
        .def(py::init<int32_t, int32_t, bool_t, bool_t>(), "width"_a, "height"_a, "use4BytePixel"_a = true, "useBilinerInterpolation"_a = true)
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
        .def("set_buffer", &SonarImage::setBuffer, "Set the buffer", "width"_a, "height"_a, "use4BytePixel"_a)
        .def("set_sector_area", &SonarImage::setSectorArea, "Set the sector area", "min_range_mm"_a, "max_range_mm"_a, "sector_start"_a, "sector_size"_a)
        .def("render", &SonarImage::render, "Render the image", "data"_a, "palette"_a, "re_draw"_a = false)
        .def("render_16_bit", &SonarImage::render16Bit, "Render the image", "data"_a, "re_draw"_a = false)
        .def("render_texture", &SonarImage::renderTexture, "Render the image", "data"_a, "palette"_a, "re_draw"_a = false)
        .def("render_texture_16_bit", &SonarImage::renderTexture16Bit, "Render the image", "data"_a, "re_draw"_a = false)
        .def("save_bmp", [] (const SonarImage& self, const std::string& fileName)
        {
            BmpFile::save(fileName, reinterpret_cast<const uint32_t*>(&self.buf[0]), 32, self.width, self.height);
        }, "Save the image to a BMP file", "file_name"_a);

    py::class_<SonarDataStore>(m, "SonarDataStore")
        .def(py::init<>())
        .def_property_readonly("sector", [](const SonarDataStore& self) { return self.sector; })
        .def_property_readonly("ping_data", [](const SonarDataStore& self) { return self.pingData; })
        .def("add", &SonarDataStore::add, "Add ping data", "ping"_a, "blank_range_mm"_a = 0)
        .def("clear", &SonarDataStore::clear, "Clear ping data", "start_angle"_a = 0, "angle_size"_a = Sonar::maxAngle)
        .def("render_complete", &SonarDataStore::renderComplete, "Render complete");

    py::class_<Sonar, Device, std::shared_ptr<Sonar>> sonar(m, "Sonar");
    sonar.def(py::init<const Device::Info&>())
        .def_readonly_static("max_angle", &Sonar::maxAngle, "Get the maximum angle")
        .def_property_readonly("settings", [](const Sonar& self) { return self.settings; })
        .def_property_readonly("sensor_rates", [](const Sonar& self) { return self.sensorRates; })
        .def_property_readonly("tvg_points", [](const Sonar& self) { return self.tvgPoints; })
        .def_property_readonly("mac_address", [](const Sonar& self) { return self.macAddress; })
        .def_readonly("ahrs", &Sonar::ahrs, "AHRS sensor")
        .def_readonly("gyro", &Sonar::gyro, "Gyro sensor")
        .def_readonly("accel", &Sonar::accel, "Accelerometer sensor")
        .def_readonly("mag", &Sonar::mag, "Magnetometer sensor")
        .def_readonly("on_settings_updated", &Sonar::onSettingsUpdated)
        .def_readonly("on_head_homed", &Sonar::onHeadHomed)
        .def_readonly("on_ping_data", &Sonar::onPingData)
        .def_readonly("on_echo_data", &Sonar::onEchoData)
        .def_readonly("on_pwr_and_temp", &Sonar::onPwrAndTemp)
        .def("set_sensor_rates", &Sonar::setSensorRates, "Set the sensor rates", "sensors"_a)
        .def("set_system_settings", &Sonar::setSystemSettings, "Set the system settings", "settings"_a, "save"_a = true)
        .def("set_acoustic_settings", &Sonar::setAcousticSettings, "Set the acoustic settings", "settings"_a, "save"_a = true)
        .def("set_setup_settings", &Sonar::setSetupSettings, "Set the setup settings", "settings"_a, "save"_a = true)
        .def("home_head", &Sonar::homeHead, "Home the head")
        .def("start_scanning", &Sonar::startScanning, "Start scanning")
        .def("stop_scanning", &Sonar::stopScanning, "Stop scanning")
        .def("move_head", &Sonar::moveHead, "Move the head", "angle"_a, "relative"_a = false)
        .def("test_pattern", &Sonar::testPattern, "Test pattern", "enable"_a)
        .def("set_tvg", &Sonar::setTvg, "Set the TVG curve", "points"_a)
        .def_static("get_default_tvg", &Sonar::getDefaultTvg, "Get the default TVG curve")
        .def_static("load_config", &Sonar::loadConfig, "Load the configuration", "fileName"_a, "info"_a = nullptr, "settings"_a = nullptr, "cal"_a = nullptr, "tvgPoints"_a = nullptr)
        .def("has_ahrs", &Sonar::hasAhrs, "Has AHRS")
        .def("is_hd", &Sonar::isHd, "Is HD")
        .def("is_profiler", &Sonar::isProfiler, "Is Profiler");

    //----------------------------------------- Signals --------------------------------------------
    
    py::class_<Signal<Sonar&, bool_t, Sonar::Settings::Type>>(sonar, "Signal_onSettingsUpdated")
        .def("connect", &Signal<Sonar&, bool_t, Sonar::Settings::Type>::pyConnect, "Connect to the signal with a callback_function(sonar, success, type)")
        .def("disconnect", &Signal<Sonar&, bool_t, Sonar::Settings::Type>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Sonar&, const Sonar::HeadHome&>>(sonar, "Signal_onHeadHomed")
        .def("connect", &Signal<Sonar&, const Sonar::HeadHome&>::pyConnect, "Connect to the signal with a callback_function(sonar, homeInfo)")
        .def("disconnect", &Signal<Sonar&, const Sonar::HeadHome&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Sonar&, const Sonar::Ping&>>(sonar, "Signal_onPingData")
        .def("connect", &Signal<Sonar&, const Sonar::Ping&>::pyConnect, "Connect to the signal with a callback_function(sonar, ping)")
        .def("disconnect", &Signal<Sonar&, const Sonar::Ping&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Sonar&, const Sonar::Echos&>>(sonar, "Signal_onEchoData")
        .def("connect", &Signal<Sonar&, const Sonar::Echos&>::pyConnect, "Connect to the signal with a callback_function(sonar, echos)")
        .def("disconnect", &Signal<Sonar&, const Sonar::Echos&>::pyDisconnect, "Disconnect a callback function from the signal");

    py::class_<Signal<Sonar&, const Sonar::CpuPowerTemp&>>(sonar, "Signal_onPwrAndTemp")
        .def("connect", &Signal<Sonar&, const Sonar::CpuPowerTemp&>::pyConnect, "Connect to the signal with a callback_function(sonar, status)")
        .def("disconnect", &Signal<Sonar&, const Sonar::CpuPowerTemp&>::pyDisconnect, "Disconnect a callback function from the signal");

    
    //----------------------------------------- Structs --------------------------------------------

    py::class_<Sonar::Sector>(sonar, "Sector")
        .def(py::init<>())
        .def_readwrite("start", &Sonar::Sector::start, "Start angle of the sector")
        .def_readwrite("size", &Sonar::Sector::size, "Size of the sector");

    py::class_<Sonar::System> system(sonar, "System");
    system.def(py::init<>())
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
        .def_readwrite("echo_mode", &Sonar::System::echoMode, "Applies to profiling mode only. Selects which echo to report back as the chosen one when profiling")
        .def_readwrite("xc_threashold_low", &Sonar::System::xcThreasholdLow, "Applies to profiling mode only. Sets a lower limit on the quality of the return pulse. This ensures resilience to false echos. Value ranges from 0 to 1")
        .def_readwrite("xc_threashold_high", &Sonar::System::xcThreasholdHigh, "Applies to profiling mode only. When the return signal level drops bellow this value the end of an echo pulse is realised. Value ranges from 0 to 1")
        .def_readwrite("energy_threashold", &Sonar::System::energyThreashold, "Applies to profiling mode only. Minimum enery an echo must have to be reported. Range is 0 to 1")
        .def_readwrite("use_tilt_correction", &Sonar::System::useTiltCorrection, "Applies to profiling mode only. Not implemented yet")
        .def_readwrite("profiler_depth_gating", &Sonar::System::profilerDepthGating, "Applies to profiling mode only. Not implemented yet")
        .def_readwrite("profiler_min_range_mm", &Sonar::System::profilerMinRangeMm, "Applies to profiling mode only. Start listening for echos after this range in millimeters")
        .def_readwrite("profiler_max_range_mm", &Sonar::System::profilerMaxRangeMm, "Applies to profiling mode only. Listen for echos up until this range in millimeters")
        .def("defaults", &Sonar::System::defaults, "Set the default settings");

    py::enum_<Sonar::System::EchoMode>(system, "EchoMode")
        .value("First", Sonar::System::EchoMode::First)
        .value("Strongest", Sonar::System::EchoMode::Strongest)
        .value("All", Sonar::System::EchoMode::All);

    py::class_<Sonar::Acoustic> acoustic(sonar, "Acoustic");
    acoustic.def(py::init<>())
        .def_readwrite("tx_start_frequency", &Sonar::Acoustic::txStartFrequency, "Transmit pulse start frequency in hertz")
        .def_readwrite("tx_end_frequency", &Sonar::Acoustic::txEndFrequency, "Transmit pulse end frequency in hertz")
        .def_readwrite("tx_pulse_width_us", &Sonar::Acoustic::txPulseWidthUs, "Transmit pulse length in micro seconds")
        .def_readwrite("tx_pulse_amplitude", &Sonar::Acoustic::txPulseAmplitude, "Transmit pulse amplitude as a percent 0% to 100%")
        .def_readwrite("high_sample_rate", &Sonar::Acoustic::highSampleRate, "If true the ADC sample rate is 5 MHz else it's 2.5 MHz")
        .def_readwrite("psk_mode", &Sonar::Acoustic::pskMode, "PSK modulation mode")
        .def("defaults", &Sonar::Acoustic::defaults, "Set the default settings");

    py::enum_<Sonar::Acoustic::PskMode>(acoustic, "PskMode")
        .value("Off", Sonar::Acoustic::PskMode::Off)
        .value("Code1", Sonar::Acoustic::PskMode::Code1)
        .value("Code2", Sonar::Acoustic::PskMode::Code2)
        .value("Code3", Sonar::Acoustic::PskMode::Code3)
        .value("Code4", Sonar::Acoustic::PskMode::Code4);

    py::class_<Sonar::Setup>(sonar, "Setup")
        .def(py::init<>())
        .def_readwrite("digital_gain", &Sonar::Setup::digitalGain, "Digital gain for the image data as a simple multiplier factor. limits 1 to 1000")
        .def_readwrite("speed_of_sound", &Sonar::Setup::speedOfSound, "Speed of sound in meters per second. limits 1000 to 2500")
        .def_readwrite("max_range_mm", &Sonar::Setup::maxRangeMm, "Listen for echos up until this range in millimeters")
        .def_readwrite("min_range_mm", &Sonar::Setup::minRangeMm, "Start listening for echos after this range in millimeters")
        .def_readwrite("step_size", &Sonar::Setup::stepSize, "Angle the tranducer head should move between pings in units of 12800th. Positive values turn clockwise, negative anticlockwise. limits -6399 to 6399")
        .def_readwrite("sector_start", &Sonar::Setup::sectorStart, "Start angle of the sector. limmts 0 to 12799")
        .def_readwrite("sector_size", &Sonar::Setup::sectorSize, "Size of the sector. limits 0 to 12800")
        .def_readwrite("flyback_mode", &Sonar::Setup::flybackMode, "If true the transducer head returns back to either the sectorStart position when stepSize is positive, or ssectorStart + sectorSize when stepSize is negative")
        .def_readwrite("image_data_point", &Sonar::Setup::imageDataPoint, "Number of data points per ping between the range set by minRangeMm and maxRangeMm. limits 20 to 4096")
        .def_readwrite("data_8_bit", &Sonar::Setup::data8Bit, "true = 8-bit data, false = 16-bit data")
        .def("defaults", &Sonar::Setup::defaults, "Set the default settings");

    py::class_<Sonar::Settings> settings(sonar, "Settings");
    settings.def(py::init<>())
        .def_readwrite("system", &Sonar::Settings::system, "System settings")
        .def_readwrite("acoustic", &Sonar::Settings::acoustic, "Acoustic settings")
        .def_readwrite("setup", &Sonar::Settings::setup, "Setup settings")
        .def("defaults", &Sonar::Settings::defaults, "Set the default settings");

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
        .def_readwrite("voltage_and_temp", &Sonar::SensorRates::voltageAndTemp, "Interval in milliseconds between system voltage and temperature data");

    py::class_<Sonar::AhrsCal>(sonar, "AhrsCal")
        .def(py::init<>())
        .def_readwrite("gyro_bias", &Sonar::AhrsCal::gyroBias, "Gyro bias corrections in degress per second")
        .def_readwrite("accel_bias", &Sonar::AhrsCal::accelBias, "Accel bias corrections in G")
        .def_readwrite("mag_bias", &Sonar::AhrsCal::magBias, "Mag bias corrections in uT")
        .def_readwrite("accel_transform", &Sonar::AhrsCal::accelTransform, "Transformation matrix for accelerometer")
        .def_readwrite("mag_transform", &Sonar::AhrsCal::magTransform, "Transformation matrix for magnetometer");

    py::class_<Sonar::HeadHome> headhome(sonar, "HeadHome");
    headhome.def(py::init<>())
        .def_readwrite("state", &Sonar::HeadHome::state, "State of the homing process");

    py::enum_<Sonar::HeadHome::HeadHomeState>(headhome, "HeadHomeState")
        .value("Ok", Sonar::HeadHome::HeadHomeState::OK)
        .value("Error_e1_e2", Sonar::HeadHome::HeadHomeState::Error_E1_E2)
        .value("Error_e2", Sonar::HeadHome::HeadHomeState::Error_E2)
        .value("Error_e1", Sonar::HeadHome::HeadHomeState::Error_E1)
        .value("Error", Sonar::HeadHome::HeadHomeState::Error);

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
        });

    py::class_<Sonar::Echos> echos(sonar, "Echos");
    echos.def(py::init<>())
        .def_readwrite("time_us", &Sonar::Echos::timeUs, "Time in microseconds of the start of the ping")
        .def_readwrite("angle", &Sonar::Echos::angle, "Angle the data was aquired at in units of 12800th. 360 degrees = a value of 12800")
        .def_readwrite("min_range_mm", &Sonar::Echos::minRangeMm, "Start distance of the data in millimeters, data[0] is aquired at this range")
        .def_readwrite("max_range_mm", &Sonar::Echos::maxRangeMm, "Final distance of the data in millimeters, data[dataCount-1] is aquired at this range")
        .def_readwrite("data", &Sonar::Echos::data, "Array of echos. Each echo represents a single target");

    py::class_<Sonar::Echos::Echo> (echos, "Echo")
        .def(py::init<>())
        .def_readwrite("total_tof", &Sonar::Echos::Echo::totalTof, "Total time of flight in seconds to the target and back")
        .def_readwrite("correlation", &Sonar::Echos::Echo::correlation, "How well the received echo correlates 0 to 1")
        .def_readwrite("signal_energy", &Sonar::Echos::Echo::signalEnergy, "Normalised energy level of the echo 0 to 1");

    py::class_<Sonar::CpuPowerTemp>(sonar, "CpuPowerTemp")
        .def(py::init<>())
        .def_readwrite("core1_v0", &Sonar::CpuPowerTemp::core1V0, "CPU Core voltage, should be 1V")
        .def_readwrite("aux1_v8", &Sonar::CpuPowerTemp::aux1V8, "Auxillary voltage, should be 1.8V")
        .def_readwrite("ddr1_v35", &Sonar::CpuPowerTemp::ddr1V35, "DDR voltage, should be 1.35V")
        .def_readwrite("cpu_temperature", &Sonar::CpuPowerTemp::cpuTemperature, "CPU temperature in degrees C")
        .def_readwrite("aux_temperature", &Sonar::CpuPowerTemp::auxTemperature, "Auxillary temperature in degrees C");

    
}
//--------------------------------------------------------------------------------------------------