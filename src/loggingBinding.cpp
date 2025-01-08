//------------------------------------------ Includes ----------------------------------------------

#include "loggingBinding.h"
#include "files/logFile.h"
#include "logging/loggingDevice.h"
#include "logging/logWriter.h"
#include "logging/logReader.h"
#include "logging/logPlayer.h"

#include "devices/device.h"
#include "devices/isa500.h"
#include "devices/isd4000.h"
#include "devices/ism3d.h"
#include "devices/sonar.h"

#include <pybind11/stl.h>

using namespace pybind11::literals;
using namespace IslSdk;

//--------------------------------------------------------------------------------------------------
void PySdk::initLogging(py::module &m)
{
    py::class_<LoggingDevice, std::shared_ptr<LoggingDevice>>(m, "LoggingDevice")
        .def("set_logger", [](LoggingDevice& self, std::shared_ptr<LogWriter>& logger, const py::bytes& data)
        {
            uint_t size = PyBytes_Size(data.ptr());
            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(PyBytes_AsString(data.ptr()));
            self.setLogger(logger, ptr, size);
            
        }, "Set the logger", "logger"_a, "track_data"_a)
        .def("log", [](LoggingDevice& self, const py::bytes& data, uint8_t dataType, bool_t canSkip=true)
        {
            uint_t size = PyBytes_Size(data.ptr());
            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(PyBytes_AsString(data.ptr()));
            self.log(ptr, size, dataType, canSkip);
        }, "Log data", "data"_a, "data_type"_a, "can_skip"_a=true)
        .def("start_logging", &LoggingDevice::startLogging, "Start logging")
        .def("stop_logging", &LoggingDevice::stopLogging, "Stop logging")
        .def("is_logging", &LoggingDevice::islogging, "Is logging");
    
    py::class_<LogFile> logfile(m, "LogFile");
    
    py::class_<LogFile::RecordHeader> recordHeader(logfile, "RecordHeader");
        recordHeader.def_readonly("time_ms", &LogFile::RecordHeader::timeMs)
        .def_readonly("track_id", &LogFile::RecordHeader::trackId)
        .def_readonly("can_skip", &LogFile::RecordHeader::canSkip)
        .def_readonly("record_type", &LogFile::RecordHeader::recordType)
        .def_readonly("data_size", &LogFile::RecordHeader::dataSize)
        .def_readonly("data_type", &LogFile::RecordHeader::dataType);

    py::enum_<LogFile::RecordHeader::Type>(recordHeader, "RecordType")
        .value("Meta", LogFile::RecordHeader::Type::Meta)
        .value("Data", LogFile::RecordHeader::Type::Data)
        .value("Index", LogFile::RecordHeader::Type::Index)
        .value("Track", LogFile::RecordHeader::Type::Track);

    py::class_<LogFile::Track>(logfile, "Track")
        .def_readonly("id", &LogFile::Track::id)
        .def_readonly("file_offset", &LogFile::Track::fileOffset)
        .def_readonly("start_index", &LogFile::Track::startIndex)
        .def_readonly("record_count", &LogFile::Track::recordCount)
        .def_readonly("start_time", &LogFile::Track::startTime)
        .def_readonly("duration_ms", &LogFile::Track::durationMs)
        .def_readonly("data_type", &LogFile::Track::dataType)
        .def_readonly("data", &LogFile::Track::data);

    //---------------------------------------- Log Writer ------------------------------------------
    py::class_<LogWriter, std::shared_ptr<LogWriter>> logWriter(m, "LogWriter");
        logWriter.def(py::init<>())
        .def("start_new_file", &LogWriter::startNewFile, "Start a new file", "file_name"_a)
        .def("add_track", [](LogWriter& self, uint8_t dataType, const py::bytes& data)
        {
            uint_t size = PyBytes_Size(data.ptr());
            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(PyBytes_AsString(data.ptr()));
            return self.addTrack(dataType, ptr, size);
        }, "Add a track", "data_type"_a, "data"_a)
        .def("add_track_data", [](LogWriter& self, uint8_t trackId, const py::bytes& data, uint8_t dataType, bool_t canSkip=true)
        {
            uint_t size = PyBytes_Size(data.ptr());
            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(PyBytes_AsString(data.ptr()));
            return self.addTrackData(trackId, ptr, size, dataType, canSkip);
        }, "Add track data", "track_id"_a, "data"_a, "data_type"_a, "can_skip"_a)
        .def("close", &LogWriter::close, "Close the file")
        .def("set_max_file_size", &LogWriter::setMaxFileSize, "Set the max file size", "max_size"_a)
        .def_property_readonly("record_count", &LogWriter::recordCount)
        .def_property_readonly("time_ms", &LogWriter::timeMs)
        .def_property_readonly("duration_ms", &LogWriter::durationMs)
        .def_readonly("on_max_file_size", &LogWriter::onMaxFileSize);

    py::class_<Signal<LogWriter&>>(logWriter, "SignalOnMaxFileSize") 
        .def("connect", &Signal<LogWriter&>::pyConnect)
        .def("disconnect", &Signal<LogWriter&>::pyDisconnect);

    //---------------------------------------- Log Reader ------------------------------------------
    py::class_<LogReader, LogWriter, std::shared_ptr<LogReader>> logReader(m, "LogReader");
    logReader.def(py::init<>())
        .def("open", &LogReader::open, "Open the log file", "file_name"_a)
        .def("play", &LogReader::play, "Play the log file", "play_speed"_a)
        .def("reset", &LogReader::reset, "Reset the log file")
        .def("seek", &LogReader::seek, "Seek to a record", "index"_a)
        .def("process", &LogReader::process, "Run the log file")
        .def_readonly("on_record", &LogReader::onRecord)
        .def_readonly("on_error", &LogReader::onError);

    //----------------------------------------- Signals --------------------------------------------
    py::class_<Signal<LogReader&, const LogReader::RecordData&>>(logReader, "SignalOnRecord") 
        .def("connect", &Signal<LogReader&, const LogReader::RecordData&>::pyConnect)
        .def("disconnect", &Signal<LogReader&, const LogReader::RecordData&>::pyDisconnect);

    py::class_<Signal<LogReader&, const std::string&>>(logReader, "SignalOnError")
        .def("connect", &Signal<LogReader&, const std::string&>::pyConnect)
        .def("disconnect", &Signal<LogReader&, const std::string&>::pyDisconnect);

    py::class_<LogReader::RecordData>(logReader, "RecordData")
        .def_readonly("track_id", &LogReader::RecordData::trackId)
        .def_readonly("time_ms", &LogReader::RecordData::timeMs)
        .def_readonly("record_index", &LogReader::RecordData::recordIndex)
        .def_property_readonly("record_type", [](const LogReader::RecordData& self) {return static_cast<uint_t>(self.recordType);})
        .def_readonly("data_type", &LogReader::RecordData::dataType)
        .def_readonly("data", &LogReader::RecordData::data);

    //---------------------------------------- Log Player ------------------------------------------
    py::class_<LogPlayer, LogReader, std::shared_ptr<LogPlayer>> logPlayer(m, "LogPlayer");
    logPlayer.def(py::init<>())
        .def("get_device_from_track", [](LogPlayer& self, const LogFile::Track& track)
        {
            std::shared_ptr<Device> device;
            Device::Info deviceInfo;
            if (self.getIslDeviceTrackData(track, deviceInfo))
            {
                switch (deviceInfo.pid)
                {
                case Device::Pid::Isa500:
                {
                    device = std::make_shared<Isa500>(deviceInfo);
                    break;
                }
                case Device::Pid::Isd4000:
                {
                    device = std::make_shared<Isd4000>(deviceInfo);
                    break;
                }
                case Device::Pid::Ism3d:
                {
                    device = std::make_shared<Ism3d>(deviceInfo);
                    break;
                }
                case Device::Pid::Sonar:
                {
                    device = std::make_shared<Sonar>(deviceInfo);
                    break;
                }
                default:
                    break;
                };

                if (device)
                {
                    self.addDevice(device, track);
                }
            } 
            return device;
        }, "Add a device from a track", "track"_a)
        .def_readonly("on_new_track", &LogPlayer::onNewTrack);

    //----------------------------------------- Signals --------------------------------------------
    py::class_<Signal<LogPlayer&, const LogFile::Track&>>(logPlayer, "SignalOnNewTrack")
        .def("connect", &Signal<LogPlayer&, const LogFile::Track&>::pyConnect)
        .def("disconnect", &Signal<LogPlayer&, const LogFile::Track&>::pyDisconnect);
}
//--------------------------------------------------------------------------------------------------