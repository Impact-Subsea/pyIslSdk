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
            
        }, "Set the logger", "logger"_a, "trackData"_a)
        .def("log", [](LoggingDevice& self, const py::bytes& data, uint8_t dataType, bool_t canSkip=true)
        {
            uint_t size = PyBytes_Size(data.ptr());
            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(PyBytes_AsString(data.ptr()));
            self.log(ptr, size, dataType, canSkip);
        }, "Log data", "data"_a, "dataType"_a, "canSkip"_a=true)
        .def("start_logging", &LoggingDevice::startLogging, "Start logging")
        .def("stop_logging", &LoggingDevice::stopLogging, "Stop logging")
        .def("is_logging", &LoggingDevice::islogging, "Is logging");

    //---------------------------------------- Log File --------------------------------------------
    py::class_<LogFile> logfile(m, "LogFile");
    logfile.def(py::init<>())
        .def("open", &LogFile::open, "Open a log file", "fileName"_a)
        .def("repair", &LogFile::repair, "Repair a log file", "fileName"_a)
        .def("start_new", &LogFile::startNew, "Start a new log file", "fileName"_a, "timeMs"_a)
        .def("add_track", [](LogFile& self, uint8_t trackId, uint8_t dataType, const std::string& data)
        {
            return self.addTrack(trackId, dataType, reinterpret_cast<const uint8_t*>(&data[0]), data.size());
        }, "Add a track", "trackId"_a, "dataType"_a, "data"_a)
        .def("log_record", [](LogFile& self, uint8_t trackId, uint32_t timeMs, uint8_t dataType, bool_t canSkip, const std::string& data)
        {
            LogFile::RecordHeader recordHeader = { timeMs, trackId, canSkip, LogFile::RecordHeader::Type::Data, static_cast<uint32_t>(data.size()), dataType };
            return self.logRecord(*self.findTrack(trackId), recordHeader, reinterpret_cast<const uint8_t*>(&data[0]));
        }, "Log a record", "trackId"_a, "timeMs"_a, "dataType"_a, "canSkip"_a, "data"_a)
        .def("read_record_header", &LogFile::readRecordHeader, "Read a record header", "recordHeader"_a, "position"_a)
        .def("read_record_data", &LogFile::readRecordData, "Read a record data", "data"_a, "size"_a)
        .def("close", &LogFile::close, "Close the log file")
        .def("find_track", &LogFile::findTrack, "Find a track", "id"_a)
        
        .def_property_readonly("time_ms", [](const LogFile& self){return self.timeMs;})
        .def_property_readonly("duration_ms", [](const LogFile& self){return self.durationMs;})
        .def_property_readonly("records", [](const LogFile& self){return self.records;})
        .def_property_readonly("tracks", [](const LogFile& self){return self.tracks;})
        .def_property_readonly("file_write_position", &LogFile::fileWritePosition);

    py::class_<LogFile::Track>(logfile, "Track")
        .def_readonly("id", &LogFile::Track::id)
        .def_readonly("file_offset", &LogFile::Track::fileOffset)
        .def_readonly("start_index", &LogFile::Track::startIndex)
        .def_readonly("record_count", &LogFile::Track::recordCount)
        .def_readonly("start_time", &LogFile::Track::startTime)
        .def_readonly("duration_ms", &LogFile::Track::durationMs)
        .def_readonly("data_type", &LogFile::Track::dataType)
        .def_property_readonly("data",[](const LogFile::Track& self) { return self.data; });

    py::class_<LogFile::RecordIndex>(logfile, "RecordIndex")
        .def_readonly("track_id", &LogFile::RecordIndex::trackId)
        .def_readonly("file_offset", &LogFile::RecordIndex::fileOffset)
        .def_readonly("time_ms", &LogFile::RecordIndex::timeMs)
        .def_readonly("can_skip", &LogFile::RecordIndex::canSkip)
        .def_readonly("type", &LogFile::RecordIndex::type);

    py::class_<LogFile::RecordHeader> recordHeader(logfile, "RecordHeader");
    recordHeader.def_readonly("time_ms", &LogFile::RecordHeader::timeMs)
        .def_readonly("track_id", &LogFile::RecordHeader::trackId)
        .def_readonly("can_skip", &LogFile::RecordHeader::canSkip)
        .def_readonly("type", &LogFile::RecordHeader::recordType)
        .def_readonly("data_size", &LogFile::RecordHeader::dataSize)
        .def_readonly("data_type", &LogFile::RecordHeader::dataType);

    py::enum_<LogFile::RecordHeader::Type>(recordHeader, "RecordType")
        .value("Meta", LogFile::RecordHeader::Type::Meta)
        .value("Data", LogFile::RecordHeader::Type::Data)
        .value("Index", LogFile::RecordHeader::Type::Index)
        .value("Track", LogFile::RecordHeader::Type::Track);

    py::class_<LogFile::FileHeader>(logfile, "FileHeader")
        .def_readonly("id", &LogFile::FileHeader::id)
        .def_readonly("version", &LogFile::FileHeader::version)
        .def_readonly("time_ms", &LogFile::FileHeader::timeMs)
        .def_readonly("index_position", &LogFile::FileHeader::indexPosition);

    py::enum_<LogFile::Error>(logfile, "Error")
        .value("NoErr", LogFile::Error::None)
        .value("CannotOpen", LogFile::Error::CannotOpen)
        .value("CannotCreate", LogFile::Error::CannotCreate)
        .value("CannotWrite", LogFile::Error::CannotWrite)
        .value("Damaged", LogFile::Error::Damaged);

    
    //---------------------------------------- Log Writer ------------------------------------------
    py::class_<LogWriter, std::shared_ptr<LogWriter>>(m, "LogWriter")
        .def(py::init<>())
        .def("start_new_file", &LogWriter::startNewFile, "Start a new file", "filename"_a)
        .def("add_track", [](LogWriter& self, uint8_t dataType, const py::bytes& data)
        {
            uint_t size = PyBytes_Size(data.ptr());
            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(PyBytes_AsString(data.ptr()));
            return self.addTrack(dataType, ptr, size);
        }, "Add a track", "dataType"_a, "data"_a)
        .def("add_track_data", [](LogWriter& self, uint8_t trackId, const py::bytes& data, uint8_t dataType, bool_t canSkip=true)
        {
            uint_t size = PyBytes_Size(data.ptr());
            const uint8_t* ptr = reinterpret_cast<const uint8_t*>(PyBytes_AsString(data.ptr()));
            return self.addTrackData(trackId, ptr, size, dataType, canSkip);
        }, "Add track data", "trackId"_a, "data"_a, "dataType"_a, "canSkip"_a)
        .def("close", &LogWriter::close, "Close the file")
        .def("set_max_file_size", &LogWriter::setMaxFileSize, "Set the max file size", "maxSize"_a)
        .def_property_readonly("record_count", &LogWriter::recordCount)
        .def_property_readonly("time_ms", &LogWriter::timeMs)
        .def_property_readonly("duration_ms", &LogWriter::durationMs);

    //---------------------------------------- Log Reader ------------------------------------------
    py::class_<LogReader, LogWriter, std::shared_ptr<LogReader>> logReader(m, "LogReader");
    logReader.def(py::init<>())
        .def("open", &LogReader::open, "Open the log file", "filename"_a)
        .def("play", &LogReader::play, "Play the log file", "playSpeed"_a)
        .def("reset", &LogReader::reset, "Reset the log file")
        .def("seek", &LogReader::seek, "Seek to a record", "index"_a)
        .def("process", &LogReader::process, "Run the log file")
        .def_readonly("on_record", &LogReader::onRecord)
        .def_readonly("on_error", &LogReader::onError);

    //----------------------------------------- Signals --------------------------------------------
    py::class_<Signal<LogReader&, const LogReader::RecordData&>>(logReader, "Signal_OnRecord")        // onRecord type signal
        .def("connect", &Signal<LogReader&, const LogReader::RecordData&>::pyConnect)
        .def("disconnect", &Signal<LogReader&, const LogReader::RecordData&>::pyDisconnect);

    py::class_<Signal<LogReader&, const std::string&>>(logReader, "Signal_OnError")                 // onError type signal
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
    py::class_<Signal<LogPlayer&, const LogFile::Track&>>(logPlayer, "Signal_OnNewTrack")        // onNewTrack type signal
        .def("connect", &Signal<LogPlayer&, const LogFile::Track&>::pyConnect)
        .def("disconnect", &Signal<LogPlayer&, const LogFile::Track&>::pyDisconnect);
}
//--------------------------------------------------------------------------------------------------