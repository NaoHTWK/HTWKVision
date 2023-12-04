#include "flightrecorderlog.h"

#include <log.pb.h>
#include <sockethelper.h>
#include <stl_ext.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstdarg>

using namespace FlightRecorder;
namespace plog = protobuf::log;

static const char* logLevelString[] = {"INFO ", "DEBUG", "WARN ", "ERROR"};

LogLevel FlightRecorderLog::logLevel = LogLevel::INFO;
LogLevel FlightRecorderLog::textLogLevel = LogLevel::INFO;

std::list<FlightRecorderLog::BacklogEntryPtr>* FlightRecorderLog::backlog;
std::list<FlightRecorderLog::BacklogEntryPtr>* FlightRecorderLog::textBacklog;
Thread::ProducerConsumerLock* FlightRecorderLog::backlogMutex;
Thread::ProducerConsumerLock* FlightRecorderLog::textBacklogMutex;

LogSource FlightRecorderLog::src = LogSource::FIRMWARE;

void FlightRecorderLog::setTextLogLevel(LogLevel level) {
    FlightRecorderLog::textLogLevel = level;
}

void FlightRecorderLog::setLogLevel(LogLevel level) {
    FlightRecorderLog::logLevel = level;
}

void FlightRecorderLog::setLogSource(LogSource log_src) {
    src = log_src;
}

FlightRecorderLog::FlightRecorderLog() {
#ifdef FLIGHTLOGCLIENT_IS_DUMMY
    return;
#endif

    static std::list<BacklogEntryPtr> _backlog;
    static std::list<BacklogEntryPtr> _textBacklog;
    static Thread::ProducerConsumerLock _backlogMutex;
    static Thread::ProducerConsumerLock _textBacklogMutex;

    backlog = &_backlog;
    textBacklog = &_textBacklog;
    backlogMutex = &_backlogMutex;
    textBacklogMutex = &_textBacklogMutex;

    backlogMutex->init([]() -> bool { return FlightRecorderLog::backlog->empty(); });
    textBacklogMutex->init([]() -> bool { return FlightRecorderLog::textBacklog->empty(); });

    launch_named_thread("NFR:NetWriter", false, &flightLogWriterThread).detach();
    launch_named_thread("NFR:ConPrinter", false, &flightLogPrinterThread).detach();
}

void FlightRecorderLog::log(const BacklogEntryPtr& entry) {
#ifdef FLIGHTLOGCLIENT_IS_DUMMY
    return;
#endif

    backlogMutex->producerLock();
    backlog->push_back(entry);
    backlogMutex->producerUnlock();
}

void FlightRecorderLog::logText(const BacklogEntryPtr& entry) {
#ifdef FLIGHTLOGCLIENT_IS_DUMMY
    return;
#endif

    textBacklogMutex->producerLock();
    textBacklog->push_back(entry);
    textBacklogMutex->producerUnlock();
}

void FlightRecorderLog::flightLogPrinterThread() {
    std::list<BacklogEntryPtr> entries;

    while (true) {
        textBacklogMutex->consumerLock();
        while (!textBacklog->empty()) {
            entries.push_back(textBacklog->front());
            textBacklog->pop_front();
        }
        textBacklogMutex->consumerUnlock();

        tm tms;
        time_t tmpTime = time(nullptr);
        memset(&tms, 0, sizeof(tms));
        localtime_r(&tmpTime, &tms);

        for (auto& entry : entries) {
            plog::LogEntry logEntry;
            logEntry.ParseFromArray(entry->ptr, entry->length);

            if (logEntry.loglevel() == plog::ERROR) {
                fprintf(stderr, "\e[1;91m%02d:%02d:%02d [%s][%s] %s\e[0m\n", tms.tm_hour, tms.tm_min, tms.tm_sec,
                        logLevelString[logEntry.loglevel()], logEntry.subsystem().c_str(), logEntry.logentry().c_str());
                fflush(stderr);
            } else {
                printf("%02d:%02d:%02d [%s][%s] %s\n", tms.tm_hour, tms.tm_min, tms.tm_sec,
                       logLevelString[logEntry.loglevel()], logEntry.subsystem().c_str(), logEntry.logentry().c_str());
            }
        }

        entries.clear();
    }
}

void FlightRecorderLog::flightLogWriterThread() {
    while (true) {
        int sock;
        struct sockaddr_in saddr;

        uint16_t port = 40588;
        if (src == LogSource::BIDGE)
            port = 40589;

        if (!openTcpSendSocket(sock, saddr, "127.0.0.1", port)) {
            /* If we don't have a connection remove everything to have not a memory leak */
            backlogMutex->consumerLock();
            backlog->clear();
            backlogMutex->consumerUnlock();

            sleep(1);
            continue;
        }

        std::list<BacklogEntryPtr> entries;

        while (true) {
            backlogMutex->consumerLock();
            /* Copy everything may have been logged in between to our private list. */
            backlog->swap(entries);
            backlogMutex->consumerUnlock();

            bool isFine = true;

            /* Write all cached entries */
            while (!entries.empty()) {
                auto entry = entries.front();
                entries.pop_front();

                if (!socketWriteBytes(sock, sizeof(entry->length), &(entry->length))) {
                    close(sock);
                    isFine = false;
                    break;
                }

                if (!socketWriteBytes(sock, entry->length, entry->ptr)) {
                    close(sock);
                    isFine = false;
                    break;
                }
            }

            // If we have a socket connection we don't want to keep everything.
            entries.clear();
            if (!isFine)
                break;
        }
    }
}

LogPtr FlightRecorderLog::instance(const char* subsystem) {
    static auto* instance = new FlightRecorderLog();
    return LogPtr(new Log(subsystem, *instance));
}

void Log::info(google::protobuf::MessageLite& msg, const char* typeinfo) {
    logProtobuf(LogLevel::INFO, msg, typeinfo);
}

void Log::debug(google::protobuf::MessageLite& msg, const char* typeinfo) {
    logProtobuf(LogLevel::DEBUG, msg, typeinfo);
}

void Log::warn(google::protobuf::MessageLite& msg, const char* typeinfo) {
    logProtobuf(LogLevel::WARN, msg, typeinfo);
}

void Log::err(google::protobuf::MessageLite& msg, const char* typeinfo) {
    logProtobuf(LogLevel::ERROR, msg, typeinfo);
}

void Log::info(const void* ptr, uint32_t length, const char* typeinfo) {
    logBinaryData(LogLevel::INFO, ptr, length, typeinfo);
}

void Log::debug(const void* ptr, uint32_t length, const char* typeinfo) {
    logBinaryData(LogLevel::DEBUG, ptr, length, typeinfo);
}

void Log::warn(const void* ptr, uint32_t length, const char* typeinfo) {
    logBinaryData(LogLevel::WARN, ptr, length, typeinfo);
}

void Log::err(const void* ptr, uint32_t length, const char* typeinfo) {
    logBinaryData(LogLevel::ERROR, ptr, length, typeinfo);
}

void Log::infoMsg(const char* message, ...) {
    va_list args;
    char buffer[1024];

    va_start(args, message);
    vsnprintf(buffer, sizeof(buffer), message, args);
    va_end(args);
    logText(LogLevel::INFO, buffer);
}

void Log::debugMsg(const char* message, ...) {
    va_list args;
    char buffer[1024];

    va_start(args, message);
    vsnprintf(buffer, sizeof(buffer), message, args);
    va_end(args);
    logText(LogLevel::DEBUG, buffer);
}

void Log::warnMsg(const char* message, ...) {
    va_list args;
    char buffer[1024];

    va_start(args, message);
    vsnprintf(buffer, sizeof(buffer), message, args);
    va_end(args);
    logText(LogLevel::WARN, buffer);
}

void Log::errMsg(const char* message, ...) {
    va_list args;
    char buffer[1024];

    va_start(args, message);
    vsnprintf(buffer, sizeof(buffer), message, args);
    va_end(args);
    logText(LogLevel::ERROR, buffer);
}

void Log::logProtobuf(LogLevel loglevel, google::protobuf::MessageLite& msg, const char* typeinfo) {
    if (loglevel < FlightRecorderLog::logLevel)
        return;

    size_t desiredSize = msg.ByteSizeLong();
    char* buffer = (char*)malloc(desiredSize);

    if (buffer == nullptr) {
        fprintf(stderr, "Error malloc buffer for log %s %d\n", __FILE__, __LINE__);
        return;
    }

    if (!msg.SerializeToArray(buffer, desiredSize)) {
        fprintf(stderr, "Error serializing protobuf in Log %s %d\n", __FILE__, __LINE__);
        free(buffer);
        return;
    }

    plog::LogEntry entry;
    entry.set_loglevel((plog::LogLevel)loglevel);
    entry.set_logtype(plog::PROTOBUF);
    entry.set_subsystem(subsystem);
    entry.set_timestamp(time_us());
    entry.set_logentry(buffer, desiredSize);
    entry.set_size(desiredSize);
    entry.set_src(recorder.src == LogSource::FIRMWARE ? protobuf::log::LogSource::FIRMWARE
                                                      : protobuf::log::LogSource::BRIDGE);

    if (typeinfo != nullptr) {
        entry.set_typeinfo(typeinfo);
    }

    size_t logEntrySize = entry.ByteSizeLong();
    FlightRecorderLog::BacklogEntryPtr logEntry = std::make_shared<FlightRecorderLog::BacklogEntry>(logEntrySize);
    if (!entry.SerializeToArray(logEntry->ptr, logEntrySize)) {
        fprintf(stderr, "ERROR: Serializing log entry in %s %d\n", __FILE__, __LINE__);
        free(buffer);
        return;
    }
    recorder.log(logEntry);
    entry.Clear();
    free(buffer);
}

void Log::logBinaryData(LogLevel loglevel, const void* ptr, uint32_t length, const char* typeinfo) {
    if (loglevel < FlightRecorderLog::logLevel)
        return;

    plog::LogEntry entry;
    entry.set_loglevel((plog::LogLevel)loglevel);
    entry.set_logtype(plog::BINARY);
    entry.set_subsystem(subsystem);
    entry.set_timestamp(time_us());
    entry.set_logentry(ptr, length);
    entry.set_size(length);
    entry.set_src(recorder.src == LogSource::FIRMWARE ? protobuf::log::LogSource::FIRMWARE
                                                      : protobuf::log::LogSource::BRIDGE);

    if (typeinfo != nullptr) {
        entry.set_typeinfo(typeinfo);
    }

    size_t logEntrySize = entry.ByteSizeLong();
    FlightRecorderLog::BacklogEntryPtr logEntry(new FlightRecorderLog::BacklogEntry(logEntrySize));
    if (!entry.SerializeToArray(logEntry->ptr, logEntrySize)) {
        fprintf(stderr, "ERROR: Serializing log entry in %s %d\n", __FILE__, __LINE__);
        return;
    }

    entry.Clear();
    recorder.log(logEntry);
}

void Log::logText(LogLevel loglevel, const char* message) {
    if (loglevel < FlightRecorderLog::textLogLevel)
        return;

    plog::LogEntry entry;
    entry.set_loglevel((plog::LogLevel)loglevel);
    entry.set_logtype(plog::TEXT);
    entry.set_subsystem(subsystem);
    entry.set_timestamp(time_us());
    entry.set_logentry(message, strlen(message));
    entry.set_size(strlen(message));
    entry.set_src(recorder.src == LogSource::FIRMWARE ? protobuf::log::LogSource::FIRMWARE
                                                      : protobuf::log::LogSource::BRIDGE);

    size_t logEntrySize = entry.ByteSizeLong();
    FlightRecorderLog::BacklogEntryPtr logEntry(new FlightRecorderLog::BacklogEntry(logEntrySize));
    if (!entry.SerializeToArray(logEntry->ptr, logEntrySize)) {
        fprintf(stderr, "ERROR: Serializing log entry in %s %d\n", __FILE__, __LINE__);
        return;
    }
    recorder.log(logEntry);
    recorder.logText(logEntry);
    entry.Clear();
}
