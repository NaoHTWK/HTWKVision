#ifndef FLIGHTRECORDERLOG_H
#define FLIGHTRECORDERLOG_H

#include <google/protobuf/message_lite.h>
#include <pthread.h>

#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <utility>

#include "producerconsumerlock.h"

namespace FlightRecorder {

enum class LogSource { FIRMWARE = 0, BIDGE };

enum class LogLevel { INFO, DEBUG, WARN, ERROR };

class FlightRecorderLog;

/**
 * @brief The Log class is facede for th FlightRecorderLog which offers all
 *         functions to log messages and binary blobs.
 */
class Log {
private:
    const std::string subsystem;
    FlightRecorderLog& recorder;

    Log(std::string subsystem, FlightRecorderLog& recorder) : subsystem(std::move(subsystem)), recorder(recorder) {}

    void logProtobuf(LogLevel loglevel, google::protobuf::MessageLite& msg, const char* typeinfo);
    void logBinaryData(LogLevel loglevel, const void* ptr, uint32_t length, const char* typeinfo);
    void logText(LogLevel loglevel, const char* message);

    friend class FlightRecorderLog;

public:
    /* Logging of protobuf data. Typeinfo can be NULL. */
    void info(google::protobuf::MessageLite& msg, const char* typeinfo = nullptr);
    void debug(google::protobuf::MessageLite& msg, const char* typeinfo = nullptr);
    void warn(google::protobuf::MessageLite& msg, const char* typeinfo = nullptr);
    void err(google::protobuf::MessageLite& msg, const char* typeinfo = nullptr);

    /* Logging of binary data. The typeinfo can be NULL. */
    void info(const void* ptr, uint32_t length, const char* typeinfo = nullptr);
    void debug(const void* ptr, uint32_t length, const char* typeinfo = nullptr);
    void warn(const void* ptr, uint32_t length, const char* typeinfo = nullptr);
    void err(const void* ptr, uint32_t length, const char* typeinfo = nullptr);

    /* Logging of text messages. Similiar to printf. */
    void infoMsg(const char* message, ...);
    void debugMsg(const char* message, ...);
    void warnMsg(const char* message, ...);
    void errMsg(const char* message, ...);
};

using LogPtr = std::shared_ptr<Log>;

/**
 * @brief The FlightRecorderLog class is a singelton logs all input to a daemon
 *        which runs usally on the nao. The flight recorder has no insight in
 *        the packages and is only the storage interface und don't interprets
 *        the data.
 */
class FlightRecorderLog {
private:
    struct BacklogEntry {
        explicit BacklogEntry(uint32_t _length) {
            length = _length;
            ptr = malloc(_length);
        }

        BacklogEntry(const BacklogEntry&) = delete;
        BacklogEntry(BacklogEntry&&) = delete;
        BacklogEntry& operator=(const BacklogEntry&) = delete;
        BacklogEntry& operator=(BacklogEntry&&) = delete;

        ~BacklogEntry() {
            free(ptr);
        }

        uint32_t length;
        void* ptr;
    };

    using BacklogEntryPtr = std::shared_ptr<BacklogEntry>;

    friend class Log;

    /** Contain all elements that we don't had send yet */
    static std::list<BacklogEntryPtr>* backlog;
    static std::list<BacklogEntryPtr>* textBacklog;
    static Thread::ProducerConsumerLock* backlogMutex;
    static Thread::ProducerConsumerLock* textBacklogMutex;

    static LogSource src;

    /** Create a instance via the instance function. */
    FlightRecorderLog();

    static LogLevel logLevel;
    static LogLevel textLogLevel;

    /**
     * @brief log We only accept binary payload which is forwarded to the daemon.
     *        The preprocessing must be done by another class. The memory is than
     *        in the ownership of this class.
     * @param entry The entry to store.
     */
    void log(const BacklogEntryPtr& entry);

    /**
     * @brief log Text messages that should be printed on the console
     * @param entry The entry to store.
     */
    void logText(const BacklogEntryPtr& entry);

    /**
     * @brief flightLogWriterThread This is the entry function for the thread which writes the data to a file.
     * @param param The param is here the instance of this class.
     * @return Nothing, needed by convention.
     */
    static void flightLogWriterThread();

    /**
     * @brief flightLogPrinterThread This is the entry function for the thread which prints log messages to the console.
     * @param param The param is here the instance of this class.
     * @return Nothing, needed by convention.
     */
    static void flightLogPrinterThread();

public:
    /** This class is not intended to be copied */
    FlightRecorderLog(FlightRecorderLog&) = delete;
    FlightRecorderLog(FlightRecorderLog&&) = delete;
    FlightRecorderLog(const FlightRecorderLog& v) = delete;
    ~FlightRecorderLog() = default;
    FlightRecorderLog& operator=(const FlightRecorderLog&) = delete;
    FlightRecorderLog& operator=(FlightRecorderLog&&) = delete;

    /**
     * @brief instance Returns a singelton instance of the recorder.
     * @param subsystem The subsystem for which thw log is.
     * @param sec Are we logging from our firmware or the bridge.
     * @return A singelton instance of the flight recorder.
     */
    static LogPtr instance(const char* subsystem);

    /**
     * @brief setTextLogLevel Set the level of the log. This is only used
     *        for text messages.
     * @param level
     */
    static void setTextLogLevel(LogLevel level);

    /**
     * @brief setLogLevel Set the level of the log. This is used
     *        for everything except text messages.
     * @param level
     */
    static void setLogLevel(LogLevel level);

    static void setLogSource(LogSource src = LogSource::FIRMWARE);
};

}  // namespace FlightRecorder

#endif  // FLIGHTRECORDERLOG_H
