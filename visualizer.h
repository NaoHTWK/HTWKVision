#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include <cstdint>
#include <cstring>
#include <string>

#include <pthread.h>

#include <map>
#include <list>
#include <deque>
#include <memory>

#include <boost/optional.hpp>

#include <shape.h>
#include <parameter.h>
#include <flightrecorderlog.h>
#include <imu.h>

namespace protobuf {
namespace visualizer {
    class Head;
    class VisualizerTransaction;
}
}

namespace NaoControl
{
class VAnnotation;
using VAnnPtr = std::shared_ptr<VAnnotation>;


enum VisualizerTransactionType {
    ABSOLUTE = 0, //This will be displayed at the 'Absolute Localization' tab and
                  // coordinates are for the field. Center is (0,0), Buttom left
                  // is (-(FIELD_WITH/2), -(FIELD_LENGTH/2)) our half, Top right
                  // ((FIELD_WITH/2), (FIELD_LENGTH/2)) opponent half.

    RELATIVE_HEAD,// This will be displayed at the 'Relative Localization' tab and
                  // coordinates are from the robot. Center is (0,0) is the robot,
                  // (-1,1) one meter left ( the -1) and one meter in front (the 1)
                  // All primitives are rotated by head yaw.

    RELATIVE_BODY // This will be displayed at the 'Relative Localization' tab and
                  // coordinates are from the robot. Center is (0,0) is the robot,
                  // (-1,1) one meter left ( the -1) and one meter in front (the 1)
                  // All primitives are NOT rotated by head yaw.

};

enum VisualizerReplacement {
    NO_REPLACE = 0, // Transactions of this subsystem and type in NaoControl are
                    // only deleted if _ALL_ shapes faded out. Be sure all shapes
                    // have a fadeout time or you will run out on memory

    REPLACE         // All transactions of this subsystem and given type (RELATIVE,
                    // ABSOLUTE) in NaoControl are deleted and this one is displayed.
                    // If a shape is not completely fade out, it will still be deleted
};

class VisualizerTransactionInterface {
public:
    VisualizerTransactionInterface() = default;
    VisualizerTransactionInterface(VisualizerTransactionInterface& o) = delete;
    VisualizerTransactionInterface(VisualizerTransactionInterface&& o) = delete;
    VisualizerTransactionInterface& operator=(const VisualizerTransactionInterface&) = delete;
    VisualizerTransactionInterface& operator=(VisualizerTransactionInterface&&) = delete;

    virtual ~VisualizerTransactionInterface() = default;

    /**
     * Add a shape to this transaction. The owner of the pointer is the
     * transaction. The caller guarantees that the shape isn't change anymore.
     */
    virtual void addShape(const Shape2D* s) = 0;

    /**
     * Add a text message to this transaction which will be displayed in
     * NaoControl. The string will be copied and not modified. This works like
     * printf.
     *
     * @param fmt The format string
     * @param ... Arguments for the format string.
     */
    virtual void addMessage(const char* fmt, ...) = 0;

    /**
     * Add a parameter to this transaction. This parameter will be displayed.
     * The last value of this parameter will be displayed in NaoControl.
     */
    virtual void addParameter(ParamPtr param) = 0;

    /**
     * Size of the transaction in bytes.
     */
    virtual size_t size() const = 0;

    /**
     * Save the transaction to a byte array. The array has to be at least size()
     * bytes. Returned is the first free byte after the write operation.
     */
    virtual uint8_t* save(uint8_t* start) = 0;

    /**
     * @brief invalidate Invalidates the transaction so we crash.
     */
    virtual void invalidate() = 0;
};

class Visualizer;

class VisualizerTransaction : public VisualizerTransactionInterface {
private:
    protobuf::visualizer::VisualizerTransaction* transaction;

    VisualizerTransaction(const boost::optional<YPR>& headPos, const std::string &sys,
                          VisualizerTransactionType type,
                          VisualizerReplacement replaceOldTransaction);

    bool invalidated;
    void crash();

public:
    VisualizerTransaction(VisualizerTransaction& o) = delete;
    VisualizerTransaction(VisualizerTransaction&& o) = delete;
    VisualizerTransaction& operator=(const VisualizerTransaction&) = delete;
    VisualizerTransaction& operator=(VisualizerTransaction&&) = delete;

    ~VisualizerTransaction() override;

    void addShape(const Shape2D* s) override;
    void addMessage(const char* fmt, ...) override;
    void addParameter(ParamPtr param) override;

    size_t size() const override;
    uint8_t* save(uint8_t* start) override;

    void invalidate() override { invalidated = true; }

    friend class Visualizer;
};

using VisTransPtr = std::shared_ptr<VisualizerTransactionInterface>;


class DummyVisualizerTransaction : public VisualizerTransactionInterface {
private:
    DummyVisualizerTransaction() = default;

public:
    DummyVisualizerTransaction(DummyVisualizerTransaction& o) = delete;
    DummyVisualizerTransaction(DummyVisualizerTransaction&& o) = delete;
    DummyVisualizerTransaction& operator=(const DummyVisualizerTransaction&) = delete;
    DummyVisualizerTransaction& operator=(DummyVisualizerTransaction&&) = delete;

    ~DummyVisualizerTransaction() override = default;

    void addShape(const Shape2D* s) override { delete s; }
    void addMessage(const char* /* fmt */, ...) override {}
    void addParameter(const ParamPtr /* param */) override {}
    virtual void replaceOldTransaction() {}

    size_t size() const override { return 0; }
    uint8_t* save(uint8_t* start) override { return start; }

    void invalidate() override {}

    friend class Visualizer;
};

class Visualizer {
private:
    std::deque<VisTransPtr> networkTransactionToSend;
    std::deque<VisTransPtr> flightLogTransactionToSend;

    static pthread_mutex_t flightLogContainerMutex;

    static const bool visualizerFlightLogEnable;

    static void flightLogClientThread();

    FlightRecorder::LogPtr visLog;

    /** This is a singelton... it should be never destroyed or subclassed. */
    ~Visualizer();

public:
    /* This class is not copyable */
    Visualizer();
    Visualizer(const Visualizer& v) = delete;
    Visualizer(const Visualizer&& v) = delete;
    Visualizer& operator=(const Visualizer&) = delete;
    Visualizer& operator=(Visualizer&&) = delete;

    /**
     * Gets a instance of this singelton. The first time this is called all
     * communication threads are created and the visualizer system is ready to
     * run.
     */
    static Visualizer& instance();

    /**
     * Start a drawing transaction for the given subsystem, for a given type
     * (ABSOLUTE, RELATIVE) and a replacement strategy for old transactions.
     * When it's a relative transaction the current head position is used.
     *
     * @param subsystem
     *   Who is senden this transaction e.g. Localization, Projection this is
     *   later used for filtering and replacement. Use it wisely, don't hijack
     *   other subsystems
     *
     * @param
     *   type Is this for the ABSOLUTE or RELATIVE localization tab in NaoControl
     *
     * @param replaceOldTransaction
     *   Replace or not replace all old transactions of the subsystem and type.
     *   If set to NO_REPLACE be sure all shapes have a fadeout time or they will
     *   be never deleted and you run out of memory.
     */
    VisTransPtr startTransaction(const boost::optional<YPR>& headPos,
                                 const std::string &s,
                                 VisualizerTransactionType type,
                                 VisualizerReplacement replaceOldTrans = REPLACE);

    /**
     * Commit a transaction and send it to NaoControl. You agree that you will
     * never touch this transaction again.
     */
    void commit(const VisTransPtr& t);
};

} /* namespace naocontrol */
#endif /* VISUALIZER_H_ */
