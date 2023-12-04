#include "visualizer.h"

#include <cstdarg>

#include <execinfo.h>
#include <sys/time.h>

#include <stl_ext.h>

#include <visualizer.pb.h>

namespace NaoControl
{
/******************************************************************************/
VisualizerTransaction::VisualizerTransaction(const boost::optional<YPR>& headPos,
                                             const std::string& sys,
                                             VisualizerTransactionType type,
                                             VisualizerReplacement replaceOldTransaction)
    : invalidated(false)
{
    transaction = new protobuf::visualizer::VisualizerTransaction();
    transaction->set_subsystem(sys);
    transaction->set_replacement((protobuf::visualizer::VisualizerReplacement)replaceOldTransaction);
    transaction->set_transactiontype((protobuf::visualizer::VisualizerTransactionType)type);

    if(headPos) {
        transaction->mutable_head()->set_yaw(headPos.get().yaw);
        transaction->mutable_head()->set_pitch(headPos.get().pitch);
    }

    struct timeval tv;
    gettimeofday(&tv, nullptr);
    uint64_t curTime = tv.tv_sec * 1000LL + tv.tv_usec / 1000LL;
    transaction->set_time(curTime);
}

VisualizerTransaction::~VisualizerTransaction()
{
    transaction->Clear();
    delete transaction;
}

void VisualizerTransaction::addShape(const Shape2D* s) {
    if(invalidated) {
        crash();
    }
    s->addToVisualizerTransaction(transaction);
    delete s;
}

#define MAX_FMT_SIZE 1024
void VisualizerTransaction::addMessage(const char *fmt, ...) {
    if(invalidated) {
        crash();
    }

    char formattedString[MAX_FMT_SIZE];

    va_list argptr;
    va_start(argptr, fmt);
    vsnprintf(formattedString, MAX_FMT_SIZE, fmt, argptr);
    va_end(argptr);

    transaction->add_messages(formattedString);
}

void VisualizerTransaction::addParameter(ParamPtr param){
    if(invalidated) {
        crash();
    }

    transaction->add_parameter()->CopyFrom(*(param->param));
}

size_t VisualizerTransaction::size() const
{
    return transaction->ByteSizeLong();
}

uint8_t* VisualizerTransaction::save(uint8_t* start)
{
    int size = transaction->ByteSizeLong();
    transaction->SerializeToArray(start, size);
    return start + size;
}


void VisualizerTransaction::crash() {
    void* array[10];
    size_t size;
    char** strings;
    size_t i;

    printf("Visualizer: Transaction was changed after it was commit! Change was from:\n");

    size = backtrace(array, 10);
    strings = backtrace_symbols(array, size);

    for(i = 0; i < size; i++) {
        printf("%s\n", strings[i]);
    }

    free(strings);
    fflush(stdout);

    exit(1);
}


/******************************************************************************/
pthread_mutex_t Visualizer::flightLogContainerMutex;
const bool Visualizer::visualizerFlightLogEnable = true;

Visualizer::Visualizer() {
    visLog = FlightRecorder::FlightRecorderLog::instance("Visualizer");
    pthread_mutex_init(&flightLogContainerMutex, nullptr);

    if(visualizerFlightLogEnable)
        launch_named_thread("NCVis:FlightLog", false, &Visualizer::flightLogClientThread).detach();
}

Visualizer::~Visualizer() {
    pthread_mutex_destroy(&flightLogContainerMutex);
}

Visualizer& Visualizer::instance() {
    static Visualizer myInstance;
    return myInstance;
}

VisTransPtr Visualizer::startTransaction(const boost::optional<YPR>& headPos, const std::string& s,
                                         VisualizerTransactionType type,
                                         VisualizerReplacement replaceOldTrans) {
    if(s.empty()) {
        visLog->errMsg("Visualizer::startTransaction: subsystem can't be null!");
        exit(1);
    }
    if(!visualizerFlightLogEnable)
        return VisTransPtr(new DummyVisualizerTransaction());
    if (type == VisualizerTransactionType::RELATIVE_HEAD && !headPos) {
        visLog->errMsg(("Visualizer::startTransaction: headPos has to be provided for non-absolute transaction" + std::string(s) + ".").c_str());
    }

    VisTransPtr tmp(new VisualizerTransaction(headPos, s, type, replaceOldTrans));
    return tmp;
}

void Visualizer::commit(const VisTransPtr& t) {
    if(visualizerFlightLogEnable) {
        pthread_mutex_lock(&flightLogContainerMutex);
        t->invalidate();
        flightLogTransactionToSend.push_back(t);
        pthread_mutex_unlock(&flightLogContainerMutex);
    }
};

#define FNAME_LEN 255

void Visualizer::flightLogClientThread() {
    uint8_t* buffer = (uint8_t*)malloc(sizeof(*buffer)*1024);
    uint8_t* ptr = nullptr;
    int curBufSize = 1024;

    std::deque<VisTransPtr>& viz = Visualizer::instance().flightLogTransactionToSend;
    VisTransPtr trans;

    static FlightRecorder::LogPtr visFileLog = FlightRecorder::FlightRecorderLog::instance("VisualizerFileLog");

    while(true) {
        pthread_mutex_lock(&flightLogContainerMutex);
        if(viz.empty()) {
            pthread_mutex_unlock(&flightLogContainerMutex);
            usleep(10000);
            continue;
        }

        trans = viz.front();
        viz.pop_front();
        pthread_mutex_unlock(&flightLogContainerMutex);

        // No size_t here or it will break compability across 32-bit and 64-bit systems. Sorry :-/
        const int tmpSize = trans->size();
        const int transSize = tmpSize + sizeof(tmpSize);

        /* only allocate memory once */
        if(curBufSize < transSize) {
            free(buffer);
            buffer = (uint8_t*)malloc(transSize);
            curBufSize = transSize;
        }

        ptr = buffer;
        memcpy(ptr, &tmpSize, sizeof(tmpSize));
        ptr += sizeof(tmpSize);
        trans->save(ptr);

        visFileLog->debug(buffer, transSize, "Transaction");
    }

    free(buffer);
}

} /* namespace naocontrol */
