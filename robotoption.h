#ifndef ROBOTOPTION_H_
#define ROBOTOPTION_H_

#include <arpa/inet.h>
#include <booloption.h>
#include <enumoption.h>
#include <floatoption.h>
#include <intoption.h>
#include <option.h>
#include <optionset.h>
#include <visstring.h>

#include <cstdint>
#include <cstring>
#include <deque>
#include <future>
#include <list>
#include <map>
#include <mutex>

namespace NaoControl {

class RobotOption {
private:
    std::map<std::string, OptionSetPtr> optionSets;
    std::list<OptionSetPtr> optionSetList;

    struct sockaddr_in* lastRcvAddr{};

    size_t length;
    std::mutex optionSetMutex;

    static void serverThread(std::promise<void> &&promise);
    static void clientThread(int sock);

    static uint16_t serverPort;

    const static uint8_t VERSION;
    const static uint32_t MAGIC_BYTE;

    enum RobotOptionType { OPTIONSET };

    /** This is a singelton... it should be never destroyed or subclassed. */
    ~RobotOption();
    RobotOption();

    void sendOptionSets(int sock);

public:
    /* This class is not copyable */
    RobotOption(const RobotOption& v) = delete;
    RobotOption(RobotOption&& v) = delete;
    RobotOption& operator=(const RobotOption&) = delete;
    RobotOption& operator=(RobotOption&&) = delete;

    /**
     * @brief setServerPort Set the port the server will later listen on.
     * @param port The port where the server should created.
     */
    static void setServerPort(uint16_t port);

    /**
     * Gets a instance of this singelton. The first time this is called all
     * communication threads are created and the robotoption system is ready to
     * run.
     */
    static RobotOption& instance();

    /**
     * Add a OptionSet. This options will be sent to NaoControl
     * and can be edited in the GUI. After you added the OptionSet it belongs to
     * RobotOption and MUST NOT be changed.
     */
    void addOptionSet(OptionSet* optionSet);

    /**
     * @brief Returns the source address of the last received option or NULL if none is known so far. The retuned value
     * should never be free()'d.
     */
    struct sockaddr_in* getLastRcvAddr();
};

}  // namespace NaoControl
#endif /* ROBOTOPTION_H_ */
