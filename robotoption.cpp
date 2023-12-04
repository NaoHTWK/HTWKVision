#include <robotoption.h>
#include <sockethelper.h>
#include <stl_ext.h>

#include <cstdarg>

namespace NaoControl {
const uint32_t RobotOption::MAGIC_BYTE = 0xabfb3c2a;
const uint8_t RobotOption::VERSION = 1;

uint16_t RobotOption::serverPort = 0;

RobotOption::RobotOption() {
    length = sizeof(MAGIC_BYTE) + sizeof(VERSION) + sizeof(uint8_t) /* type */ + sizeof(uint32_t) /* option sets */;

    std::promise<void> promise;
    std::future<void> future = promise.get_future();

    launch_named_thread("NCOptions::Srv", false, [&promise]() { RobotOption::serverThread(std::move(promise)); }).detach();
    future.get();
}

RobotOption::~RobotOption() {}

RobotOption& RobotOption::instance() {
    static RobotOption myInstance;
    return myInstance;
}

void RobotOption::addOptionSet(OptionSet* optionSet) {
    std::lock_guard<std::mutex> lck(optionSetMutex);
    OptionSetPtr oSet(optionSet);
    optionSets[optionSet->getName()] = oSet;
    optionSetList.push_back(oSet);
    length += optionSet->size();
}

void RobotOption::sendOptionSets(int sock) {
    optionSetMutex.lock();
    int sendLeft = length;
    std::list<OptionSetPtr> copyList(optionSetList);
    optionSetMutex.unlock();

    uint32_t optionSetCount = copyList.size();
    uint8_t* buffer = (uint8_t*)malloc(sendLeft);

    uint8_t* ptr = buffer;
    memcpy(ptr, &MAGIC_BYTE, sizeof(MAGIC_BYTE));
    ptr += sizeof(MAGIC_BYTE);
    *ptr = VERSION;
    ptr++;
    *ptr = (uint8_t)OPTIONSET;
    ptr++;
    memcpy(ptr, &optionSetCount, sizeof(optionSetCount));
    ptr += sizeof(optionSetCount);

    for (auto& it : copyList) {
        ptr = it->save(ptr);
    }

    socketWriteBytes(sock, sendLeft, buffer);
    free(buffer);
}

void RobotOption::clientThread(int sock) {
    RobotOption& rOpts = RobotOption::instance();

    rOpts.sendOptionSets(sock);

    while (true) {
        char* optionSet = VisString::read(sock);
        if (optionSet == nullptr) {
            printf("RobotOption::clientThread error reading OptionSet (is normal behaviour on NaoControl exit) %s:%d\n",
                   __FILE__, __LINE__);
            break;
        }

        char* option = VisString::read(sock);
        if (option == nullptr) {
            printf("RobotOption::clientThread error reading Option %s:%d\n", __FILE__, __LINE__);
            free(optionSet);
            break;
        }

        int32_t len;
        if (!socketReadBytes(sock, sizeof(len), &len)) {
            free(optionSet);
            free(option);
            break;
        }

        uint8_t* buffer = (uint8_t*)malloc(len);
        if (!socketReadBytes(sock, len, buffer)) {
            free(optionSet);
            free(option);
            break;
        }

        std::string sOptionSet(optionSet);
        std::string sOption(option);

        {
            std::lock_guard<std::mutex> lck(rOpts.optionSetMutex);
            std::map<std::string, OptionSetPtr>::iterator setIt(rOpts.optionSets.find(sOptionSet));
            if (setIt != rOpts.optionSets.end()) {
                setIt->second->setOption(sOption, buffer, len);
            }
        }

        /* search and set the option */

        free(optionSet);
        free(option);
        free(buffer);
    }

    close(sock);
}

void RobotOption::setServerPort(uint16_t port) {
    serverPort = port;
}

void RobotOption::serverThread(std::promise<void>&& promise) {
    int rc, sock;
    struct sockaddr_in server;

    if (serverPort == 0) {
        printf("Option Framework was not initialized correctly. Please set a port! %s %d\n", __FILE__, __LINE__);
        exit(1);
    }

    sock = socket(PF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("RobotOption: socket");
        exit(1);
    }

    const int ON = true;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &ON, sizeof(ON));
    server.sin_family = AF_INET;
    server.sin_port = htons(serverPort);
    server.sin_addr.s_addr = INADDR_ANY;

    rc = bind(sock, (struct sockaddr*)&server, sizeof(server));
    if (rc < 0) {
        perror("RobotOption: bind");
        close(sock);
        exit(1);
    }

    const int BACKLOG_LENGTH = 5;
    rc = listen(sock, BACKLOG_LENGTH);
    if (rc < 0) {
        perror("RobotOption: listen");
        close(sock);
        exit(1);
    }

    socklen_t len;
    struct sockaddr_in client;

    len = (socklen_t)sizeof(client);

    promise.set_value();
    while (true) {
        int clientSock = accept(sock, (struct sockaddr*)&client, &len);
        if (clientSock <= 0) {
            perror("RobotOption: accept failed");
            close(sock);
            pthread_exit(nullptr);
        }

        RobotOption& self = RobotOption::instance();
        if (self.lastRcvAddr == nullptr) {
            self.lastRcvAddr = (struct sockaddr_in*)malloc(sizeof(struct sockaddr_in));
            if (self.lastRcvAddr == nullptr)
                exit(EXIT_FAILURE);
        }
        memcpy(self.lastRcvAddr, &client, sizeof(client));

        launch_named_thread("NCOption:RemoteClient", false, &RobotOption::clientThread, clientSock).detach();
    }
}
struct sockaddr_in* RobotOption::getLastRcvAddr() {
    return lastRcvAddr;
}

}  // namespace NaoControl
