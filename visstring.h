#ifndef VISSTRING_H_
#define VISSTRING_H_

#include <cstdlib>
#include <cstdint>

#include <boost/shared_ptr.hpp>

namespace NaoControl
{

class VisString {
private:
    uint8_t* buffer;
    int len;

public:
    /** This class is not intended to by copied */
    VisString(VisString& s) = delete;
    VisString(VisString&& s) = delete;
    VisString& operator=(const VisString&) = delete;
    VisString& operator=(VisString&&) = delete;
    explicit VisString(const char* str);

    ~VisString();
    uint8_t* save(uint8_t* start);
    size_t size() const { return len; }

    /*
     * Read a VisString from a socket, on error null is returned.
     */
    static char* read(int sock);
};

using VisStringPtr = boost::shared_ptr<VisString>;
}
#endif /* VISSTRING_H_ */
