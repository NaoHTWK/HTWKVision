#ifndef OPTIONSET_H_
#define OPTIONSET_H_

#include <option.h>

#include <climits>
#include <cstddef>
#include <cstdint>
#include <list>
#include <map>
#include <string>

namespace NaoControl {

class OptionSet {
private:
    std::map<std::string, OptionPtr> options;
    std::list<OptionPtr> optionList;
    std::string name;

    uint32_t length;

public:
    /** This class is not intended to by copied */
    OptionSet(OptionSet& s) = delete;
    OptionSet(OptionSet&& s) = delete;
    OptionSet& operator=(const OptionSet&) = delete;
    OptionSet& operator=(OptionSet&&) = delete;
    ~OptionSet() = default;

    explicit OptionSet(const char* cname);
    void addOption(Option* option);
    void setOption(const std::string& name, const uint8_t* buffer, int len);

    const std::string& getName() const {
        return name;
    }

    uint8_t* save(uint8_t* start);
    size_t size() const {
        return length;
    }

    std::list<OptionPtr>::const_iterator begin() {
        return optionList.begin();
    }
    std::list<OptionPtr>::const_iterator end() {
        return optionList.end();
    }
};

using OptionSetPtr = std::shared_ptr<OptionSet>;
}  // namespace NaoControl

#endif /* OPTIONSET_H_ */
