#ifndef ENUMOPTION_H_
#define ENUMOPTION_H_

#include <option.h>

#include <cstdint>
#include <list>
#include <memory>

namespace NaoControl {

class EnumItem {
private:
    uint8_t* buffer;
    int len;

public:
    /** This class is not intended to by copied */
    EnumItem(const EnumItem& s) = delete;
    EnumItem(EnumItem&&) = delete;
    EnumItem& operator=(const EnumItem&) = delete;
    EnumItem& operator=(EnumItem&&) = delete;
    ~EnumItem() {
        free(buffer);
    }
    EnumItem(const char* name, int value);
    size_t size() const {
        return len;
    }
    uint8_t* save(uint8_t* ptr);
};

using EnumItemPtr = std::shared_ptr<EnumItem>;

class EnumOption : public Option {
private:
    int* value;
    std::function<void(int)> callback;
    std::list<EnumItemPtr> enumItems;

public:
    /** This class is not intended to by copied */
    EnumOption(EnumOption& s) = delete;
    EnumOption(EnumOption&& s) = delete;
    EnumOption& operator=(const EnumOption&) = delete;
    EnumOption& operator=(EnumOption&&) = delete;
    ~EnumOption() override = default;

    EnumOption(const char* cname, int* val, std::function<void(int)> callb = nullptr);
    uint8_t* save(uint8_t* start) override;
    void setValue(const uint8_t* buffer, int len) override;

    void addEnumItem(EnumItem* item);
    OptionType getOptionType() override {
        return O_ENUM;
    }
    void* getValuePtr() override {
        return value;
    }
};

} /* namespace NaoControl */
#endif /* ENUMOPTION_H_ */
