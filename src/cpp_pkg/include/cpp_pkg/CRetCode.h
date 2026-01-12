#ifndef CRETTYPE_H_1711617693
#define CRETTYPE_H_1711617693

#include <string>
#include <ostream>

struct RetCode final {
public:
    enum RetEnum : int {
        Running = 0,
        Success = 1,
        ForceStop,
        TextParsingError,
        WriteError,
        ReadError,
        TimeOut,
        MCUError,
        RefuseError,
        TryAgain,
        CrashError,
        SerialError,

        TODO,
        UnknownError = 0x3f3f3f3f,
    };

public:
    RetEnum result = UnknownError;

public:
    RetCode() = default;
    RetCode(RetEnum num) : result(num) {}
    RetCode(const RetCode&)            = default;
    RetCode& operator=(const RetCode&) = default;

public:
    const char* c_str() const noexcept {
        switch (result) {
        case Running          : return "Running";
        case Success          : return "Success";
        case ForceStop        : return "ForceStop";
        case WriteError       : return "WriteError";
        case TextParsingError : return "TextParsingError";
        case ReadError        : return "ReadError";
        case TimeOut          : return "TimeOut";
        case MCUError         : return "MCUError";
        case RefuseError      : return "RefuseError";
        case TryAgain         : return "TryAgain";
        case CrashError       : return "CrashError";
        case SerialError      : return "SerialError";
        case TODO             : return "TODO";
        case UnknownError     : return "UnknownError";
        }
        return "Enum Out of Range";
    }

    std::string to_string() const {
        return std::string(c_str());
    }

public:
    friend inline std::ostream& operator<<(std::ostream& os, const RetCode& rc) {
        return os << rc.c_str();
    }

    friend inline bool operator==(const RetCode& lhs, const RetCode& rhs) {
        return lhs.result == rhs.result;
    }

    friend inline bool operator!=(const RetCode& lhs, const RetCode& rhs) {
        return lhs.result != rhs.result;
    }

    explicit operator bool() const noexcept {
        return result == Success;
    }
};

inline std::string to_string(const RetCode& rc) {
    return rc.to_string();
}


#endif // CRETTYPE_H_1711617693
