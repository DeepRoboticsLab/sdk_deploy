#include "common_types.h"
using namespace types;
#include "user_command_interface.h"
using namespace interface;
#include "gamepad_interface.hpp"

#include <cctype>
#include <cstdlib>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace {

struct JVal {
    enum Type { Null, Bool, Int, Double, String, Array, Object } type = Null;
    int64_t iv = 0;
    double dv = 0.0;
    std::string str;
    std::vector<JVal> arr;
    std::map<std::string, JVal> obj;

    bool is_object() const { return type == Object; }
    bool is_array() const { return type == Array; }
    bool is_string() const { return type == String; }
    bool is_number_integer() const { return type == Int; }
    bool is_number() const { return type == Int || type == Double; }
    bool contains(const std::string& k) const { return type == Object && obj.count(k) != 0; }
    const JVal& operator[](const std::string& k) const {
        static const JVal empty;
        auto it = obj.find(k);
        return it == obj.end() ? empty : it->second;
    }
    double as_double() const { return type == Int ? static_cast<double>(iv) : dv; }
};

class MiniJsonParser {
    const char* p_;
    const char* end_;
    void skip_ws() {
        while (p_ < end_ && std::isspace(static_cast<unsigned char>(*p_))) ++p_;
    }
    char peek() {
        skip_ws();
        if (p_ >= end_) throw std::runtime_error("Unexpected EOF");
        return *p_;
    }
    std::string parse_string() {
        if (*p_ != '"') throw std::runtime_error("Expected '\"'");
        ++p_;
        std::string out;
        while (p_ < end_ && *p_ != '"') {
            if (*p_ == '\\') {
                ++p_;
                if (p_ >= end_) throw std::runtime_error("Bad escape");
                switch (*p_) {
                    case '"': case '\\': case '/': out.push_back(*p_); break;
                    case 'b': out.push_back('\b'); break;
                    case 'f': out.push_back('\f'); break;
                    case 'n': out.push_back('\n'); break;
                    case 'r': out.push_back('\r'); break;
                    case 't': out.push_back('\t'); break;
                    default: out.push_back(*p_); break;
                }
            } else {
                out.push_back(*p_);
            }
            ++p_;
        }
        if (p_ >= end_ || *p_ != '"') throw std::runtime_error("Unterminated string");
        ++p_;
        return out;
    }
    JVal parse_number() {
        const char* start = p_;
        bool is_float = false;
        if (*p_ == '-') ++p_;
        while (p_ < end_ && std::isdigit(static_cast<unsigned char>(*p_))) ++p_;
        if (p_ < end_ && *p_ == '.') {
            is_float = true;
            ++p_;
            while (p_ < end_ && std::isdigit(static_cast<unsigned char>(*p_))) ++p_;
        }
        if (p_ < end_ && (*p_ == 'e' || *p_ == 'E')) {
            is_float = true;
            ++p_;
            if (p_ < end_ && (*p_ == '+' || *p_ == '-')) ++p_;
            while (p_ < end_ && std::isdigit(static_cast<unsigned char>(*p_))) ++p_;
        }
        if (p_ == start) throw std::runtime_error("Invalid number");
        std::string num(start, p_);
        JVal v;
        if (is_float) {
            v.type = JVal::Double;
            v.dv = std::strtod(num.c_str(), nullptr);
        } else {
            v.type = JVal::Int;
            v.iv = std::strtoll(num.c_str(), nullptr, 10);
        }
        return v;
    }

public:
    MiniJsonParser(const char* buf, int len) : p_(buf), end_(buf + len) {}
    JVal parse() {
        char c = peek();
        if (c == '{') {
            ++p_;
            JVal v;
            v.type = JVal::Object;
            if (peek() == '}') { ++p_; return v; }
            while (true) {
                if (peek() != '"') throw std::runtime_error("Expected key");
                std::string k = parse_string();
                if (peek() != ':') throw std::runtime_error("Expected ':'");
                ++p_;
                v.obj[k] = parse();
                char nxt = peek();
                if (nxt == '}') { ++p_; break; }
                if (nxt != ',') throw std::runtime_error("Expected ',' or '}'");
                ++p_;
            }
            return v;
        }
        if (c == '[') {
            ++p_;
            JVal v;
            v.type = JVal::Array;
            if (peek() == ']') { ++p_; return v; }
            while (true) {
                v.arr.push_back(parse());
                char nxt = peek();
                if (nxt == ']') { ++p_; break; }
                if (nxt != ',') throw std::runtime_error("Expected ',' or ']'");
                ++p_;
            }
            return v;
        }
        if (c == '"') {
            JVal v;
            v.type = JVal::String;
            v.str = parse_string();
            return v;
        }
        if (c == 't' && end_ - p_ >= 4 && std::strncmp(p_, "true", 4) == 0) {
            p_ += 4;
            JVal v; v.type = JVal::Bool; v.iv = 1;
            return v;
        }
        if (c == 'f' && end_ - p_ >= 5 && std::strncmp(p_, "false", 5) == 0) {
            p_ += 5;
            JVal v; v.type = JVal::Bool; v.iv = 0;
            return v;
        }
        if (c == 'n' && end_ - p_ >= 4 && std::strncmp(p_, "null", 4) == 0) {
            p_ += 4;
            return JVal{};
        }
        return parse_number();
    }
};

std::string getCurrentDateTime() {
    auto now = std::chrono::system_clock::now();
    std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
    std::tm* localTime = std::localtime(&currentTime);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::stringstream ss;
    ss << std::put_time(localTime, "%Y-%m-%d %H:%M:%S") << "." << std::setw(3) << std::setfill('0') << ms.count();
    return ss.str();
}

std::string buildResponse(const CmdInfo& cmd_info, const int& errorCode, std::string errorMsg = "") {
    std::ostringstream os;
    os << "{\"PatrolDevice\":{\"Command\":" << cmd_info.command
       << ",\"Items\":{\"ErrorCode\":" << errorCode;
    if (!errorMsg.empty()) {
        os << ",\"ErrorMessage\":\"" << errorMsg << "\"";
    }
    os << "},\"Time\":\"" << getCurrentDateTime() << "\",\"Type\":" << cmd_info.type << "}}";
    return os.str();
}

bool ParseJson(const char* buf, int len, JVal& document, int& type, int& command, JVal& items) {
    if (!buf || len == 0) return false;
    try {
        document = MiniJsonParser(buf, len).parse();
        if (!document.is_object() || !document.contains("PatrolDevice"))
            throw std::runtime_error("No PatrolDevice");
        const JVal& root = document["PatrolDevice"];
        if (!root.is_object()) throw std::runtime_error("PatrolDevice not object");
        if (!root.contains("Items")) throw std::runtime_error("No Items");
        items = root["Items"];
        if (!root.contains("Type") || !root.contains("Command")) throw std::runtime_error("No Type/Command");
        if (!root["Type"].is_number_integer() || !root["Command"].is_number_integer())
            throw std::runtime_error("Type/Command not Int");
        type = static_cast<int>(root["Type"].iv);
        command = static_cast<int>(root["Command"].iv);
    } catch (const std::exception& e) {
        std::cerr << "JSON Parse/Logic Error: " << e.what() << std::endl;
        return false;
    }
    return true;
}

}  // namespace

struct GamepadInterface::JsonHandlers {
    using HANDLE_FUNC = std::function<void(const JVal& item, std::string& res, const CmdInfo& cmd_info)>;
    std::unordered_map<int, std::unordered_map<int, HANDLE_FUNC>> handle_map;
};

GamepadInterface::GamepadInterface(RobotName robot_name)
    : UserCommandInterface(robot_name), handlers_(std::make_unique<JsonHandlers>()) {
    mHeartbeat_ = std::make_unique<Heartbeat>();

    // 实际手柄设备使用 M系列 ProductID(0x0010) 作为 Type 高位
    constexpr uint32_t kCA9BasicType = (CA9TypeHigh << 16) | BasicTypeLow;   // 1048676
    constexpr uint32_t kCA9MotionType = (CA9TypeHigh << 16) | MotionTypeLow; // 1048577
    constexpr uint32_t kCA9SDKType   = (CA9TypeHigh << 16) | SDKTypeLow;     // 1048581

    // === 心跳 === type=1048676, command=5
    handlers_->handle_map[kCA9BasicType][kCmdHeartbeat] =
        [this](const JVal&, std::string& res, const CmdInfo& cmd_info) {
            NetInfo netInfo;
            netInfo.ip = cmd_info.ip;
            netInfo.port = cmd_info.port;
            netInfo.fromType = cmd_info.fromType;
            mHeartbeat_->wakeUp(netInfo);
            // std::cout << "[CMD] Heartbeat from " << cmd_info.ip << ":" << cmd_info.port << std::endl;
            res = buildResponse(cmd_info, 0);
        };

    // === 摇杆(轴指令) === type=1048577(运动模块), command=1048578
    handlers_->handle_map[kCA9MotionType][kCmdSteer] =
        [this](const JVal& items, std::string& res, const CmdInfo& cmd_info) {
            if (!items.contains("X") || !items.contains("Y")) {
                res = buildResponse(cmd_info, 2, "Missing fields");
                return;
            }
            if (!items["X"].is_number() || !items["Y"].is_number()
                || (items.contains("Yaw") && !items["Yaw"].is_number())) {
                res = buildResponse(cmd_info, 3, "Data type error");
                return;
            }
            usr_cmd_->forward_vel_scale = static_cast<float>(items["X"].as_double());
            usr_cmd_->side_vel_scale = static_cast<float>(items["Y"].as_double());
            if (items.contains("Yaw") && items["Yaw"].is_number())
                usr_cmd_->turnning_vel_scale = static_cast<float>(items["Yaw"].as_double());

            std::cout << "[CMD] Steer from " << cmd_info.ip
                      << " X=" << usr_cmd_->forward_vel_scale
                      << " Y=" << usr_cmd_->side_vel_scale
                      << " Yaw=" << usr_cmd_->turnning_vel_scale << std::endl;

            static std::string LastPerIp = "";
            static int64_t LastPerTime = 0;
            auto now = std::chrono::steady_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

            if (LastPerIp == "" || LastPerIp == cmd_info.ip) {
                LastPerIp = cmd_info.ip;
                LastPerTime = timestamp;
            } else if (timestamp - LastPerTime > 2) {
                LastPerIp = cmd_info.ip;
                LastPerTime = timestamp;
            } else {
                std::cout << "IP Conflict" << std::endl;
                res = buildResponse(cmd_info, 4, "IP Conflict");
                return;
            }
        };

    // === 按键 === type=1048581(SDK模块), command=1048578
    handlers_->handle_map[kCA9SDKType][kCmdSteer] =
        [this](const JVal& items, std::string& res, const CmdInfo& cmd_info) {
            static std::unordered_map<std::string, KeyCode> keyMap = {{"G12_KEY_C", KeyCode::L1},
                                                                      {"G12_KEY_A", KeyCode::L2},
                                                                      {"G12_KEY_B", KeyCode::R1},
                                                                      {"G12_KEY_D", KeyCode::R2}};
            if (items.contains("KeyList") && items["KeyList"].is_array()) {
                std::cout << "[CMD] Key from " << cmd_info.ip << " keys=[";
                for (size_t i = 0; i < items["KeyList"].arr.size(); i++) {
                    if (i > 0) std::cout << ",";
                    std::cout << items["KeyList"].arr[i].str;
                }
                std::cout << "]" << std::endl;

                for (const auto& key : items["KeyList"].arr) {
                    if (key.is_string()) {
                        const std::string& keyStr = key.str;
                        if (keyMap.count(keyStr)) {
                            KeyCode code = keyMap[keyStr];
                            switch (code) {
                                case KeyCode::L1:
                                    if (msfb_->GetCurrentState() == RobotMotionState::WaitingForStand
                                        || msfb_->GetCurrentState() == RobotMotionState::LieDown) {
                                        usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                                        std::cout << "[MODE] Standing Up\n";
                                    }
                                    break;
                                case KeyCode::L2:
                                    if (msfb_->GetCurrentState() == RobotMotionState::StandingUp) {
                                        usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
                                        std::cout << "[MODE] RL Control\n";
                                    }
                                    break;
                                case KeyCode::R1:
                                    if (msfb_->GetCurrentState() == RobotMotionState::StandingUp
                                        || msfb_->GetCurrentState() == RobotMotionState::RLControlMode) {
                                        usr_cmd_->target_mode = uint8_t(RobotMotionState::LieDown);
                                        std::cout << "[MODE] Lie Down\n";
                                    }
                                    break;
                                case KeyCode::R2:
                                    usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
                                    std::cout << "[MODE] Joint Damping\n";
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
                }
            }
            res = buildResponse(cmd_info, 0);
        };
}

GamepadInterface::~GamepadInterface() = default;

bool GamepadInterface::ParseData(const DataInfo& dataInfo, std::string& response) {
    int type = 0, command = 0;
    mHeartbeat_->isConntected_ = true;
    mHeartbeat_->lastHeartbeatTime_ = std::chrono::steady_clock::now();

    JVal document;
    JVal items;
    if (!ParseJson(dataInfo.buffer, dataInfo.len, document, type, command, items)) {
        response = "JSON format parsing failed";
        return false;
    }

    CmdInfo cmd_info;
    cmd_info.ip = dataInfo.ip;
    cmd_info.port = dataInfo.port;
    cmd_info.fromType = dataInfo.fromType;
    cmd_info.type = type;
    cmd_info.command = command;

    if (handlers_->handle_map[type][command]) {
        handlers_->handle_map[type][command](items, response, cmd_info);
    } else {
        // std::cout << "[CMD] Unknown command from " << cmd_info.ip
        //           << " Type=" << type << " Command=" << command << std::endl;
        response = buildResponse(cmd_info, 1, "Unknown command");
    }
    return true;
}
