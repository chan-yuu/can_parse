#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>
#include <string>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>
#include <cstdint>
#include <memory>
#include <cmath>
#include "control_command.h"

using namespace chassis::common;

struct Signal {
    std::string name;
    int startBit;
    int length;
    bool isSigned;
    bool isMotorola;
    double factor;
    double offset;
    std::string unit;
    std::map<int, std::string> valueTable;
};

struct Message {
    uint32_t id;
    std::string name;
    int dlc;
    std::vector<Signal> signals;
};

class DBCParser {
private:
    std::map<uint32_t, Message> messages;

    std::vector<std::string> split(const std::string& s, char delim) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delim)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(" \t");
        size_t last = str.find_last_not_of(" \t");
        if (first == std::string::npos) return "";
        return str.substr(first, last - first + 1);
    }

public:
    void parseDBC(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open DBC file: " << filename << std::endl;
            return;
        }

        std::string line;
        Message currentMessage;
        bool inMessage = false;
        int lineNumber = 0;

        while (std::getline(file, line)) {
            lineNumber++;
            line = trim(line);
            if (line.empty()) continue;

            try {
                if (line.rfind("BO_ ", 0) == 0) {
                    if (inMessage && !currentMessage.signals.empty()) {
                        messages[currentMessage.id] = currentMessage;
                    }
                    currentMessage = Message();
                    inMessage = true;

                    auto tokens = split(line, ' ');
                    if (tokens.size() < 4) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid BO_ format" << std::endl;
                        continue;
                    }

                    currentMessage.id = std::stoul(tokens[1]);
                    currentMessage.name = tokens[2].substr(0, tokens[2].size() - 1);
                    currentMessage.dlc = std::stoi(tokens[3]);
                }
                else if (line.rfind("SG_ ", 0) == 0 && inMessage) {
                    Signal signal;
                    auto tokens = split(line, ' ');
                    if (tokens.size() < 7) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid SG_ format" << std::endl;
                        continue;
                    }

                    signal.name = tokens[1];
                    auto bitInfo = split(tokens[3], '|');
                    signal.startBit = std::stoi(bitInfo[0]);
                    auto lengthInfo = split(bitInfo[1], '@');
                    signal.length = std::stoi(lengthInfo[0]);
                    signal.isMotorola = (lengthInfo[1].find('0') != std::string::npos);
                    signal.isSigned = (lengthInfo[1].find('+') == std::string::npos);

                    auto scale = split(tokens[4].substr(1, tokens[4].size() - 2), ',');
                    signal.factor = std::stod(scale[0]);
                    signal.offset = std::stod(scale[1]);
                    signal.unit = tokens[6].substr(1, tokens[6].size() - 2);
                    currentMessage.signals.push_back(signal);
                }
                else if (line.rfind("VAL_ ", 0) == 0) {
                    auto tokens = split(line, ' ');
                    if (tokens.size() < 4) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid VAL_ format" << std::endl;
                        continue;
                    }

                    uint32_t messageId = std::stoul(tokens[2]);
                    std::string signalName = tokens[3];
                    std::map<int, std::string> valueTable;
                    for (size_t i = 4; i < tokens.size() - 1; i += 2) {
                        int value = std::stoi(tokens[i]);
                        std::string desc = tokens[i + 1];
                        if (desc.front() == '"' && desc.back() == '"') {
                            desc = desc.substr(1, desc.size() - 2);
                        }
                        valueTable[value] = desc;
                    }

                    if (messages.find(messageId) != messages.end()) {
                        for (auto& signal : messages[messageId].signals) {
                            if (signal.name == signalName) {
                                signal.valueTable = valueTable;
                                break;
                            }
                        }
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "Exception at line " << lineNumber << ": " << e.what() << std::endl;
            }
        }
        if (inMessage && !currentMessage.signals.empty()) {
            messages[currentMessage.id] = currentMessage;
        }
        file.close();
    }

    const std::map<uint32_t, Message>& getMessages() const {
        return messages;
    }
};

class CANPublisher {
private:
    int socketFd;
    DBCParser parser;

    void encodeSignal(can_frame& frame, const Signal& signal, double value) {
        double rawValue = (value - signal.offset) / signal.factor;
        uint64_t raw = static_cast<uint64_t>(std::round(rawValue));

        uint64_t maxValue = (1ULL << signal.length) - 1;
        if (signal.isSigned) {
            int64_t signedRaw = static_cast<int64_t>(raw);
            int64_t minSigned = -(1LL << (signal.length - 1));
            int64_t maxSigned = (1LL << (signal.length - 1)) - 1;
            if (signedRaw < minSigned) raw = static_cast<uint64_t>(minSigned);
            if (signedRaw > maxSigned) raw = static_cast<uint64_t>(maxSigned);
        } else {
            if (raw > maxValue) raw = maxValue;
        }

        int startByte = signal.startBit / 8;
        int startBitInByte = signal.startBit % 8;
        int bitsRemaining = signal.length;

        if (signal.isMotorola) {
            int bitPosition = signal.length - 1;
            for (int i = startByte; i < frame.can_dlc && bitsRemaining > 0; i++) {
                int bitsInThisByte = std::min(8 - startBitInByte, bitsRemaining);
                for (int j = 0; j < bitsInThisByte; j++) {
                    int bit = (raw >> (bitPosition - j)) & 1;
                    frame.data[i] &= ~(1 << (7 - (startBitInByte + j)));
                    frame.data[i] |= (bit << (7 - (startBitInByte + j)));
                }
                bitsRemaining -= bitsInThisByte;
                bitPosition -= bitsInThisByte;
                startBitInByte = 0;
            }
        } else {
            int bitPosition = 0;
            for (int i = startByte; i < frame.can_dlc && bitsRemaining > 0; i++) {
                int bitsInThisByte = std::min(8 - startBitInByte, bitsRemaining);
                uint64_t mask = (1ULL << bitsInThisByte) - 1;
                uint64_t shiftedValue = (raw >> bitPosition) & mask;
                frame.data[i] &= ~(mask << startBitInByte);
                frame.data[i] |= (shiftedValue << startBitInByte);
                bitPosition += bitsInThisByte;
                bitsRemaining -= bitsInThisByte;
                startBitInByte = 0;
            }
        }
    }

public:
    CANPublisher(const std::string& dbcFile) {
        parser.parseDBC(dbcFile);
        socketFd = -1;
    }

    bool initSocket(const std::string& interface) {
        socketFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socketFd < 0) {
            std::cerr << "Error creating socket: " << strerror(errno) << std::endl;
            return false;
        }

        struct ifreq ifr;
        strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        ifr.ifr_name[IFNAMSIZ - 1] = '\0';
        if (ioctl(socketFd, SIOCGIFINDEX, &ifr) < 0) {
            std::cerr << "Error getting interface index: " << strerror(errno) << std::endl;
            close(socketFd);
            return false;
        }

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socketFd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Error binding socket: " << strerror(errno) << std::endl;
            close(socketFd);
            return false;
        }

        return true;
    }

    bool publish(const ControlCommand& cmd) {
        if (!cmd.isValid()) {
            std::cerr << "Invalid ControlCommand" << std::endl;
            return false;
        }

        std::map<uint32_t, can_frame> frames;
        for (const auto& msgPair : parser.getMessages()) {
            frames[msgPair.first] = can_frame{};
            frames[msgPair.first].can_id = msgPair.first;
            frames[msgPair.first].can_dlc = msgPair.second.dlc;
            std::memset(frames[msgPair.first].data, 0, 8);
        }

        for (const auto& msgPair : parser.getMessages()) {
            uint32_t msgId = msgPair.first;
            const Message& msg = msgPair.second;

            for (const auto& signal : msg.signals) {
                // 通过信号名称匹配，不再依赖硬编码的msgId
                if (signal.name == "AD_Accelerate_Pedal") {
                    encodeSignal(frames[msgId], signal, cmd.accelerator);
                } else if (signal.name == "AD_Speed_Req") {
                    encodeSignal(frames[msgId], signal, cmd.target_velocity * 3.6);
                } else if (signal.name == "AD_Accelerate_Gear") {
                    encodeSignal(frames[msgId], signal, cmd.target_gear);
                } else if (signal.name == "AD_Accelerate_Valid") {
                    encodeSignal(frames[msgId], signal, 1);
                } else if (signal.name == "AD_BrakePressure_Req") {
                    encodeSignal(frames[msgId], signal, cmd.brake);
                } else if (signal.name == "AD_DBS_Valid") {
                    encodeSignal(frames[msgId], signal, 1);
                } else if (signal.name == "AD_Steering_Angle_Cmd") {
                    encodeSignal(frames[msgId], signal, cmd.target_steering_angle * 57.2958);
                } else if (signal.name == "AD_Steering_Valid") {
                    encodeSignal(frames[msgId], signal, 1);
                } else if (signal.name == "AD_Left_Turn_Light") {
                    encodeSignal(frames[msgId], signal, cmd.turn_lights == 1 ? 1 : 0);
                } else if (signal.name == "AD_Right_Turn_Light") {
                    encodeSignal(frames[msgId], signal, cmd.turn_lights == 2 ? 1 : 0);
                } else if (signal.name == "AD_Double_Flash_Light") {
                    encodeSignal(frames[msgId], signal, cmd.turn_lights == 3 ? 1 : 0);
                } else if (signal.name == "AD_Low_Beam") {
                    encodeSignal(frames[msgId], signal, cmd.head_lights == 1 ? 1 : 0);
                } else if (signal.name == "AD_High_Beam") {
                    encodeSignal(frames[msgId], signal, cmd.head_lights == 2 ? 1 : 0);
                } else if (signal.name == "AD_Horn_1_Control" || signal.name == "AD_Horn_2_Control") {
                    encodeSignal(frames[msgId], signal, cmd.horn_chassis);
                } else if (signal.name == "AD_Fof_Light") {
                    encodeSignal(frames[msgId], signal, cmd.fog_lamp_front || cmd.fog_lamp_rear);
                } else if (signal.name == "AD_Body_Valid") {
                    encodeSignal(frames[msgId], signal, 1);
                }
            }
        }

        for (const auto& framePair : frames) {
            if (write(socketFd, &framePair.second, sizeof(framePair.second)) != sizeof(framePair.second)) {
                std::cerr << "Error sending CAN frame ID " << framePair.first << ": " << strerror(errno) << std::endl;
                return false;
            }
        }

        return true;
    }

    ~CANPublisher() {
        if (socketFd >= 0) {
            close(socketFd);
        }
    }
};

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_dbc_file>" << std::endl;
        return 1;
    }

    std::string dbcFile = argv[1];
    CANPublisher publisher(dbcFile);

    if (!publisher.initSocket("can0")) {
        return 1;
    }

    ControlCommand cmd;
    cmd.timestamp = 1234567890.0;
    cmd.accelerator = 50.0;
    cmd.brake = 30.0;
    cmd.target_velocity = 10.0;
    cmd.target_steering_angle = 0.5;
    cmd.target_gear = 1;
    cmd.turn_lights = 1;
    cmd.head_lights = 1;
    cmd.horn_chassis = 1;
    cmd.fog_lamp_front = 1;
    cmd.fog_lamp_rear = 0;
    cmd.wiper_chassis = 0;

    std::cout << "Starting CAN message publishing loop (press Ctrl+C to stop)...\n";
    while (true) {
        if (publisher.publish(cmd)) {
            std::cout << "Successfully published CAN messages\n";
        } else {
            std::cout << "Failed to publish CAN messages\n";
        }
        usleep(20000); // 20ms delay to match typical DBC cycle time
    }

    return 0;
}