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

struct Signal {
    std::string name;
    int startBit;
    int length;
    bool isSigned;
    double factor;
    double offset;
    std::string unit;
    std::map<int, std::string> valueTable; // For enumerated values
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

    // Helper to split a string by delimiter
    std::vector<std::string> split(const std::string& s, char delim) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delim)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    // Trim whitespace from string
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
                // Parse BO_ (Message)
                if (line.rfind("BO_ ", 0) == 0) {
                    if (inMessage && !currentMessage.signals.empty()) {
                        messages[currentMessage.id] = currentMessage;
                    }
                    currentMessage = Message();
                    inMessage = true;

                    auto tokens = split(line, ' ');
                    if (tokens.size() < 4) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid BO_ format: " << line << std::endl;
                        continue;
                    }

                    // Parse message ID
                    try {
                        currentMessage.id = std::stoul(tokens[1]);
                    } catch (const std::exception& e) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid message ID: " << tokens[1] << std::endl;
                        continue;
                    }

                    currentMessage.name = tokens[2].substr(0, tokens[2].size() - 1);
                    try {
                        currentMessage.dlc = std::stoi(tokens[3]);
                    } catch (const std::exception& e) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid DLC: " << tokens[3] << std::endl;
                        continue;
                    }
                }
                // Parse SG_ (Signal)
                else if (line.rfind("SG_ ", 0) == 0 && inMessage) {
                    Signal signal;
                    auto tokens = split(line, ' ');
                    if (tokens.size() < 7) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid SG_ format: " << line << std::endl;
                        continue;
                    }

                    signal.name = tokens[1]; // Fix: Use tokens[1] for signal name

                    // Parse startBit|length@byteOrder (e.g., 60|4@1+)
                    auto bitInfo = split(tokens[3], '|');
                    if (bitInfo.size() < 2) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid bit info format: " << tokens[3] << std::endl;
                        continue;
                    }

                    try {
                        signal.startBit = std::stoi(bitInfo[0]);
                    } catch (const std::exception& e) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid start bit: " << bitInfo[0] << std::endl;
                        continue;
                    }

                    auto lengthInfo = split(bitInfo[1], '@');
                    if (lengthInfo.size() < 2) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid length format: " << bitInfo[1] << std::endl;
                        continue;
                    }

                    try {
                        signal.length = std::stoi(lengthInfo[0]);
                    } catch (const std::exception& e) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid signal length: " << lengthInfo[0] << std::endl;
                        continue;
                    }

                    signal.isSigned = (lengthInfo[1].find('+') != std::string::npos) ? false : true;

                    // Parse factor,offset (e.g., (0.1,0))
                    auto scale = split(tokens[4].substr(1, tokens[4].size() - 2), ',');
                    if (scale.size() < 2) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid scale format: " << tokens[4] << std::endl;
                        continue;
                    }

                    try {
                        signal.factor = std::stod(scale[0]);
                        signal.offset = std::stod(scale[1]);
                    } catch (const std::exception& e) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid factor/offset: " << tokens[4] << std::endl;
                        continue;
                    }

                    // Parse unit
                    signal.unit = tokens[6].substr(1, tokens[6].size() - 2);
                    currentMessage.signals.push_back(signal);
                }
                // Parse VAL_ (Value Table for enumerated signals)
                else if (line.rfind("VAL_ ", 0) == 0) {
                    auto tokens = split(line, ' ');
                    if (tokens.size() < 4) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid VAL_ format: " << line << std::endl;
                        continue;
                    }

                    uint32_t messageId;
                    try {
                        messageId = std::stoul(tokens[2]);
                    } catch (const std::exception& e) {
                        std::cerr << "Error at line " << lineNumber << ": Invalid VAL_ message ID: " << tokens[2] << std::endl;
                        continue;
                    }

                    std::string signalName = tokens[3];
                    std::map<int, std::string> valueTable;
                    for (size_t i = 4; i < tokens.size() - 1; i += 2) {
                        if (i + 1 >= tokens.size()) {
                            std::cerr << "Error at line " << lineNumber << ": Incomplete VAL_ entry: " << line << std::endl;
                            break;
                        }
                        int value;
                        try {
                            value = std::stoi(tokens[i]);
                        } catch (const std::exception& e) {
                            std::cerr << "Error at line " << lineNumber << ": Invalid VAL_ value: " << tokens[i] << std::endl;
                            continue;
                        }
                        std::string desc = tokens[i + 1];
                        if (desc.front() == '"' && desc.back() == '"') {
                            desc = desc.substr(1, desc.size() - 2);
                        }
                        valueTable[value] = desc;
                    }

                    // Assign value table to the corresponding signal
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
                std::cerr << "Exception at line " << lineNumber << ": " << e.what() << " in line: " << line << std::endl;
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

class CANReader {
private:
    int socketFd;
    DBCParser parser;

    // Decode a signal from CAN frame data
    double decodeSignal(const can_frame& frame, const Signal& signal) {
        uint64_t raw = 0;
        int startByte = signal.startBit / 8;
        int startBitInByte = signal.startBit % 8;
        int bitsRemaining = signal.length;

        // Extract bits in big-endian order
        int bitPosition = 0;
        for (int i = startByte; i < frame.can_dlc && bitsRemaining > 0; i++) {
            uint8_t byte = frame.data[i];
            int bitsInThisByte = std::min(8 - startBitInByte, bitsRemaining);
            uint64_t mask = (1ULL << bitsInThisByte) - 1;
            uint64_t value = (byte >> startBitInByte) & mask;
            raw |= (value << bitPosition);
            bitPosition += bitsInThisByte;
            bitsRemaining -= bitsInThisByte;
            startBitInByte = 0; // Reset for next byte
        }

        // Handle signed values
        if (signal.isSigned && (raw & (1ULL << (signal.length - 1)))) {
            uint64_t signMask = ~((1ULL << signal.length) - 1);
            raw |= signMask;
        }

        // Apply factor and offset
        return raw * signal.factor + signal.offset;
    }

public:
    CANReader(const std::string& dbcFile) {
        parser.parseDBC(dbcFile);
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

    void readAndParse() {
        struct can_frame frame;
        while (true) {
            int nbytes = read(socketFd, &frame, sizeof(frame));
            if (nbytes < 0) {
                std::cerr << "Error reading CAN frame: " << strerror(errno) << std::endl;
                break;
            }

            if (nbytes == sizeof(frame)) {
                auto it = parser.getMessages().find(frame.can_id);
                if (it != parser.getMessages().end()) {
                    const Message& msg = it->second;
                    std::cout << "Message: " << msg.name << " (ID: " << frame.can_id << ")\n";
                    for (const auto& signal : msg.signals) {
                        double value = decodeSignal(frame, signal);
                        std::cout << "  Signal: " << signal.name << " = ";

                        // Check if signal has a value table (for enumerated signals)
                        if (!signal.valueTable.empty()) {
                            int intValue = static_cast<int>(value);
                            auto valIt = signal.valueTable.find(intValue);
                            if (valIt != signal.valueTable.end()) {
                                std::cout << valIt->second;
                            } else {
                                std::cout << value;
                            }
                        } else {
                            std::cout << value;
                        }
                        std::cout << " " << signal.unit << "\n";
                    }
                    std::cout << std::endl; // Add newline for readability
                }
            }
        }
    }

    ~CANReader() {
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
    CANReader reader(dbcFile);

    if (!reader.initSocket("vcan0")) {
        return 1;
    }

    std::cout << "Reading CAN data from can0 interface...\n";
    reader.readAndParse();

    return 0;
}