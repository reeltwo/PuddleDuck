#pragma once

#include <string>

#define USE_YAML
#ifdef USE_YAML
#include <sys/stat.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include "yaml-cpp/yaml.h"
#endif

#include "PDUtils.h"

#define PDCONFIG_FILE  "robot.yaml"
#define PDDEFAULT_FILE "default.yaml"

namespace PDConfig {

#ifndef MAX_NUM_BUS
#define MAX_NUM_BUS 4
#endif

enum BusType {
    kGoMotor
};

struct Range {
    double value[2];
};

struct Bus {
    PDString name;
    BusType type;
    PDString adapter;
    int version;

    bool isUsed() const {
        return name.length() != 0;
    }
};

struct Neck {
    PDString bus;
    int id;
    Range range;
    double kp;
    double kd;
    double tau;
    bool invert;
};

struct Leg {
    PDString bus;

    struct Joint {
        int id;
        Range range;
        double kp;
        double kd;
        double tau;
        bool invert;
    };     

    struct {
        Joint pitch;
    } ankle;
    struct {
        Joint pitch;
    } knee;
    struct {
        Joint pitch;
        Joint roll;
        Joint yaw;
    } hip;
};

struct Robot {
    Bus bus[MAX_NUM_BUS];
    Neck neck;
    struct {
        Leg left;
        Leg right;
    } leg;

#ifdef USE_YAML
    bool exists(PDString configFile);

    bool load(PDString configFile);

    bool save(PDString configFile);
#endif
}; 

}

#ifdef USE_YAML
namespace YAML {

template<>
struct convert<PDConfig::Bus> {
    static Node encode(const PDConfig::Bus& rhs) {
        Node node;
        node["name"] = rhs.name;
        node["type"] = "GoMotor";
        node["adapter"] = rhs.adapter;
        node["version"] = rhs.version;
        return node;
    }

    static bool decode(const Node& node, PDConfig::Bus& rhs) {
        if (!node["name"] ||
            !node["type"] ||
            !node["adapter"] ||
            !node["version"])
        {
            return false;
        }
        PDString type = node["type"].as<PDString>();
        if (type != "GoMotor") {
            return false;
        }
        rhs.name = node["name"].as<PDString>();
        rhs.type = PDConfig::kGoMotor;
        rhs.adapter = node["adapter"].as<PDString>();
        rhs.version = node["version"].as<int>();
        if (rhs.version != 1 && rhs.version != 2) {
            return false;
        }
        return true;
    }
};

template<>
struct convert<PDConfig::Range> {
    static PDString encodeWithPrecision(double value, int precision) {
        std::stringstream ss;
        if (std::isnan(value)) {
            ss << ".nan";
        } else if (std::isinf(value)) {
            if (std::signbit(value)) {
                ss << "-.inf";
            } else {
                ss << ".inf";
            }
        } else {
            ss << std::fixed << std::setprecision(precision) << value;
        }
        return ss.str();
    }

    static Node encode(const PDConfig::Range& rhs) {
        Node node;

        node.push_back(encodeWithPrecision(rhs.value[0], 4));
        node.push_back(encodeWithPrecision(rhs.value[1], 4));
        node.SetStyle(YAML::EmitterStyle::Flow);
        return node;
    }

    static bool decode(const Node& node, PDConfig::Range& rhs) {
        if (!node.IsSequence() || node.size() != 2) {
            return false;
        }
        rhs.value[0] = node[0].as<double>();
        rhs.value[1] = node[1].as<double>();
        return true;
    }
};

template<>
struct convert<PDConfig::Neck> {
    static Node encode(const PDConfig::Neck& rhs) {
        Node node;
        node["bus"] = rhs.bus;
        node["id"] = rhs.id;
        node["range"] = rhs.range;
        node["kp"] = rhs.kp;
        node["kd"] = rhs.kd;
        node["tau"] = rhs.tau;
        node["invert"] = rhs.invert;
        return node;
    }

    static bool decode(const Node& node, PDConfig::Neck& rhs) {
        if (!node["bus"]   ||
            !node["id"]    ||
            !node["range"] ||
            !node["kp"]    ||
            !node["kd"]    ||
            !node["tau"]   ||
            !node["invert"])
        {
            return false;
        }
        rhs.bus = node["bus"].as<PDString>();
        rhs.id = node["id"].as<int>();
        rhs.range = node["range"].as<PDConfig::Range>();
        rhs.kp = node["kp"].as<double>();
        rhs.kd = node["kd"].as<double>();
        rhs.tau = node["tau"].as<double>();
        rhs.invert = node["invert"].as<bool>();
        return true;
    }
};

template<>
struct convert<PDConfig::Leg::Joint> {
    static Node encode(const PDConfig::Leg::Joint& rhs) {
        Node node;
        node["id"] = rhs.id;
        node["range"] = rhs.range;
        node["kp"] = rhs.kp;
        node["kd"] = rhs.kd;
        node["tau"] = rhs.tau;
        node["invert"] = rhs.invert;
        return node;
    }

    static bool decode(const Node& node, PDConfig::Leg::Joint& rhs) {
        if (!node["id"]    ||
            !node["range"] ||
            !node["kp"]    ||
            !node["kd"]    ||
            !node["tau"]   ||
            !node["invert"])
        {
            return false;
        }
        rhs.id = node["id"].as<int>();
        rhs.range = node["range"].as<PDConfig::Range>();
        rhs.kp = node["kp"].as<double>();
        rhs.kd = node["kd"].as<double>();
        rhs.tau = node["tau"].as<double>();
        rhs.invert = node["invert"].as<bool>();
        return true;
    }
};

template<>
struct convert<PDConfig::Leg> {
    static Node encode(const PDConfig::Leg& rhs) {
        Node node;
        node["bus"] = rhs.bus;
        node["ankle"]["pitch"] = rhs.ankle.pitch;
        node["knee"]["pitch"] = rhs.knee.pitch;
        node["hip"]["pitch"] = rhs.hip.pitch;
        node["hip"]["roll"] = rhs.hip.roll;
        node["hip"]["yaw"] = rhs.hip.yaw;
        return node;
    }

    static bool decode(const Node& node, PDConfig::Leg& rhs) {
        if (!node["bus"] ||
            !node["ankle"] || !node["ankle"]["pitch"] ||
            !node["knee"]  || !node["knee"]["pitch"]  ||
            !node["hip"]   || !node["hip"]["pitch"]   ||
            !node["hip"]   || !node["hip"]["roll"]    ||
            !node["hip"]   || !node["hip"]["yaw"])
        {
            return false;
        }
        rhs.bus = node["bus"].as<PDString>();
        rhs.ankle.pitch = node["ankle"]["pitch"].as<PDConfig::Leg::Joint>();
        rhs.knee.pitch = node["knee"]["pitch"].as<PDConfig::Leg::Joint>();
        rhs.hip.pitch = node["hip"]["pitch"].as<PDConfig::Leg::Joint>();
        rhs.hip.roll = node["hip"]["roll"].as<PDConfig::Leg::Joint>();
        rhs.hip.yaw = node["hip"]["yaw"].as<PDConfig::Leg::Joint>();
        return true;
    }
};

template<>
struct convert<PDConfig::Robot> {
    static Node encode(const PDConfig::Robot& rhs) {
        Node node;
        node["bus"] = YAML::Node(YAML::NodeType::Sequence);
        for (int i = 0; i < MAX_NUM_BUS; ++i) {
            if (rhs.bus[i].isUsed()) {
                node["bus"].push_back(rhs.bus[i]);
            }
        }
        node["neck"] = rhs.neck;
        node["leg"]["left"] = rhs.leg.left;
        node["leg"]["right"] = rhs.leg.right;
        return node;
    }

    static bool decode(const Node& node, PDConfig::Robot& rhs) {
        if (!node["neck"] || !node["leg"]  ||
            !node["leg"]["left"]  ||
            !node["leg"]["right"])
        {
            return false;
        }
        int i = 0;
        for (auto it = node["bus"].begin(); it != node["bus"].end() && i < MAX_NUM_BUS; it++, i++) {
            rhs.bus[i] = it->as<PDConfig::Bus>();
        }
        rhs.neck = node["neck"].as<PDConfig::Neck>();
        rhs.leg.left = node["leg"]["left"].as<PDConfig::Leg>();
        rhs.leg.right = node["leg"]["right"].as<PDConfig::Leg>();
        return true;
    }
};

template <typename T_>
YAML::Emitter& operator<<(YAML::Emitter& out, const T_& rhs) {
  out << convert<T_>::encode(rhs);
  return out;
}

}

#ifdef USE_YAML
bool
PDConfig::Robot::exists(PDString configFile) {
    struct stat buffer;   
    return (stat(configFile.c_str(), &buffer) == 0); 
}

bool
PDConfig::Robot::load(PDString configFile) {
    try {
        YAML::Node config = YAML::LoadFile(configFile);
        *this = config.as<PDConfig::Robot>();
    } catch (YAML::ParserException& ex) {
        std::cerr << "Error parsing: " << ex.what() << std::endl;
        return false;
    } catch (const std::exception& e) {
        std::cerr << "Exception reading " << configFile << ": " << e.what() << std::endl;
        return false;
    }
    return true;
}

bool
PDConfig::Robot::save(PDString configFile) {
    try {
        YAML::Emitter out;
        out.SetFloatPrecision(4);
        out.SetDoublePrecision(4);
        out << *this;

        std::ofstream fout(configFile);
        if (!fout.is_open()) {
            std::cerr << "Failed to open file for writing: " << configFile << std::endl;
            return false;
        }
        fout << "# GENERATED FILE" << std::endl;
        fout << out.c_str() << std::endl;
        fout.close();
    } catch (const YAML::EmitterException& e) {
        std::cerr << "YAML::EmitterException: " << e.what() << std::endl;
    } catch (const std::ios_base::failure& e) {
        std::cerr << "std::ios_base::failure: " << e.what() << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Exception writing " << configFile << ": " << e.what() << std::endl;
        return false;
    }
    return true;
}
#endif
#endif
