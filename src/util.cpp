#include "util.h"

#include <cctype>

#include <ros/ros.h>

std::string normalize_name(const std::string& name) {
    std::string normalized;
    normalized.reserve(name.size());
    size_t i = 0;
    if (isalpha(name[i])) {
        normalized += name[i];
    }
    i++;
    for(; i != name.size(); i++) {
        if (isalnum(name[i]) || name[i] == '_' || name[i] == '/') {
            normalized += name[i];
        }
    }
    return normalized;
}

std::string topic_for(const std::string& group, const std::string& type) {
    return ros::names::append(
        "ros_webrtc", ros::names::append(normalize_name(group), normalize_name(type))
    );
}

std::string service_for(const std::string& name) {
    return ros::names::append(
        "ros_webrtc", normalize_name(name)
    );
}

std::string topic_for(const std::string& name) {
    return ros::names::append(
        "ros_webrtc", normalize_name(name)
    );
}

std::string param_for(const std::string& name) {
    return ros::names::append(
        "ros_webrtc", normalize_name(name)
    );
}
