#include "util.h"

#include <cctype>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/random_generator.hpp>
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
        if (isalnum(name[i]) ||
            name[i] == '_' ||
            name[i] == '/' ||
            name[i] == '~') {
            normalized += name[i];
        }
    }
    return normalized;
}

std::string join_names(std::initializer_list<std::string> parts) {
    std::string path = "";
    if (parts.size() != 0) {
        auto i = parts.begin();
        path = normalize_name(*i);
        i += 1;
        for(; i != parts.end(); i += 1) {
            path = ros::names::append(path, normalize_name(*i));
        }
    }
    return path;
}

std::string generate_id() {
    boost::uuids::uuid uuid = boost::uuids::random_generator()();
    return boost::lexical_cast<std::string>(uuid);
}
