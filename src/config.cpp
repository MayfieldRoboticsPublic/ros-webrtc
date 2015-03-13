#include "config.h"

#include "util.h"


Config Config::get(ros::NodeHandle& nh) {
    Config instance;

    // cameras
    XmlRpc::XmlRpcValue cameras_xml;
    if (nh.getParam(param_for("cameras/"), cameras_xml)) {
        for (XmlRpc::XmlRpcValue::iterator i = cameras_xml.begin(); i != cameras_xml.end(); i++) {
            VideoSource camera;
            if (_get(nh, ros::names::append(param_for("cameras/"), (*i).first), camera)) {
                instance.cameras.push_back(camera);
            }
        }
    } else {
        ROS_INFO("missing 'cameras/' param");
    }

    // microphone
    _get(nh, param_for("microphone"), instance.microphone);

    // session constraints
    _get(nh, param_for(ros::names::append("session", "constraints")), instance.session_constraints);

    // ice_servers
    XmlRpc::XmlRpcValue ice_servers_xml;
    if (nh.getParam(param_for("ice_servers"), ice_servers_xml)) {
        for (size_t i = 0; i != ice_servers_xml.size(); i++) {
            webrtc::PeerConnectionInterface::IceServer ice_server;
            if (_get(nh, ice_servers_xml[0], ice_server)) {
                instance.ice_servers.push_back(ice_server);
            }
        }
    } else {
        ROS_INFO("missing 'ice_servers/' param");
    }

    // flush_frequency
    instance.flush_frequency = 10 * 60;  // 10 minutes
    if (nh.hasParam(param_for("flush_frequency"))) {
        if (!nh.getParam(param_for("flush_frequency"), instance.flush_frequency)) {
            ROS_INFO("'flush_frequency' param type not int");
        }
    }

    return instance;
}

void Config::set() {
    throw std::runtime_error("Not implemented.");
}

bool Config::_get(ros::NodeHandle& nh, const std::string& root, VideoSource& value) {
    if (!nh.getParam(ros::names::append(root, "name"), value.name)) {
        return false;
    }
    if (value.name.find("sys://") == 0) {
        value.name = value.name.substr(6);
        value.type = VideoSource::SystemType;
    } else if (value.name.find("ros://") == 0) {
        value.name = value.name.substr(6);
        value.type = VideoSource::ROSType;
    } else {
        value.type = VideoSource::SystemType;
    }
    nh.getParam(ros::names::append(root, "label"), value.label);
    if (!_get(nh, ros::names::append(root, "constraints"), value.constraints)) {
        return false;
    }
    nh.getParam(ros::names::append(root, "publish"), value.publish);
    return true;
}

bool Config::_get(ros::NodeHandle& nh, const std::string& root, AudioSource& value) {
    nh.getParam(ros::names::append(root, "label"), value.label);
    if (!_get(nh, ros::names::append(root, "constraints"), value.constraints)) {
        return false;
    }
    nh.getParam(ros::names::append(root, "publish"), value.publish);
    return true;
}

bool Config::_get(ros::NodeHandle& nh, const std::string& root, MediaConstraints& value) {
    typedef std::map<std::string, std::string> Constraints;
    Constraints constraints;
    std::string key;

    key = ros::names::append(root, "mandatory");
    if (nh.getParam(key, constraints)) {
        for (Constraints::iterator i = constraints.begin(); i != constraints.end(); i++) {
            value.mandatory().push_back(MediaConstraints::Constraint((*i).first, (*i).second));
        }
    }

    key = ros::names::append(root, "optional");
    if (nh.getParam(key, constraints)) {
        for (Constraints::iterator i = constraints.begin(); i != constraints.end(); i++) {
            value.optional().push_back(MediaConstraints::Constraint((*i).first, (*i).second));
        }
    }

    return true;
}

bool Config::_get(ros::NodeHandle& nh, XmlRpc::XmlRpcValue& root, webrtc::PeerConnectionInterface::IceServer& value) {
    if (!root.hasMember("uri")) {
        return false;
    }
    value.uri = std::string(root["uri"]);
    if (root.hasMember("username"))
        value.username = std::string(root["username"]);
    else
        value.username.clear();
    if (root.hasMember("password"))
        value.password = std::string(root["password"]);
    else
        value.password.clear();
    return true;
}
