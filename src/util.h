#ifndef WEBRTC_UTIL_H_
#define WEBRTC_UTIL_H_

#include <string>

/**
 * @brief Removes unsupported characters from a ROS name (see http://wiki.ros.org/Names#Valid_Names-1).
 * @return The normalized name.
 */
std::string normalize_name(const std::string& name);

/**
 * @brief Builds (e.g. name spaces, normalized, etc) a ROS topic from a group and type.
 * @param group A group name.
 * @param type A type name.
 * @return The ROS topic.
 */
std::string topic_for(const std::string& group, const std::string& type);

/**
 * @brief Builds (e.g. name spaces, normalized, etc) a ROS service name.
 * @param name Base service name.
 * @return The ROS service name.
 */
std::string service_for(const std::string& name);

/**
 * @brief Builds (e.g. name spaces, normalized, etc) a ROS topic.
 * @param name Base topic name.
 * @return The ROS topic name.
 */
std::string topic_for(const std::string& name);

/**
 * @brief Builds (e.g. name spaces, normalized, etc) a ROS param.
 * @param name Base param name.
 * @return The ROS param name.
 */
std::string param_for(const std::string& name);

#endif /* WEBRTC_UTIL_H_ */
