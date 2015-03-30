#ifndef ROS_WEBRTC_UTIL_H_
#define ROS_WEBRTC_UTIL_H_

#include <initializer_list>
#include <string>

/**
 * \brief Builds (e.g. name spaces, normalizes, etc) a ROS topic from its parts.
 * \param parts List of parts.
 * \return The ROS topic.
 */
std::string topic_for(std::initializer_list<std::string> part);

/**
 * \brief Generates a unique id.
 * \return The unique id as a string.
 */
std::string generate_id();

#endif /* ROS_WEBRTC_UTIL_H_ */
