#ifndef ROS_WEBRTC_UTIL_H_
#define ROS_WEBRTC_UTIL_H_

#include <initializer_list>
#include <string>

/**
 * \brief Normalizes to a ROS name.
 * \param name Candidate to normalize.
 * \return ROS name.
 */
std::string normalize_name(const std::string& name);

/**
 * \brief Normalizes and joins to from  a ROS name.
 * \param parts List of parts that form the name.
 * \return The ROS name.
 */
std::string join_names(std::initializer_list<std::string> parts);

/**
 * \brief Generates a unique id.
 * \return The unique id as a string.
 */
std::string generate_id();

#endif /* ROS_WEBRTC_UTIL_H_ */
