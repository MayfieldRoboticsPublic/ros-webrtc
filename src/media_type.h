#ifndef ROS_WEBRTC_MEDIA_TYPE_H_
#define ROS_WEBRTC_MEDIA_TYPE_H_

#include <map>
#include <string>

/**
 * \brief Represents media type components (type, tree, etc).
 */
struct MediaType {

    /**
     * \brief Parses components from a string representation of a media type.
     * \param value Media type as a string.
     * \return Components parsed from value as a MediaType.
     */
    static MediaType parse(const std::string& value);

    bool has_tree() const;

    bool has_suffix() const;

    std::string type;

    std::string tree;

    std::string sub_type;

    std::string suffix;

    std::map<std::string, std::string> params;

};

#endif /* ROS_WEBRTC_MEDIA_TYPE_H_ */
