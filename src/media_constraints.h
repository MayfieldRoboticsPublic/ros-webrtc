#ifndef ROS_WEBRTC_MEDIA_CONSTRAINTS_H_
#define ROS_WEBRTC_MEDIA_CONSTRAINTS_H_

#include <talk/app/webrtc/mediaconstraintsinterface.h>

/**
 * @class MediaConstraints
 */
class MediaConstraints : public webrtc::MediaConstraintsInterface {

public:

    virtual ~MediaConstraints();

    Constraints& mandatory();

    const Constraints& mandatory() const;

    Constraints& optional();

    const Constraints& optional() const;

// webrtc::MediaConstraintsInterface

public:

    virtual const Constraints& GetMandatory() const;

    virtual const Constraints& GetOptional() const;

private:

    Constraints _mandatory;

    Constraints _optional;

};

#endif /* ROS_WEBRTC_MEDIA_CONSTRAINTS_H_ */
