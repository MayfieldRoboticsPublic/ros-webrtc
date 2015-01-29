#ifndef IXI_MEDIA_CONSTRAINTS_H_
#define IXI_MEDIA_CONSTRAINTS_H_

#include <talk/app/webrtc/mediaconstraintsinterface.h>

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

#endif /* IXI_MEDIA_CONSTRAINTS_H_ */
