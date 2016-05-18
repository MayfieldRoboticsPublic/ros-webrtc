#include "media_constraints.h"

MediaConstraints::~MediaConstraints() {
}

MediaConstraints::Constraints& MediaConstraints::mandatory() {
    return _mandatory;
}

const MediaConstraints::Constraints& MediaConstraints::mandatory() const {
    return _mandatory;
}

MediaConstraints::Constraints& MediaConstraints::optional() {
    return _optional;
}

const MediaConstraints::Constraints& MediaConstraints::optional() const {
    return _optional;
}

MediaConstraints::operator ros_webrtc::MediaConstraints () const {
    ros_webrtc::MediaConstraints dst;
    for (auto i = mandatory().begin(); i != mandatory().end(); i++) {
        ros_webrtc::Constraint constraint;
        constraint.key = (*i).key;
        constraint.value = (*i).value;
        dst.mandatory.push_back(constraint);
    }
    for (auto i = optional().begin(); i != optional().end(); i++) {
        ros_webrtc::Constraint constraint;
        constraint.key = (*i).key;
        constraint.value = (*i).value;
        dst.optional.push_back(constraint);
    }
    return dst;
}

const MediaConstraints::Constraints& MediaConstraints::GetMandatory() const {
    return _mandatory;
}

const MediaConstraints::Constraints& MediaConstraints::GetOptional() const {
    return _optional;
}
