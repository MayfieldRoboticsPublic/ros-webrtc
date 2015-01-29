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

const MediaConstraints::Constraints& MediaConstraints::GetMandatory() const {
    return _mandatory;
}

const MediaConstraints::Constraints& MediaConstraints::GetOptional() const {
    return _optional;
}
