#include <gtest/gtest.h>

#include "cpp/media_constraints.h"


TEST(TestSuite, testMediaConstraints) {
    MediaConstraints mc;

    // defaults to empty
    ASSERT_EQ(0, mc.mandatory().size());
    ASSERT_EQ(0, mc.GetMandatory().size());
    ASSERT_EQ(0, mc.optional().size());
    ASSERT_EQ(0, mc.GetOptional().size());

    // add mandatory
    mc.mandatory().push_back(MediaConstraints::Constraint(MediaConstraints::kMaxWidth, "1024"));
    mc.mandatory().push_back(MediaConstraints::Constraint(MediaConstraints::kMinWidth, "320"));
    ASSERT_EQ(2, mc.mandatory().size());
    ASSERT_EQ(2, mc.GetMandatory().size());
    ASSERT_EQ(0, mc.optional().size());
    ASSERT_EQ(0, mc.GetOptional().size());

    // add optional
    mc.optional().push_back(MediaConstraints::Constraint(MediaConstraints::kEchoCancellation, "true"));
    ASSERT_EQ(2, mc.mandatory().size());
    ASSERT_EQ(2, mc.GetMandatory().size());
    ASSERT_EQ(1, mc.optional().size());
    ASSERT_EQ(1, mc.GetOptional().size());

    // duplicate optional
    mc.optional().push_back(MediaConstraints::Constraint(MediaConstraints::kEchoCancellation, "false"));
    ASSERT_EQ(2, mc.mandatory().size());
    ASSERT_EQ(2, mc.GetMandatory().size());
    ASSERT_EQ(2, mc.optional().size());
    ASSERT_EQ(2, mc.GetOptional().size());

    std::string value;
    ASSERT_TRUE(mc.optional().FindFirst(MediaConstraints::kEchoCancellation, &value));
    ASSERT_EQ("true", value);
}

