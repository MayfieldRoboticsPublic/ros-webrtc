#include "media_constraints.h"
#include "media_type.h"

#include <gtest/gtest.h>

// MediaType

TEST(TestSuite, testMediaType) {
    MediaType mt;

    mt = MediaType::parse("application/json");
    ASSERT_EQ("application", mt.type);
    ASSERT_FALSE(mt.has_tree());
    ASSERT_EQ("json", mt.sub_type);
    ASSERT_FALSE(mt.has_suffix());
    ASSERT_EQ(0, mt.params.size());

    mt = MediaType::parse("application/vnd.mayfield.msg.v1+json");
    ASSERT_EQ("application", mt.type);
    ASSERT_TRUE(mt.has_tree());
    ASSERT_EQ("vnd", mt.tree);
    ASSERT_EQ("mayfield.msg.v1", mt.sub_type);
    ASSERT_EQ("json", mt.suffix);
    ASSERT_EQ(0, mt.params.size());

    mt = MediaType::parse("application/vnd.mayfield.msg-chunk-v1+json; chunksize=128");
    ASSERT_EQ("application", mt.type);
    ASSERT_TRUE(mt.has_tree());
    ASSERT_EQ("vnd", mt.tree);
    ASSERT_EQ("mayfield.msg-chunk-v1", mt.sub_type);
    ASSERT_EQ("json", mt.suffix);
    ASSERT_EQ(1, mt.params.size());
    ASSERT_EQ("128", mt.params["chunksize"]);

    mt = MediaType::parse("application/vnd.mayfield.msg-chunk-v1+json; chunksize=\"256\"");
    ASSERT_EQ("application", mt.type);
    ASSERT_TRUE(mt.has_tree());
    ASSERT_EQ("vnd", mt.tree);
    ASSERT_EQ("mayfield.msg-chunk-v1", mt.sub_type);
    ASSERT_EQ("json", mt.suffix);
    ASSERT_EQ(1, mt.params.size());
    ASSERT_EQ("256", mt.params["chunksize"]);

    mt = MediaType::parse("application/vnd.mayfield-msg-chunk.v1+json; chunksize=32");
    ASSERT_EQ("application", mt.type);
    ASSERT_TRUE(mt.has_tree());
    ASSERT_EQ("vnd", mt.tree);
    ASSERT_EQ("mayfield-msg-chunk.v1", mt.sub_type);
    ASSERT_EQ("json", mt.suffix);
    ASSERT_EQ(1, mt.params.size());
    ASSERT_EQ("32", mt.params["chunksize"]);
}

// MediaConstraints

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

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
