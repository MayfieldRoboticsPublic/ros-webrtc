#include <gtest/gtest.h>

#include "cpp/media_type.h"


TEST(TestSuite, testMediaType) {
    MediaType mt;

    mt = MediaType("application/json");
    ASSERT_EQ("application", mt.type);
    ASSERT_FALSE(mt.has_tree());
    ASSERT_EQ("json", mt.sub_type);
    ASSERT_FALSE(mt.has_suffix());
    ASSERT_EQ(0, mt.params.size());

    mt = MediaType("application/vnd.mayfield.msg.v1+json");
    ASSERT_EQ("application", mt.type);
    ASSERT_TRUE(mt.has_tree());
    ASSERT_EQ("vnd", mt.tree);
    ASSERT_EQ("mayfield.msg.v1", mt.sub_type);
    ASSERT_EQ("json", mt.suffix);
    ASSERT_EQ(0, mt.params.size());

    mt = MediaType("application/vnd.mayfield.msg-chunk-v1+json; chunksize=128");
    ASSERT_EQ("application", mt.type);
    ASSERT_TRUE(mt.has_tree());
    ASSERT_EQ("vnd", mt.tree);
    ASSERT_EQ("mayfield.msg-chunk-v1", mt.sub_type);
    ASSERT_EQ("json", mt.suffix);
    ASSERT_EQ(1, mt.params.size());
    ASSERT_EQ("128", mt.params["chunksize"]);

    mt = MediaType("application/vnd.mayfield.msg-chunk-v1+json; chunksize=\"256\"");
    ASSERT_EQ("application", mt.type);
    ASSERT_TRUE(mt.has_tree());
    ASSERT_EQ("vnd", mt.tree);
    ASSERT_EQ("mayfield.msg-chunk-v1", mt.sub_type);
    ASSERT_EQ("json", mt.suffix);
    ASSERT_EQ(1, mt.params.size());
    ASSERT_EQ("256", mt.params["chunksize"]);

    mt = MediaType("application/vnd.mayfield-msg-chunk.v1+json; chunksize=32");
    ASSERT_EQ("application", mt.type);
    ASSERT_TRUE(mt.has_tree());
    ASSERT_EQ("vnd", mt.tree);
    ASSERT_EQ("mayfield-msg-chunk.v1", mt.sub_type);
    ASSERT_EQ("json", mt.suffix);
    ASSERT_EQ(1, mt.params.size());
    ASSERT_EQ("32", mt.params["chunksize"]);
}
