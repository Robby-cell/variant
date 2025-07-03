#include "variant/variant.hpp"

#include <gtest/gtest.h>

#include <string>

// NOLINTBEGIN

TEST(VariantTest, DefaultConstruction) {
    variant::Variant<int, std::string> v;
    EXPECT_TRUE(v.HasInvalidIndex());
}

TEST(VariantTest, Emplace) {
    variant::Variant<int, std::string> v;
    v.Emplace<int>(42);
    EXPECT_EQ(v.Index(), 0);
    EXPECT_TRUE(v.HoldsAlternative<int>());
    v.Emplace<std::string>("hello");
    EXPECT_EQ(v.Index(), 1);
    EXPECT_TRUE(v.HoldsAlternative<std::string>());
}

TEST(VariantTest, CopyConstruction) {
    variant::Variant<int, std::string> v1;
    v1.Emplace<std::string>("hello");
    auto v2 = v1;
    EXPECT_EQ(v2.Index(), 1);
    EXPECT_TRUE(v2.HoldsAlternative<std::string>());
}

TEST(VariantTest, CopyAssignment) {
    variant::Variant<int, std::string> v1;
    v1.Emplace<int>(42);
    variant::Variant<int, std::string> v2;
    v2.Emplace<std::string>("hello");
    v2 = v1;
    EXPECT_EQ(v2.Index(), 0);
    EXPECT_TRUE(v2.HoldsAlternative<int>());
}

TEST(VariantTest, MoveConstruction) {
    variant::Variant<int, std::string> v1;
    v1.Emplace<std::string>("hello");
    variant::Variant<int, std::string> v2 = std::move(v1);
    EXPECT_EQ(v2.Index(), 1);
    EXPECT_TRUE(v2.HoldsAlternative<std::string>());
    EXPECT_TRUE(v1.HasInvalidIndex());
}

TEST(VariantTest, MoveAssignment) {
    variant::Variant<int, std::string> v1;
    v1.Emplace<int>(42);
    variant::Variant<int, std::string> v2;
    v2.Emplace<std::string>("hello");
    v2 = std::move(v1);
    EXPECT_EQ(v2.Index(), 0);
    EXPECT_TRUE(v2.HoldsAlternative<int>());
    EXPECT_TRUE(v1.HasInvalidIndex());
}

TEST(VariantTest, HoldsAlternative) {
    variant::Variant<int, std::string> v;
    v.Emplace<int>(42);
    EXPECT_TRUE(v.HoldsAlternative<int>());
    EXPECT_FALSE(v.HoldsAlternative<std::string>());
}

TEST(VariantTest, BadVariantAccess) {
    variant::Variant<int, std::string> v;
    EXPECT_THROW(v.Visit([](auto&&) {}), variant::BadVariantAccess);
}

TEST(VariantTest, Monostate) {
    variant::Variant<variant::Monostate, int> v;
    v.Emplace<variant::Monostate>();
    EXPECT_TRUE(v.HoldsAlternative<variant::Monostate>());
}

struct NonDefaultConstructible {
    NonDefaultConstructible(int) {}
};

TEST(VariantTest, NonDefaultConstructible) {
    variant::Variant<NonDefaultConstructible, int> v;
    v.Emplace<int>(42);
    EXPECT_TRUE(v.HoldsAlternative<int>());
    v.Emplace<NonDefaultConstructible>(1);
    EXPECT_TRUE(v.HoldsAlternative<NonDefaultConstructible>());
}

// NOLINTEND