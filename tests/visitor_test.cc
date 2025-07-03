#include <gtest/gtest.h>

#include <string>

#include "variant/variant.hpp"

// NOLINTBEGIN

struct Visitor {
    std::string operator()(int) { return "int"; }
    std::string operator()(const std::string&) { return "string"; }
    std::string operator()(double) { return "double"; }
};

TEST(VisitorTest, Basic) {
    variant::Variant<int, std::string, double> v;
    v.Emplace<int>(42);
    EXPECT_EQ(v.Visit(Visitor{}), "int");
    v.Emplace<std::string>("hello");
    EXPECT_EQ(v.Visit(Visitor{}), "string");
    v.Emplace<double>(3.14);
    EXPECT_EQ(v.Visit(Visitor{}), "double");
}

struct ConstVisitor {
    std::string operator()(int) const { return "int"; }
    std::string operator()(const std::string&) const { return "string"; }
    std::string operator()(double) const { return "double"; }
};

TEST(VisitorTest, Const) {
    variant::Variant<int, std::string, double> v;
    v.Emplace<int>(42);
    EXPECT_EQ(v.Visit(ConstVisitor{}), "int");
    v.Emplace<std::string>("hello");
    EXPECT_EQ(v.Visit(ConstVisitor{}), "string");
    v.Emplace<double>(3.14);
    EXPECT_EQ(v.Visit(ConstVisitor{}), "double");
}

TEST(VisitorTest, Overloaded) {
    struct OverloadedVisitor {
        void operator()(int&) {}
        void operator()(const int&) {}
        void operator()(int&&) {}
        void operator()(const int&&) {}

        void operator()(variant::Monostate) {}
    };

    variant::Variant<variant::Monostate, int> v;
    v.Emplace<int>(42);

    v.Visit(OverloadedVisitor{});
    const auto& cv = v;
    cv.Visit(OverloadedVisitor{});
    std::move(v).Visit(OverloadedVisitor{});
    std::move(cv).Visit(OverloadedVisitor{});
}

TEST(VisitorTest, ReturnType) {
    variant::Variant<int, std::string> v;
    v.Emplace<int>(42);
    auto f = [](auto&&) -> int { return 1; };
    int result = v.Visit(f);
    EXPECT_EQ(result, 1);
}

TEST(VisitorTest, GlobalVisit) {
    variant::Variant<int, std::string> v;
    v.Emplace<int>(42);
    struct V {
        std::string operator()(int) { return "int"; }
        std::string operator()(const std::string&) { return "string"; }
    };
    EXPECT_EQ(variant::Visit(V{}, v), "int");
}

TEST(VisitorTest, GlobalHoldsAlternative) {
    variant::Variant<int, std::string> v;
    v.Emplace<int>(42);
    EXPECT_TRUE(variant::HoldsAlternative<int>(v));
    EXPECT_FALSE(variant::HoldsAlternative<std::string>(v));
}

// NOLINTEND