#include <gtest/gtest.h>

#include "variant/variant.hpp"

TEST(BasicVariantTests, SimpleBehaviorWithIndexes) {
  using variant::Variant;

  Variant<int, float, char> v;

  // Should take 4 bytes for storage, and the index should select std::uint8_t
  // 4 bytes + 1 byte, and 3 bytes for padding...
  static_assert(sizeof(v) == 8);

  // Should have invalid index at the moment
  EXPECT_EQ(v.Index(), static_cast<std::size_t>(-1));

  v.Emplace<int>();
  EXPECT_EQ(v.Index(), 0);
  v.Emplace<float>();
  EXPECT_EQ(v.Index(), 1);
  v.Emplace<char>();
  EXPECT_EQ(v.Index(), 2);
}

TEST(BasicVariantTests, ProperlyDestructingElements) {
  static int count = 0;
  struct Type {
    Type() { ++count; }
    Type(const Type &) { ++count; }
    Type(Type &&) noexcept { ++count; }
    Type &operator=(const Type &) {
      ++count;
      return *this;
    }
    Type &operator=(Type &&) noexcept {
      ++count;
      return *this;
    }
    ~Type() { --count; }
  };

  using variant::Variant;

  {
    Variant<Type, int> v;

    // Have not constructed anything yet
    EXPECT_EQ(count, 0);

    v.Emplace<int>();
    EXPECT_EQ(count, 0); // Should not affect count of Type

    v.Emplace<Type>();
    EXPECT_EQ(count, 1); // Should only be one Type.
    v.Emplace<Type>(Type{});
    EXPECT_EQ(count, 1); // Should still only be one Type.

    v.Emplace<int>();
    EXPECT_EQ(count, 0); // Should properly destroy Type

    v.Emplace<Type>();
    EXPECT_EQ(count, 1); // Should be one Type again
  }

  // Should all properly be destroyed
  EXPECT_EQ(count, 0);
}
