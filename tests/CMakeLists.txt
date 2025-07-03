enable_testing()

find_package(gtest CONFIG REQUIRED)

add_executable(tests
  tests.cc
  variant_test.cc
  visitor_test.cc)

target_link_libraries(tests PRIVATE GTest::gtest_main)
target_link_libraries(tests PRIVATE variant)

set_target_properties(tests
  PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)

include(GoogleTest)
gtest_discover_tests(tests)
