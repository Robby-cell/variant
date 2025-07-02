add_executable(example1 example1.cc)
target_link_libraries(example1 PRIVATE variant)
set_target_properties(example1
    PROPERTIES
        CXX_STANDARD 11
        CXX_STANDARD_REQUIRED YES
        CXX_EXTENSIONS NO
)
