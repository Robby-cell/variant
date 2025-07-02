#include <ios>
#include <iostream>
#include <string>

#include "variant/variant.hpp"

int main() {
    using variant::Variant;

    Variant<int, double, char> v;
    v.Emplace<int>(42);

    using variant::HoldsAlternative, variant::Visit;

    auto str = Visit([](const auto& v) { return std::to_string(v); }, v);
    std::cout << "Variant to string = " << str << '\n';

    auto holds_int = HoldsAlternative<int>(v);
    std::cout << "Holds int: " << std::boolalpha << holds_int << '\n';
}
