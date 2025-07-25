#include <ios>
#include <iostream>
#include <string>
#include <utility>

#include "variant/variant.hpp"

struct Visitor {
    template <class Ty>
    std::string operator()(const Ty& v) const {
        return std::to_string(v);
    }
};

int main() {
    using variant::Variant;

    Variant<int, double, char> v;
    v.Emplace<int>(42);

    using variant::HoldsAlternative;
    using variant::Visit;

    const auto str = Visit(Visitor{}, v);
    std::cout << "Variant to string = " << str << '\n';

    const auto holds_int = HoldsAlternative<int>(v);
    std::cout << "Holds int: " << std::boolalpha << holds_int << '\n';

    const auto copy = v;
    const auto moved = std::move(v);

    const auto definitely_an_int = copy.Get<int>();
    std::cout << "The int held by the copied variant: " << definitely_an_int
              << '\n';

    const auto definitely_an_int_also = moved.Get<int>();
    std::cout << "The int held by the moved variant: " << definitely_an_int_also
              << '\n';
}
