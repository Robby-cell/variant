#ifndef VARIANT_VARIANT_HPP
#define VARIANT_VARIANT_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

namespace variant {

template <class... Ts>
class Variant;

namespace detail {
template <std::size_t Count>
struct StorageIndex {
    using Type = std::conditional_t<
        Count <= std::numeric_limits<std::uint8_t>::max(), std::uint8_t,
        std::conditional_t<
            Count <= std::numeric_limits<std::uint16_t>::max(), std::uint16_t,
            std::conditional_t<Count <=
                                   std::numeric_limits<std::uint32_t>::max(),
                               std::uint32_t, std::uint64_t>>>;
};

template <class T, class... Ts>
struct AllTheSame;

template <class T, class T2, class... Ts>
struct AllTheSame<T, T2, Ts...> {
    static constexpr bool Value =
        (std::is_same<T, T2>::value and AllTheSame<T2, Ts...>::Value);
};

template <class T, class T2>
struct AllTheSame<T, T2> {
    static constexpr bool Value = (std::is_same<T, T2>::value);
};

template <class T>
struct AllTheSame<T> {
    static constexpr bool Value = true;
};

template <class T, class... Ts>
constexpr auto AllTheSameV = AllTheSame<T, Ts...>::Value;

template <class T, class... Ts>
struct IsOneOf;

template <class T, class... Ts>
struct IsOneOf<T, T, Ts...> {
    static constexpr bool Value = true;
};

template <class T, class U, class... Ts>
struct IsOneOf<T, U, Ts...> {
    static constexpr bool Value = IsOneOf<T, Ts...>::Value;
};

template <class T>
struct IsOneOf<T> {
    static constexpr bool Value = false;
};

template <class T, class... Ts>
constexpr auto IsOneOfV = IsOneOf<T, Ts...>::Value;

template <class T, class... Ts>
struct IsUnique {
    static constexpr bool Value = !IsOneOfV<T, Ts...>;
};

template <class T, class... Ts>
constexpr auto IsUniqueV = IsUnique<T, Ts...>::Value;

template <class, class...>
struct AllUnique;

template <class T, class U, class... Ts>
struct AllUnique<T, U, Ts...> {
    static constexpr bool Value =
        IsUniqueV<T, U, Ts...> and AllUnique<U, Ts...>::Value;
};

template <class T>
struct AllUnique<T> {
    static constexpr bool Value = true;
};

template <class... Ts>
constexpr auto AllUniqueV = AllUnique<Ts...>::Value;

template <template <class...> class, class...>
struct IsInstanceOf {
    static constexpr bool Value = false;
};

template <template <class...> class T, class... Ts>
struct IsInstanceOf<T, T<Ts...>> {
    static constexpr bool Value = true;
};

template <template <class...> class T, class... Ts>
constexpr auto IsInstanceOfV = IsInstanceOf<T, Ts...>::Value;

template <typename T>
struct RemoveCVRef {
    using Type =
        typename std::remove_cv<typename std::remove_reference<T>::type>::type;
};

template <typename T, typename... Ts>
struct IndexOfType;

template <typename T, typename... Ts>
struct IndexOfType<T, T, Ts...> {
    static constexpr std::size_t Value = 0;
};

template <typename T, typename U, typename... Ts>
struct IndexOfType<T, U, Ts...> {
    static constexpr std::size_t Value = 1 + IndexOfType<T, Ts...>::Value;
};

template <typename T, typename... Ts>
constexpr auto IndexOfTypeV = IndexOfType<T, Ts...>::Value;

template <typename MaybeVariant>
static constexpr auto IsVariantV =
    IsInstanceOfV<Variant, typename RemoveCVRef<MaybeVariant>::Type>;

template <typename V>
struct IsVariant {
    static constexpr auto Value = IsVariantV<V>;
};
}  // namespace detail

struct BadVariantAccess : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

struct Monostate {};

template <class... Ts>
class Variant {
   private:
    static_assert(sizeof...(Ts) > 0, "Variant must have at least one type");
    static_assert(detail::AllUniqueV<Ts...>,
                  "All element types must be unique");

    static constexpr auto StorageSize = std::max({sizeof(Ts)...});
    static constexpr auto StorageAlign = std::max({alignof(Ts)...});
    static constexpr auto TypeCount = sizeof...(Ts);

    enum struct Byte : std::uint8_t {};
    using StorageIndexType = typename detail::StorageIndex<TypeCount>::Type;

    static constexpr auto InvalidIndex = static_cast<StorageIndexType>(-1);

    template <std::size_t Index>
    using TypeAtIndex =
        typename std::tuple_element<Index, std::tuple<Ts...>>::type;

   private:
    template <std::size_t Index>
    TypeAtIndex<Index> &Get() & {
        // Note: No type_index_ check here. The caller (Visit) is responsible.
        return *reinterpret_cast<TypeAtIndex<Index> *>(&data_);
    }

    template <std::size_t Index>
    const TypeAtIndex<Index> &Get() const & {
        return *reinterpret_cast<const TypeAtIndex<Index> *>(&data_);
    }

    template <std::size_t Index>
    TypeAtIndex<Index> &&Get() && {
        return std::move(*reinterpret_cast<TypeAtIndex<Index> *>(&data_));
    }

    template <std::size_t Index>
    const TypeAtIndex<Index> &&Get() const && {
        return std::move(*reinterpret_cast<const TypeAtIndex<Index> *>(&data_));
    }

    template <typename T>
    void EnsureHoldsAlternative() const {
        if (type_index_ != detail::IndexOfTypeV<T, Ts...>) {
            throw BadVariantAccess("Unexpected alternative active");
        }
    }

    template <std::size_t I>
    void EnsureHoldsAlternative() const {
        if (type_index_ == I) {
            throw BadVariantAccess("Unexpected alternative active");
        }
    }

   public:
    template <typename T>
    TypeAtIndex<detail::IndexOfTypeV<T, Ts...>> &Get() & {
        EnsureHoldsAlternative<T>();
        return Get<detail::IndexOfTypeV<T, Ts...>>();
    }

    template <typename T>
    const TypeAtIndex<detail::IndexOfTypeV<T, Ts...>> &Get() const & {
        EnsureHoldsAlternative<T>();
        return Get<detail::IndexOfTypeV<T, Ts...>>();
    }

    template <typename T>
    TypeAtIndex<detail::IndexOfTypeV<T, Ts...>> &&Get() && {
        EnsureHoldsAlternative<T>();
        return static_cast<Variant &&>(*this)
            .Get<detail::IndexOfTypeV<T, Ts...>>();
    }

    template <typename T>
    const TypeAtIndex<detail::IndexOfTypeV<T, Ts...>> &&Get() && {
        EnsureHoldsAlternative<T>();
        return static_cast<const Variant &&>(*this)
            .Get<detail::IndexOfTypeV<T, Ts...>>();
    }

   private:
    template <class Fn, class Self, class ReturnType, std::size_t... Indexes>
    constexpr static void DummySameReturnTypes(
        std::index_sequence<Indexes...>) {
        static_assert(
            detail::AllTheSameV<
                ReturnType,
                decltype(std::declval<Fn>()(
                    std::declval<Self>().template Get<Indexes>()))...>,
            "All return types must be the same");
    }

    template <class ReturnType, class Fn, class Self>
    struct VisitImplStruct {
        static decltype(auto) DoVisit(Fn &&fn, Self &&self) {
            using VisitFuncPtr = ReturnType (*)(Fn &&, Self &&);
            VisitFuncPtr visit_table[] = {+[](Fn &&f, Self &&s) -> ReturnType {
                constexpr std::size_t I = TypeIndexV<Ts, Ts...>;
                return std::forward<Fn>(f)(
                    std::forward<Self>(s).template Get<I>());
            }...};
            return visit_table[self.type_index_](std::forward<Fn>(fn),
                                                 std::forward<Self>(self));
        }
    };

    template <class Fn, class Self>
    struct VisitImplStruct<void, Fn, Self> {
        static decltype(auto) DoVisit(Fn &&fn, Self &&self) {
            using VisitFuncPtr = void (*)(Fn &&, Self &&);
            VisitFuncPtr visit_table[] = {
                // The lambda gets its own index `I` via a template parameter.
                +[](Fn &&f, Self &&s) {
                    constexpr std::size_t I = TypeIndexV<Ts, Ts...>;
                    std::forward<Fn>(f)(
                        std::forward<Self>(s).template Get<I>());
                }...};
            visit_table[self.type_index_](std::forward<Fn>(fn),
                                          std::forward<Self>(self));
        }
    };

    template <typename Fn, class Self>
    static decltype(auto) DoVisitImpl(Fn &&fn, Self &&self) {
        using ReturnType = decltype(std::forward<Fn>(fn)(
            std::forward<Self>(self).template Get<0>()));
        DummySameReturnTypes<Fn, Self, ReturnType>(
            std::make_index_sequence<TypeCount>());

        return VisitImplStruct<ReturnType, Fn, Self>::DoVisit(
            std::forward<Fn>(fn), std::forward<Self>(self));
    }

    template <typename T, typename... Pack>
    struct TypeIndex;

    template <typename T, typename... Rest>
    struct TypeIndex<T, T, Rest...> : std::integral_constant<std::size_t, 0> {};

    template <typename T, typename U, typename... Rest>
    struct TypeIndex<T, U, Rest...>
        : std::integral_constant<std::size_t,
                                 1 + TypeIndex<T, Rest...>::value> {};

    template <class T, class... Pack>
    static constexpr auto TypeIndexV = TypeIndex<T, Pack...>::value;

   private:
    struct DestroyInPlace {
        template <class Type>
        void operator()(Type &value) const {
            value.~Type();
        }
    };

   private:
    void DoDestroy() {
        if (HasInvalidIndex()) {
            return;
        }

        Visit(DestroyInPlace{});
    }

    void throw_if_invalid_index() const {  // NOLINT
        if (HasInvalidIndex()) {
            throw BadVariantAccess("Invalid index");
        }
    }

   public:
    Variant() = default;

    ~Variant() { DoDestroy(); }

    template <typename T>
    explicit Variant(T &&value) {
        Emplace<typename detail::RemoveCVRef<T>::Type>(std::forward<T>(value));
    }

    template <typename T>
    explicit Variant(const T &value) {
        Emplace<typename detail::RemoveCVRef<T>::Type>(value);
    }

    Variant(const Variant &that) { Copy(that); }

    Variant &operator=(const Variant &that) {
        if (this == std::addressof(that)) {
            return *this;
        }
        Copy(that);
        return *this;
    }
    Variant(Variant &&that) noexcept { Take(std::move(that)); }

    Variant &operator=(Variant &&that) noexcept {
        if (this == std::addressof(that)) {
            return *this;
        }
        Take(std::move(that));
        return *this;
    }

   private:
    struct CopyVisitor {
        Variant &variant;
        template <typename Type>
        void operator()(const Type &value) noexcept {
            variant.Emplace<typename detail::RemoveCVRef<Type>::Type>(value);
        }
    };

   public:
    void Copy(const Variant &that) {
        Reset();
        that.Visit(CopyVisitor{*this});
    }

   private:
    struct MoveVisitor {
        Variant &variant;
        template <typename Type>
        void operator()(Type &&value) noexcept {
            variant.Emplace<typename detail::RemoveCVRef<Type>::Type>(
                std::move(value));
        }
    };

   public:
    void Take(Variant &&that) noexcept {
        Reset();
        that.Visit(MoveVisitor{*this});
        that.type_index_ = InvalidIndex;
    }

    void Reset() { DoDestroy(); }

    constexpr bool HasInvalidIndex() const {
        return type_index_ == InvalidIndex;
    }

    template <typename T>
    constexpr bool HoldsAlternative() const {
        static_assert(detail::IsOneOfV<T, Ts...>,
                      "Must be one of the types belonging to the variant");
        return type_index_ == TypeIndexV<T, Ts...>;
    }

    template <typename T, typename... Args>
    void Emplace(Args &&...args) {
        using Type = T;
        static_assert(detail::IsOneOfV<Type, Ts...>,
                      "Type to be constructed must be in the variant");
        static_assert(std::is_constructible<Type, Args...>::value,
                      "Type must be constructible from the provided args");

        DoDestroy();
        new (reinterpret_cast<Type *>(data_)) Type(std::forward<Args>(args)...);
        type_index_ = static_cast<StorageIndexType>(TypeIndexV<Type, Ts...>);
    }

    template <typename Fn>
    decltype(auto) Visit(Fn &&fn) & {
        throw_if_invalid_index();
        return DoVisitImpl(std::forward<Fn>(fn), *this);
    }

    template <typename Fn>
    decltype(auto) Visit(Fn &&fn) const & {
        throw_if_invalid_index();
        return DoVisitImpl(std::forward<Fn>(fn), *this);
    }

    template <typename Fn>
    decltype(auto) Visit(Fn &&fn) && {
        throw_if_invalid_index();
        return DoVisitImpl(std::forward<Fn>(fn), std::move(*this));
    }

    template <typename Fn>
    decltype(auto) Visit(Fn &&fn) const && {
        throw_if_invalid_index();
        return DoVisitImpl(std::forward<Fn>(fn), std::move(*this));
    }

    constexpr std::size_t Index() const noexcept {
        if (HasInvalidIndex()) {
            return static_cast<std::size_t>(-1);
        }
        return type_index_;
    }

   private:
    alignas(StorageAlign) Byte data_[StorageSize];
    StorageIndexType type_index_ = InvalidIndex;
};

template <typename Visitor, typename V>
decltype(auto) Visit(Visitor &&visitor, V &&v) {
    static_assert(detail::IsVariantV<V>, "Must be a variant");
    return std::forward<V>(v).Visit(std::forward<Visitor>(visitor));
}

template <typename T, typename V>
decltype(auto) HoldsAlternative(const V &v) {
    static_assert(detail::IsVariantV<V>, "Must be a variant");
    return v.template HoldsAlternative<T>();
}

template <typename T, typename V>
decltype(auto) Get(V &&v) {
    static_assert(detail::IsVariantV<V>, "Must be a variant");
    return std::forward<V>(v).template Get<T>();
}

}  // namespace variant

#endif  // VARIANT_VARIANT_HPP
