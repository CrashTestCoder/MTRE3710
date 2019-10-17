#include <type_traits>
namespace std {
    template< class... T >
    struct common_reference;

    template<typename... T>
    using common_reference_t = typename common_reference<T...>::type;

    template<> struct common_reference<>;
    template<typename T> struct common_reference<T> {
        using type = T;
    };

    template<typename T1, typename T2>
        requires ((requires { typename std::common_type_t<T1,T2>; }) &&
                  std::is_lvalue_reference_v<T1> &&
                  std::is_lvalue_reference_v<T2>)
    struct common_reference<T1,T2> {
        using type = typename std::add_lvalue_reference<std::common_type<T1,T2>>;
    };

    template<typename T1, typename T2>
        requires requires { typename std::common_type_t<T1,T2>; } && 
                (std::is_rvalue_reference_v<T1> &&
                 std::is_rvalue_reference_v<T2> &&
                 is_convertible_v<T1, std::add_rvalue_reference<common_type<T1,T2>>> &&
                 is_convertible_v<T2, std::add_rvalue_reference<common_type<T1,T2>>>)
    struct common_reference<T1,T2> {
        using type = typename std::add_rvalue_reference<common_type<T1,T2>>;
    };

    template<typename T1, typename T2>      // &&      const&
        requires std::is_lvalue_reference_v<T1> &&
                 std::is_rvalue_reference_v<T2> &&
                 requires { typename std::common_type_t<T1,std::decay<T2> const&>; } &&
                 std::is_convertible_v<T2,std::common_type_t<T1,std::decay<T2> const&>>
    struct common_reference<T1,T2> {
        using type = typename std::common_type_t<T1,std::decay<T2> const&>;
    };

    template<typename T1, typename T2, typename T3, typename... Ts>
    struct common_reference<T1,T2,T3,Ts...> {
        using type = std::common_reference_t<std::common_reference_t<T1, T2>, T3, Ts...>;
    };

    // language-related     concepts
    template<class T, class U>  
        concept __SameImpl = is_same_v<T, U>;  // exposition only 
    template<class T, class U> 
        concept same_as = __SameImpl<T, U> && __SameImpl<U, T>; 

    //  concept derived_from
    template<class Derived, class Base>
        concept derived_from =
            is_base_of_v<Base, Derived> &&
            is_convertible_v<const volatile Derived*, const volatile Base*>;

    //  concept convertible_to
    template<class From, class To>
        concept convertible_to =
            is_convertible_v<From, To> &&
            requires(From (&f)()) {
                static_cast<To>(f());
            };

    //  concept common_reference_with
    template<class T, class U>
        concept common_reference_with = 
            same_as<common_reference_t<T, U>, common_reference_t<U, T>> &&
            convertible_to<T, common_reference_t<T, U>> &&
            convertible_to<U, common_reference_t<T, U>>;

    //  concept common_with
    template<class T, class U>
        concept common_with =
            same_as<common_type_t<T, U>, common_type_t<U, T>> &&
            requires {
                static_cast<common_type_t<T, U>>(declval<T>());
                static_cast<common_type_t<T, U>>(declval<U>());
            } &&
            common_reference_with<
                add_lvalue_reference_t<const T>,
                add_lvalue_reference_t<const U>> &&
            common_reference_with<
                add_lvalue_reference_t<common_type_t<T, U>>,
                common_reference_t<
                    add_lvalue_reference_t<const T>,
                    add_lvalue_reference_t<const U>>>;

    // arithmetic   concepts
    template<class T>
        concept integral = is_integral_v<T>;

    template<class T>
        concept signed_integral = integral<T> && is_signed_v<T>;

    template<class T>
        concept unsigned_integral = integral<T> && !is_signed_v<T>;

    template<class T>
        concept floating_point = is_floating_point_v<T>;

    //  concept assignable_from
    template<class LHS, class RHS>
        concept assignable_from =
            is_lvalue_reference_v<LHS> &&
            common_reference_with<
                const remove_reference_t<LHS>&,
                const remove_reference_t<RHS>&> &&
            requires(LHS lhs, RHS&& rhs) {
                { lhs = std::forward<RHS>(rhs) } -> same_as<LHS>; 
            };

    //  concept swappable
    namespace ranges {
        inline namespace {
            template<typename T1, typename T2>
            inline constexpr void swap(T1&& t1, T2&& t2)
            {
                std::swap(std::forward<T1>(t1), std::forward<T2>(t2));
            }

        }
    }
    template<class T>
        concept swappable = requires(T& a, T& b) { ranges::swap(a, b); };

    template<class T, class U>
        concept swappable_with =
            std::common_reference_with<const remove_reference_t<T>&, const remove_reference_t<U>&> &&
            requires(T&& t, U&& u) {
                ranges::swap(std::forward<T>(t), std::forward<T>(t));
                ranges::swap(std::forward<U>(u), std::forward<U>(u));
                ranges::swap(std::forward<T>(t), std::forward<U>(u));
                ranges::swap(std::forward<U>(u), std::forward<T>(t));
            }; 

    //  concept destructible
    template<class T>
        concept destructible = std::is_nothrow_destructible_v<T>;

    //  concept constructible_from
    template<class T, class... Args>
        concept constructible_from = destructible<T> && std::is_constructible_v<T, Args...>;

    //  concept default_constructible
    template<class T>
        concept default_constructible = constructible_from<T>;

    //  concept move_constructible
    template<class T>
        concept move_constructible = constructible_from<T, T> && std::convertible_to<T, T>;

    //  concept copy_constructible
    template<class T>
        concept copy_constructible =
            move_constructible<T> &&
            constructible_from<T, T&> && std::convertible_to<T&, T> &&
            constructible_from<T, const T&> && convertible_to<const T&, T> &&
            constructible_from<T, const T> && convertible_to<const T, T>;

    template<class T>
        concept movable = std::is_object_v<T> && move_constructible<T> &&
                          assignable_from<T&, T> && swappable<T>;

    template<class T>
        concept copyable = copy_constructible<T> && movable<T> && std::assignable_from<T&, const T&>;

    // comparison   concepts
    //  concept boolean
    template<class B>
        concept boolean =
            movable<std::remove_cvref_t<B>> &&
            requires(const remove_reference_t<B>& b1,
                     const remove_reference_t<B>& b2, const bool a) {
                { b1 } -> convertible_to<bool>;
                { !b1 } -> convertible_to<bool>;
                { b1 && b2 } -> same_as<bool>;
                { b1 &&  a } -> same_as<bool>;
                {  a && b2 } -> same_as<bool>;
                { b1 || b2 } -> same_as<bool>;
                { b1 ||  a } -> same_as<bool>;
                {  a || b2 } -> same_as<bool>;
                { b1 == b2 } -> convertible_to<bool>;
                { b1 ==  a } -> convertible_to<bool>;
                {  a == b2 } -> convertible_to<bool>;
                { b1 != b2 } -> convertible_to<bool>;
                { b1 !=  a } -> convertible_to<bool>;
                {  a != b2 } -> convertible_to<bool>;
            };

    //  concept equality_comparable
    template<class T, class U>
        concept __WeaklyEqualityComparableWith = // exposition only
            requires(const remove_reference_t<T>& t,
                     const remove_reference_t<U>& u) {
                { t == u } -> boolean;
                { t != u } -> boolean;
                { u == t } -> boolean;
                { u != t } -> boolean;
            };
 
    template<class T>
        concept equality_comparable = __WeaklyEqualityComparableWith<T, T>;

    template<class T, class U>
        concept equality_comparable_with =
            equality_comparable<T> && equality_comparable<U> &&
            std::common_reference_with<const remove_reference_t<T>&, const remove_reference_t<U>&> &&
            equality_comparable<
                common_reference_t<
                    const remove_reference_t<T>&,
                    const remove_reference_t<U>&>> &&
            __WeaklyEqualityComparableWith<T, U>;

    //  concept totally_ordered
    template<class T>
        concept totally_ordered =
            equality_comparable<T> &&
            requires(const remove_reference_t<T>& a,
                     const remove_reference_t<T>& b) {
                { a <  b } -> boolean;
                { a >  b } -> boolean;
                { a <= b } -> boolean;
                { a >= b } -> boolean;
            };

    template<class T, class U>
        concept totally_ordered_with =
            totally_ordered<T> && totally_ordered<U> &&
            std::common_reference_with<const remove_reference_t<T>&, const remove_reference_t<U>&> &&
            totally_ordered<
                common_reference_t<
                    const remove_reference_t<T>&,
                    const remove_reference_t<U>&>> &&
            equality_comparable_with<T, U> &&
            requires(const remove_reference_t<T>& t,
                     const remove_reference_t<U>& u) {
                { t <  u } -> boolean;
                { t >  u } -> boolean;
                { t <= u } -> boolean;
                { t >= u } -> boolean;
                { u <  t } -> boolean;
                { u >  t } -> boolean;
                { u <= t } -> boolean;
                { u >= t } -> boolean;
            };

    // object   concepts
    template<class T>
        concept semiregular = copyable<T> && default_constructible<T>;

    template<class T>
        concept regular = semiregular<T> && equality_comparable<T>;

    // callable     concepts
    //  concept invocable
    template<class F, class... Args>
        concept invocable = requires(F&& f, Args&&... args) {
                invoke(std::forward<F>(f), std::forward<Args>(args)...);
                  // not required to be equality-preserving
            };

    //  concept regular_invocable
    template<class F, class... Args>
        concept regular_invocable = invocable<F, Args...>;

    //  concept predicate
    template<class F, class... Args>
        concept predicate = regular_invocable<F, Args...> && boolean<std::invoke_result_t<F, Args...>>;

    //  concept relation
    template<class R, class T, class U>
        concept relation =
            predicate<R, T, T> && predicate<R, U, U> &&
            predicate<R, T, U> && predicate<R, U, T>;

    //  concept strict_weak_order
    template<class R, class T, class U>     
        concept strict_weak_order = relation<R, T, U>;

    template<typename T>
        concept weakly_incrementable = requires (T t) {
            { t++ } -> same_as<T&>;
            ++t;
        } && copyable<T> && default_constructible<T>;
    
    template<typename T> 
        concept input_or_output_iterator = 
            requires(T t) {
                { *t }
            } &&
            weakly_incrementable<T>;

    template < class T >
        concept incrementable = weakly_incrementable<T> &&
                                equality_comparable<T>;
}