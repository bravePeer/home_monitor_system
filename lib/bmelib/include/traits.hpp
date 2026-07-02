#pragma once

#if defined(__GNUC__) && !defined(__clang__) && defined(__AVR__)
    #define USE_LOCAL_TRAITS
#else
    #include <type_traits>
#endif


namespace traits
{
#if defined(USE_LOCAL_TRAITS)
    
    // enable_if
    template<bool B, typename T = void>
    struct enable_if 
    { };
    
    template<typename T>
    struct enable_if<true, T> 
    {
        using type = T;
    };

    // is_same
    template<typename T, typename U>
    struct is_same 
    {
        static constexpr bool value = false;
    };

    template<typename T>
    struct is_same<T, T> 
    {
        static constexpr bool value = true;
    };

#else

    // Alias to is_same from std
    template<typename T, typename U>
    using is_same = std::is_same<T, U>;

    // Alias to enable_if from std
    template<bool B, typename T = void>
    using enable_if = std::enable_if<B, T>;

#endif

}
