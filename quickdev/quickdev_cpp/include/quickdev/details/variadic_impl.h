// =============================================================================================================================================
template
<
    unsigned int __Index__,
    class __Type,
    class... __Types,
    typename std::enable_if<(__Index__ == 0), int>::type
>
static typename variadic::element<__Index__, __Type, __Types...>::type &&
at_rec( __Type && type, __Types&&... types )
{
    return type;
}

// =============================================================================================================================================
template
<
    unsigned int __Index__,
    class __Type,
    class... __Types,
    typename std::enable_if<(__Index__ > 0), int>::type
>
static typename variadic::element<__Index__, __Type, __Types...>::type &&
at_rec( __Type && type, __Types&&... types )
{
    return variadic::at_rec<__Index__ - 1>( std::forward<__Types>( types )... );
}

// =============================================================================================================================================
template
<
    int __Index__,
    class... __Types,
    typename std::enable_if<(__Index__ >= 0), int>::type
>
static typename variadic::element<__Index__, __Types...>::type &&
at( __Types&&... types )
{
    return variadic::at_rec<__Index__>( std::forward<__Types>( types )... );
}

// =============================================================================================================================================
template
<
    int __Index__,
    class... __Types,
    typename std::enable_if<(__Index__ < 0), int>::type
>
static typename variadic::element<sizeof...(__Types)+__Index__, __Types...>::type &&
at( __Types&&... types )
{
    return variadic::at_rec<sizeof...(__Types)+__Index__>( std::forward<__Types>( types )... );
}
