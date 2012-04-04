// =============================================================================================================================================
template<unsigned int __Index__, class __Type, class... __Types>
static typename std::enable_if<(__Index__ == 0), typename variadic::element<__Index__, __Type, __Types...>::type>::type &
at_rec( __Type & type, __Types&&... types )
{
    return type;
}

// =============================================================================================================================================
template<unsigned int __Index__, class __Type, class... __Types>
static typename std::enable_if<(__Index__ > 0), typename variadic::element<__Index__, __Type, __Types...>::type>::type &
at_rec( __Type & type, __Types&&... types )
{
    return variadic::at_rec<__Index__ - 1>( std::forward<__Types>( types )... );
}

// =============================================================================================================================================
template<unsigned int __Index__, class... __Types>
static typename variadic::element<__Index__, __Types...>::type &
at( __Types&&... types )
{
    return variadic::at_rec<__Index__>( std::forward<__Types>( types )... );
}
