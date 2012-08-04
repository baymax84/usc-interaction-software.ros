// =============================================================================================================================================
/*
template<class... __Types>
template
<
    class... __MTypes,
    typename std::enable_if<(sizeof...(__MTypes) > 0 && std::is_same<SimpleContainer<__Types...>, SimpleContainer<__MTypes...> >::value), int>::type
>
Container<__Types...>::Container( __MTypes&&... values )
:
    values_( std::make_tuple( std::forward<__MTypes>( values )... ) )
{
    //
}
*/
template<class... __Types>
Container<__Types...>::Container()
{
    //
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class... __Types>
static Container<__Types...>
make_container( __Types&&... types )
{
    return quickdev::Container<__Types...>( types... );
}

// =============================================================================================================================================
template<class... __Types>
static Container<__Types...>
make_simple_container( __Types&&... types )
{
    return quickdev::SimpleContainer<__Types...>();
}
