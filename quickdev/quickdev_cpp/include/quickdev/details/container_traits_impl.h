// =============================================================================================================================================
template
<
    unsigned int __Index__,
    class __Container
>
static typename container::element<__Index__, __Container>::type
at( __Container const & container )
{
    return std::get<__Index__>( container.values_ );
}

// =============================================================================================================================================
template<class __Container>
static typename container::elem_traits<__Container>::_Front
front( __Container const & container )
{
    return container::at<__Container::front_>( container );
}

// =============================================================================================================================================
template<class __Container>
static typename container::elem_traits<__Container>::_Back
back( __Container const & container )
{
    return container::at<__Container::back_>( container );
}

// #############################################################################################################################################

// =============================================================================================================================================
template
<
    unsigned int __CurrentIndex__,
    class... __Types
>
static typename std::enable_if<(__CurrentIndex__ == sizeof...( __Types ) - 1 ), void>::type
print_rec( const quickdev::Container<__Types...> & container )
{
    std::cout << container::at<__CurrentIndex__>( container ) << std::endl;
}

// =============================================================================================================================================
template
<
    unsigned int __CurrentIndex__,
    class... __Types
>
static typename std::enable_if<(__CurrentIndex__ < sizeof...( __Types ) - 1 ), void>::type
print_rec( const quickdev::Container<__Types...> & container )
{
    std::cout << container::at<__CurrentIndex__>( container ) << std::endl;
    container::print_rec<__CurrentIndex__ + 1>( container );
}

// =============================================================================================================================================
template<class... __Types>
static typename std::enable_if<(sizeof...(__Types) > 0), void>::type
print( const quickdev::Container<__Types...> & container )
{
    container::print_rec<0>( container );
}

// =============================================================================================================================================
template<class... __Types>
static typename std::enable_if<(sizeof...(__Types) == 0), void>::type
print( const quickdev::Container<__Types...> & container )
{
    std::cout << "<empty container>" << std::endl;
}

// #############################################################################################################################################

// =============================================================================================================================================
template
<
    unsigned int __EndIndex__,
    unsigned int __StartIndex__
>
template
<
    class __Container,
    class... __Types
>
typename container::subtype_rec<__EndIndex__, __StartIndex__, __Container, __Types...>::type
subset_rec<__EndIndex__, __StartIndex__>::exec( __Container const & container, __Types&&... types )
{
    return container::subset_rec<__EndIndex__ - 1, __StartIndex__>::exec( container, container::at<__EndIndex__>( container ), types... );
}

// =============================================================================================================================================
template<unsigned int __StartIndex__>
template
<
    class __Container,
    class... __Types
>
quickdev::Container<typename container::element<__StartIndex__, __Container>::type, __Types...>
subset_rec<__StartIndex__, __StartIndex__>::exec( __Container const & container, __Types&&... types )
{
    return quickdev::make_container( container::at<__StartIndex__>( container ), types... );
}

// =============================================================================================================================================
template
<
    unsigned int __StartIndex__,
    unsigned int __NumItems__,
    class __Container
>
static typename container::subtype<__StartIndex__, __NumItems__, __Container>::type
subset( __Container const & container )
{
    return container::subset_rec<__StartIndex__ + __NumItems__ - 1, __StartIndex__>::exec( container );
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Container>
static typename container::subtype<1, __Container::back_, __Container>::type
pop_front( __Container const & container )
{
    return container::subset<1, __Container::back_>( container );
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Container>
static typename container::subtype<0, __Container::back_, __Container>::type
pop_back( __Container const & container )
{
    return container::subset<0, __Container::back_>( container );
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Container>
static typename container::traits<__Container>::_Head
head( __Container const & container )
{
    return container::subset<__Container::front_, 1>( container );
}

// #############################################################################################################################################

// =============================================================================================================================================
template<class __Container>
static typename container::traits<__Container>::_Tail
tail( __Container const & container )
{
    return container::subset<1, __Container::back_>( container );
}

// #############################################################################################################################################

// =============================================================================================================================================
template<unsigned int __EndSize__>
template
<
    class __Container,
    class... __Types
>
typename container::push_back_type<__Container, typename container::elem_traits<quickdev::Container<__Types...> >::_Back>::type
push_back_rec<__EndSize__>::exec( __Container const & container, __Types&&... types )
{
    // push the items of container in order onto types...
    return container::push_back_rec<__EndSize__ - 1>::exec( container, container::at<__EndSize__ - 2>( container ), std::forward<__Types>( types )... );
}

// =============================================================================================================================================
template
<
    class __Container,
    class... __Types
>
quickdev::Container<__Types...>
push_back_rec<1>::exec( __Container const & container, __Types&&... types )
{
    return quickdev::make_container( std::forward<__Types>( types )... );
}

// =============================================================================================================================================
template
<
    class __Container,
    class __Type
>
static typename container::push_back_type<__Container, __Type>::type
push_back( __Container const & container, __Type const & type )
{
    return container::push_back_rec<__Container::size_ + 1>::exec( container, type );
}

// #############################################################################################################################################

// =============================================================================================================================================
template<unsigned int __EndSize__>
template
<
    class __Container,
    class __Type,
    class... __Types
>
typename container::push_front_type<__Container, __Type>::type
push_front_rec<__EndSize__>::exec( __Container const & container, __Type const & type, __Types&&... types )
{
    // push the items of container in order onto types ending with type
    return container::push_front_rec<__EndSize__ - 1>::exec( container, type, container::at<__EndSize__ - 2>( container ), std::forward<__Types>( types )... );
}

// =============================================================================================================================================
template
<
    class __Container,
    class __Type,
    class... __Types
>
quickdev::Container<__Type, __Types...>
push_front_rec<1>::exec( __Container const & container, __Type const & type, __Types&&... types )
{
    return quickdev::make_container( type, std::forward<__Types>( types )... );
}

// =============================================================================================================================================
template
<
    class __Container,
    class __Type
>
static typename container::push_front_type<__Container, __Type>::type
push_front( __Container const & container, __Type const & type )
{
    return container::push_front_rec<__Container::size_ + 1>::exec( container, type );
}

// #############################################################################################################################################

// =============================================================================================================================================
template<unsigned int __Index__>
template
<
    class __Container1,
    class __Container2,
    class... __Types
>
typename std::enable_if<(__Index__ > __Container1::size_), typename container::combine_type<__Container1, __Container2>::type>::type
combine_rec<__Index__>::exec( __Container1 const & container1, __Container2 const & container2, __Types&&... types )
{
    return container::combine_rec<__Index__ - 1>::exec( container1, container2, container::at<__Index__ - __Container1::size_ - 1>( container2 ), std::forward<__Types>( types )... );
}

// =============================================================================================================================================
template<unsigned int __Index__>
template
<
    class __Container1,
    class __Container2,
    class... __Types
>
typename std::enable_if<(__Index__ <= __Container1::size_), typename container::combine_type<__Container1, __Container2>::type>::type
combine_rec<__Index__>::exec( __Container1 const & container1, __Container2 const & container2, __Types&&... types )
{
    return container::combine_rec<__Index__ - 1>::exec( container1, container2, container::at<__Index__ - 1>( container1 ), std::forward<__Types>( types )... );
}

// =============================================================================================================================================
template
<
    class __Container1,
    class __Container2,
    class... __Types
>
typename container::combine_type<__Container1, __Container2>::type
combine_rec<0>::exec( __Container1 const & container1, __Container2 const & container2, __Types&&... types )
{
    return quickdev::make_container( std::forward<__Types>( types )... );
}

// =============================================================================================================================================
template
<
    class __Container1,
    class __Container2
>
static typename container::combine_type<__Container1, __Container2>::type
combine( __Container1 const & container1, __Container2 const & container2 )
{
    return container::combine_rec<__Container1::size_+ __Container2::size_>::exec( container1, container2 );
};
