#ifndef QUICKDEVCPP_QUICKDEV_VECTOR_H_
#define QUICKDEVCPP_QUICKDEV_VECTOR_H_

#include <quickdev/macros.h>
#include <quickdev/reference.h>
#include <array>
#include <ostream>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

namespace vector_base
{
namespace operations
{
    struct Add{};
    struct Sub{};
    struct Mul{};
    struct Div{};
}
}

template<class __Current, class __Derived>
struct passthrough_derived
{
    typedef __Derived type;
};

template<class __Current>
struct passthrough_derived<__Current, void>
{
    typedef __Current type;
};

/*template<class __Storage, class __Derived>
class VectorBase;

template<template<size_t, typename> class __Storage, class __Derived, size_t __Dim__, class __Data>
class VectorBase<__Storage<__Dim__, __Data>, __Derived > : public std::array<__Data, __Dim__>
{
public:
    typedef std::array<__Data, __Dim__> _Parent;
    typedef __Storage<__Dim__, __Data> _Storage;

public:
    VectorBase()
    {
        //set( __Data() );
    }

    VectorBase( __Data&& value )
    {
        set( std::forward<__Data>( value ) );
    }

    template
    <
        class... __Args,
        typename std::enable_if<(sizeof...(__Args) > 1), int>::type = 0
    >
    VectorBase( __Args&&... args )
    {
        init( std::forward<__Args>( args )... );
    }

    void set( __Data&& value )
    {
        std::fill( this->begin(), this->end(), std::forward<__Data>( value ) );
    }

    void init( std::initializer_list<__Data> const & args )
    {
        std::copy( args.begin(), args.end(), this->begin() );
    }

    template<class... __Args>
    void init( __Args&&... args )
    {
        init( { std::forward<__Args>( args )... } );
    }

    __Derived & getDerived()
    {
        return *(__Derived*)this;
    }

    __Derived getDerived() const
    {
        return *(__Derived*)this;
    }

    __Derived copy() const
    {
        return getDerived();
    }

    __Derived & copyFrom( __Derived const & other )
    {
        std::copy( other.begin(), other.end(), this->begin() );

        return getDerived();
    }

    __Derived & operator=( __Derived const & other )
    {
        return copyFrom( other );
    }

    // entry point for operators; operation<>() handles vector and single-value-numeric specialization
    template<class __Value>
    __Derived & operator+=( __Value const & value ){ *this = operation<vector_base::operations::Add>( value ); return getDerived(); }

    template<class __Value>
    __Derived operator+( __Value const & value ) const { return operation<vector_base::operations::Add>( value ); }

    template<class __Value>
    __Derived & operator-=( __Value const & value ) { *this = operation<vector_base::operations::Sub>( value ); return getDerived(); }

    template<class __Value>
    __Derived operator-( __Value const & value ) const { return operation<vector_base::operations::Sub>( value ); }

    template<class __Value>
    __Derived & operator*=( __Value const & value ){ *this = operation<vector_base::operations::Mul>( value ); return getDerived(); }

    template<class __Value>
    __Derived operator*( __Value const & value ) const { return operation<vector_base::operations::Mul>( value ); }

    template<class __Value>
    __Derived & operator/=( __Value const & value ){ *this = operation<vector_base::operations::Div>( value ); return getDerived(); }

    template<class __Value>
    __Derived operator/( __Value const & value ) const { return operation<vector_base::operations::Div>( value ); }

protected:
    //! Perform an addition on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Add>::value), int>::type = 0
    >
    static inline __Data1 operation( __Data1 const & value1, __Data2 const & value2 ){ return value1 + value2; }

    //! Perform a subtraction on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Sub>::value), int>::type = 0
    >
    static inline __Data1 operation( __Data1 const & value1, __Data2 const & value2 ){ return value1 - value2; }

    //! Perform a multiplication on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Mul>::value), int>::type = 0
    >
    static inline __Data1 operation( __Data1 const & value1, __Data2 const & value2 ){ return value1 * value2; }

    //! Perform a division on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Div>::value), int>::type = 0
    >
    static inline __Data1 operation( __Data1 const & value1, __Data2 const & value2 ){ return value1 / value2; }

    //! Add two vectors (per-element)
    template<class __Operation>
    __Derived operation( __Derived const & other ) const
    {
        __Derived result;
        auto to_elem = result.begin();
        auto other_elem = other.cbegin();

        for( auto from_elem = this->cbegin(); from_elem != this->cend(); ++from_elem, ++other_elem, ++to_elem )
        {
            *to_elem = operation<__Operation>( *from_elem, *other_elem );
        }

        return result;
    }

    //! Add another numeric value to a vector (per-element)
    template
    <
        class __Operation,
        class __Value,
        typename std::enable_if<(std::is_arithmetic<__Value>::value), int>::type = 0
    >
    __Derived operation( __Value const & value ) const
    {
        __Derived result;
        auto to_elem = result.begin();
        for( auto from_elem = this->cbegin(); from_elem != this->cend(); ++from_elem, ++to_elem )
        {
            *to_elem = operation<__Operation>( *from_elem, value );
        }

        return result;
    }
};*/

/*
template<class __Storage, class __Derived>
class VectorBaseHelper : public VectorBase<__Storage, typename get_derived<__Storage, __Derived>::type>
{
    template<class... __Args>
    VectorBaseHelper( __Args&&... args )
    :
        VectorBase<__Storage, typename get_derived<__Storage, __Derived>::type>( std::forward<__Args>( args )... )
    {
        //
    }
};

template<size_t __Dim__, class __Data>
struct VectorStorage
{
    static size_t const _DIM_ = __Dim__;
    typedef __Data _Data;
};*/

template<size_t __Dim__, class __Data>
class VectorBase : public std::array<__Data, __Dim__>
{
public:
    typedef VectorBase<__Dim__, __Data> _VectorBase;

    VectorBase()
    {
        set( __Data() );
    }

    VectorBase( __Data&& value )
    {
        set( std::forward<__Data>( value ) );
    }

    /*template
    <
        class __Other,
        typename std::enable_if<!(std::is_same<__Other, __Data>::value), int>::type = 0
    >
    VectorBase( __Other const & other )
    {
        copyFrom( other );
    }*/

    template
    <
        class... __Args,
        typename std::enable_if<(sizeof...(__Args) > 1), int>::type = 0
    >
    VectorBase( __Args&&... args )
    {
        init( std::forward<__Args>( args )... );
    }

    void set( __Data&& value )
    {
        std::fill( this->begin(), this->end(), std::forward<__Data>( value ) );
    }

    void init( std::initializer_list<__Data> const & args )
    {
        std::copy( args.begin(), args.end(), this->begin() );
    }

    template<class... __Args>
    void init( __Args&&... args )
    {
        init( { std::forward<__Args>( args )... } );
    }

    _VectorBase copy() const
    {
        return *this;
    }

    template<class __Other>
    _VectorBase & copyFrom( __Other const & other )
    {
        std::copy( other.begin(), other.end(), this->begin() );

        return *this;
    }

    friend std::ostream & operator<<( std::ostream & stream, _VectorBase const & vector )
    {
        stream << "[ ";
        for( auto elem = vector.cbegin(); elem != vector.cend(); ++elem )
        {
            stream << *elem;
            if( elem != vector.cend() - 1 ) stream << ",";// << std::endl;
        }
        stream << " ]";

        return stream;
    }

    protected:
    //! Perform an addition on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Add>::value), int>::type = 0
    >
    static inline __Data1 elemOperation( __Data1 const & value1, __Data2 const & value2 ){ return value1 + value2; }

    //! Perform a subtraction on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Sub>::value), int>::type = 0
    >
    static inline __Data1 elemOperation( __Data1 const & value1, __Data2 const & value2 ){ return value1 - value2; }

    //! Perform a multiplication on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Mul>::value), int>::type = 0
    >
    static inline __Data1 elemOperation( __Data1 const & value1, __Data2 const & value2 ){ return value1 * value2; }

    //! Perform a division on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Div>::value), int>::type = 0
    >
    static inline __Data1 elemOperation( __Data1 const & value1, __Data2 const & value2 ){ return value1 / value2; }

    /*// entry point for operators; operation<>() handles vector and single-value-numeric specialization
    template<class __Value>
    _VectorBase & operator+=( __Value const & value ){ return operation<vector_base::operations::Add>( value ); }

    template<class __Value>
    _VectorBase operator+( __Value const & value ) const { return operation<vector_base::operations::Add>( *this, value ); }

    template<class __Value>
    _VectorBase & operator-=( __Value const & value ) { return operation<vector_base::operations::Sub>( value ); }

    template<class __Value>
    _VectorBase operator-( __Value const & value ) const { return operation<vector_base::operations::Sub>( *this, value ); }

    template<class __Value>
    _VectorBase & operator*=( __Value const & value ){ return operation<vector_base::operations::Mul>( value ); }

    template<class __Value>
    _VectorBase operator*( __Value const & value ) const { return operation<vector_base::operations::Mul>( *this, value ); }

    template<class __Value>
    _VectorBase & operator/=( __Value const & value ){ return operation<vector_base::operations::Div>( value ); }

    template<class __Value>
    _VectorBase operator/( __Value const & value ) const { return operation<vector_base::operations::Div>( *this, value ); }

protected:
    //! Perform an addition on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Add>::value), int>::type = 0
    >
    static inline __Data1 operation( __Data1 const & value1, __Data2 const & value2 ){ return value1 + value2; }

    //! Perform a subtraction on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Sub>::value), int>::type = 0
    >
    static inline __Data1 operation( __Data1 const & value1, __Data2 const & value2 ){ return value1 - value2; }

    //! Perform a multiplication on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Mul>::value), int>::type = 0
    >
    static inline __Data1 operation( __Data1 const & value1, __Data2 const & value2 ){ return value1 * value2; }

    //! Perform a division on a single element
    template
    <
        class __Operation, class __Data1, class __Data2,
        typename std::enable_if<(std::is_same<__Operation, vector_base::operations::Div>::value), int>::type = 0
    >
    static __Data1 operation( __Data1 const & value1, __Data2 const & value2 ){ return value1 / value2; }

    //! Add two vectors (per-element)
    template<class __Operation>
    _VectorBase operation( _VectorBase const & vec1, _VectorBase const & vec2 ) const
    {
        _VectorBase result;
        auto to_elem = result.begin();
        auto other_elem = vec2.cbegin();

        for( auto from_elem = vec1.cbegin(); from_elem != vec1.cend(); ++from_elem, ++other_elem, ++to_elem )
        {
            *to_elem = operation<__Operation>( *from_elem, *other_elem );
        }

        return result;
    }

    //! Add two vectors (per-element)
    template<class __Operation>
    _VectorBase & operation( _VectorBase const & other )
    {
        auto other_elem = other.cbegin();

        for( auto from_elem = this->begin(); from_elem != this->end(); ++from_elem, ++other_elem )
        {
            *from_elem = operation<__Operation>( *from_elem, *other_elem );
        }

        return *this;
    }

    //! Add another numeric value to a vector (per-element)
    template
    <
        class __Operation,
        class __Value,
        typename std::enable_if<(std::is_arithmetic<__Value>::value), int>::type = 0
    >
    _VectorBase operation( _VectorBase const & vec, __Value const & value ) const
    {
        _VectorBase result;
        auto to_elem = result.begin();
        for( auto from_elem = vec.cbegin(); from_elem != vec.cend(); ++from_elem, ++to_elem )
        {
            *to_elem = operation<__Operation>( *from_elem, value );
        }

        return result;
    }

    //! Add another numeric value to a vector (per-element)
    template
    <
        class __Operation,
        class __Value,
        typename std::enable_if<(std::is_arithmetic<__Value>::value), int>::type = 0
    >
    _VectorBase & operation( __Value const & value )
    {
        for( auto from_elem = this->begin(); from_elem != this->end(); ++from_elem )
        {
            *from_elem = operation<__Operation>( *from_elem, value );
        }

        return *this;
    }*/
};

template<size_t __Dim__, class __Data>
class Vector : public VectorBase<__Dim__, __Data>
{
public:
    template<class... __Args>
    Vector( __Args&&... args )
    :
        VectorBase<__Dim__, __Data>( std::forward<__Args>( args )... )
    {
        //
    }
};

template<class __Data>
class Vector<2, __Data> : public VectorBase<2, __Data>
{
public:
    typedef Vector<2, __Data> _Vector;
    typedef VectorBase<2, __Data> _VectorBase;

    __Data & x;
    __Data & y;

    template<class... __Args>
    Vector( __Args&&... args  )
    :
        _VectorBase( std::forward<__Args>( args )... ),
        x( this->at( 0 ) ),
        y( this->at( 1 ) )
    {
        //
    }

    _Vector & operator=( _Vector const & other )
    {
        _VectorBase::copyFrom( other );

        x = other.x;
        y = other.y;

        return *this;
    }
};

template<class __Data>
class Vector<3, __Data> : public VectorBase<3, __Data>//, private Vector<2, Ref<typename remove_ref<__Data>::type> >
{
public:
    typedef Vector<3, __Data> _Vector;
    typedef VectorBase<3, __Data> _VectorBase;

    __Data & x;
    __Data & y;
    __Data & z;

    template<class... __Args>
    Vector( __Args&&... args )
    :
        _VectorBase( std::forward<__Args>( args )... ),
        //Vector<2, Ref<typename remove_ref<__Data>::type> >( _Parent::at( 0 ), _Parent::at( 1 ) ),
        x( this->at( 0 ) ),
        y( this->at( 1 ) ),
        z( this->at( 2 ) )
    {
        //
    }

    _Vector & operator=( _Vector const & other )
    {
        _VectorBase::copyFrom( other );

        x = other.x;
        y = other.y;
        z = other.z;

        return *this;
    }
};

/*template<class __Data>
class Vector<6, __Data> : public VectorBase<Vector<6, __Data> >
{
public:
    typedef Vector<6, __Data> _Vector;
    typedef VectorBase<_Vector> _VectorBase;

    Vector<3, Ref<typename remove_ref<__Data>::type> > linear;
    Vector<3, Ref<typename remove_ref<__Data>::type> > angular;

    template<class... __Args>
    Vector( __Args&&... args )
    :
        _VectorBase( std::forward<__Args>( args )... ),
        linear
        (
            this->at( 0 ),
            this->at( 1 ),
            this->at( 2 )
        ),
        angular
        (
            this->at( 3 ),
            this->at( 4 ),
            this->at( 5 )
        )
    {
        //
    }

    _Vector & operator=( _Vector const & other )
    {
        _VectorBase::copyFrom( other );

        linear = other.linear;
        angular = other.angular;

        return *this;
    }
};*/

template<size_t __Dim__, class __Data, class __Derived>
class VectorWrapperBase : public Vector<__Dim__, __Data>
{
public:
    typedef Vector<__Dim__, __Data> _Vector;
    typedef VectorBase<__Dim__, __Data> _VectorBase;
    typedef VectorWrapperBase<__Dim__, __Data, __Derived> _VectorWrapperBase;

    template<class... __Args>
    VectorWrapperBase( __Args&&... args )
    :
        Vector<__Dim__, __Data>( std::forward<__Args>( args )... )
    {
        //
    }

    __Derived & getDerived()
    {
        return *(__Derived*)this;
    }

    __Derived getDerived() const
    {
        return *(__Derived*)this;
    }

    __Derived copy() const
    {
        return getDerived();
    }

    _VectorBase copyStorage() const
    {
        return VectorBase<__Dim__, __Data>::copy();
    }

    _VectorBase & getStorage()
    {
        return *(VectorBase<__Dim__, __Data>*)this;
    }

    _VectorBase getStorage() const
    {
        return copyStorage();
    }

    template<class __Other>
    __Derived & operator=( __Other const & other )
    {
        copyFrom( other );

        return getDerived();
    }

    __Derived & operator=( _VectorBase const & other )
    {
        copyFrom( other );

        return getDerived();
    }

    /*_VectorWrapperBase & operator=( _VectorBase const & other )
    {
        copyFrom( other );

        return *this;
    }*/

        // entry point for operators; operation<>() handles vector and single-value-numeric specialization
    template<class __Value>
    __Derived & operator+=( __Value const & value ){ return operation<vector_base::operations::Add>( value ); }

    template<class __Value>
    __Derived operator+( __Value const & value ) const { return operation<vector_base::operations::Add>( *this, value ); }

    template<class __Value>
    __Derived & operator-=( __Value const & value ) { return operation<vector_base::operations::Sub>( value ); }

    template<class __Value>
    __Derived operator-( __Value const & value ) const { return operation<vector_base::operations::Sub>( *this, value ); }

    template<class __Value>
    __Derived & operator*=( __Value const & value ){ return operation<vector_base::operations::Mul>( value ); }

    template<class __Value>
    __Derived operator*( __Value const & value ) const { return operation<vector_base::operations::Mul>( *this, value ); }

    template<class __Value>
    __Derived & operator/=( __Value const & value ){ return operation<vector_base::operations::Div>( value ); }

    template<class __Value>
    __Derived operator/( __Value const & value ) const { return operation<vector_base::operations::Div>( *this, value ); }

protected:

    //! Add two vectors (per-element)
    template<class __Operation>
    __Derived operation( _VectorWrapperBase const & vec1, _VectorWrapperBase const & vec2 ) const
    {
        __Derived result;
        auto to_elem = result.begin();
        auto other_elem = vec2.cbegin();

        for( auto from_elem = vec1.cbegin(); from_elem != vec1.cend(); ++from_elem, ++other_elem, ++to_elem )
        {
            *to_elem = _VectorBase::template elemOperation<__Operation>( *from_elem, *other_elem );
        }

        return result;
    }

    //! Add two vectors (per-element)
    template<class __Operation>
    __Derived & operation( _VectorWrapperBase const & other )
    {
        auto other_elem = other.cbegin();

        for( auto from_elem = this->begin(); from_elem != this->end(); ++from_elem, ++other_elem )
        {
            *from_elem = _VectorBase::template elemOperation<__Operation>( *from_elem, *other_elem );
        }

        return getDerived();
    }

    //! Add another numeric value to a vector (per-element)
    template
    <
        class __Operation,
        class __Value,
        typename std::enable_if<(std::is_arithmetic<__Value>::value), int>::type = 0
    >
    __Derived operation( _VectorWrapperBase const & vec, __Value const & value ) const
    {
        __Derived result;
        auto to_elem = result.begin();
        for( auto from_elem = vec.cbegin(); from_elem != vec.cend(); ++from_elem, ++to_elem )
        {
            *to_elem = _VectorBase::template elemOperation<__Operation>( *from_elem, value );
        }

        return result;
    }

    //! Add another numeric value to a vector (per-element)
    template
    <
        class __Operation,
        class __Value,
        typename std::enable_if<(std::is_arithmetic<__Value>::value), int>::type = 0
    >
    __Derived & operation( __Value const & value )
    {
        for( auto from_elem = this->begin(); from_elem != this->end(); ++from_elem )
        {
            *from_elem = _VectorBase::template elemOperation<__Operation>( *from_elem, value );
        }

        return getDerived();
    }
};

template<size_t __Dim__, class __Data, class __Derived = void>
class VectorWrapper : public VectorWrapperBase<__Dim__, __Data, typename passthrough_derived<VectorWrapper<__Dim__, __Data, void>, __Derived>::type>
{
public:
    typedef VectorWrapperBase<__Dim__, __Data, typename passthrough_derived<VectorWrapper<__Dim__, __Data, void>, __Derived>::type> _VectorWrapperBase;

    template<class... __Args>
    VectorWrapper( __Args&&... args )
    :
        _VectorWrapperBase( std::forward<__Args>( args )... )
    {
        //
    }
};

} // quickdev


#endif // QUICKDEVCPP_QUICKDEV_VECTOR_H_
