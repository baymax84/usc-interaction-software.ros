/***************************************************************************
 *  include/quickdev/unit.h
 *  --------------------
 *
 *  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are
 *  met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following disclaimer
 *    in the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of usc-ros-pkg nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **************************************************************************/

#ifndef QUICKDEVCPP_QUICKDEV_UNIT_H_
#define QUICKDEVCPP_QUICKDEV_UNIT_H_

//#include <quickdev/container.h>
#include <math.h>
#include <type_traits>
#include <stdint.h>
#include <boost/type_traits/is_base_of.hpp>

#define GET_UNIT_NAMESPACE() \
unit

#define GET_UNIT_NS() \
GET_UNIT_NAMESPACE()

#define DECLARE_UNIT_NAMESPACE() \
namespace GET_UNIT_NAMESPACE()

#define DECLARE_UNIT_NS() \
DECLARE_UNIT_NAMESPACE()

#define DECLARE_UNIT_CONVERSION( __FromUnit, __ToUnit ) \
DECLARE_UNIT_NS(){ \
template<> \
struct can_convert<__FromUnit, __ToUnit>{ static bool const value = true; }; \
}

//namespace __FromUnit#_conversion{ typedef decltype( container_push( type_map, __ToUnit ) ) type_map; }

#define DECLARE_UNIT_CONVERSION_LAMBDA( __FromData, __ToData, var_name, conversion... ) \
DECLARE_UNIT_CONVERSION( __FromData, __ToData ) \
DECLARE_UNIT_NS() \
{ \
    template<> \
    struct UnitConverter<__FromData, __ToData, __ToData> \
    { \
        template<class __MFromStorage> \
        static typename std::enable_if<std::is_same<__MFromStorage, __FromData>::value, __ToData>::type \
        convert( __MFromStorage const & var_name ) \
        { \
            conversion \
        } \
    }; \
}

#define DECLARE_NUMERIC_UNIT_STORAGE_CONVERSION( __FromData, __ToData, __FromStorage, __ToStorage, var_name, conversion ) \
DECLARE_UNIT_CONVERSION( __FromData, __ToData ) \
DECLARE_UNIT_NS() \
{ \
    template<> \
    struct UnitConverter<__FromData, __ToData, __ToStorage> \
    { \
        template<class __MFromStorage> \
        static typename std::enable_if<std::is_same<__MFromStorage, __FromStorage>::value, __ToStorage>::type \
        convert( __MFromStorage const & var_name ) \
        { \
            return conversion; \
        } \
    }; \
}

#define DECLARE_NUMERIC_UNIT_CONVERSION( __FromData, __ToData, var_name, conversion ) \
DECLARE_UNIT_CONVERSION( __FromData, __ToData ) \
DECLARE_UNIT_NS() \
{ \
    template<class __ToStorage> \
    struct UnitConverter<__FromData, __ToData, __ToStorage> \
    { \
        template<class __FromStorage> \
        static __ToStorage convert( __FromStorage const & var_name ) \
        { \
            return conversion; \
        } \
    }; \
}

#define DECLARE_SCALAR_UNIT_CONVERSION( __FromData, __ToData, scalar ) \
DECLARE_NUMERIC_UNIT_CONVERSION( __FromData, __ToData, value, value * scalar ); \
DECLARE_NUMERIC_UNIT_CONVERSION( __ToData, __FromData, value, value / ( scalar ) )

// macro to declare some unit with simple numeric storage
#define DECLARE_NUMERIC_UNIT( __Unit, __Storage... ) \
class __Unit : public GET_UNIT_NS()::NumericUnit<__Unit, ## __Storage> \
{ \
public: \
    template<class... __ParentArgs> \
    __Unit( __ParentArgs&&... parent_args ) : GET_UNIT_NS()::NumericUnit<__Unit, ## __Storage>( std::forward<__ParentArgs>( parent_args )... ) {} \
}; \
 \
DECLARE_UNIT_NS(){ \
template<> \
struct is_numeric_helper<__Unit>{ static bool const value = true; }; \
}

DECLARE_UNIT_NS()
{

template<class __FromData, class __ToData, class __ToStorage>
struct UnitConverter{};

template<class __FromUnit, class __ToUnit>
struct can_convert{};

template<class __Unit>
struct is_numeric_helper{ static bool const value = false; };

template<class __Unit>
struct is_numeric
{
    static bool const value = is_numeric_helper<__Unit>::value;
};

template<class __Unit>
struct unit_traits
{
    static bool const is_numeric_ = is_numeric<__Unit>::value;
};

class UnitBaseType{};

template<class __Data, class __Storage>
class UnitBase : public UnitBaseType
{
public:
    typedef __Data _Data;
    typedef __Storage _Storage;
    typedef UnitBase<__Data, __Storage> _UnitBase;

protected:
    __Storage value_;

public:
    UnitBase(){}

    UnitBase( __Storage const value )
    :
        value_( value )
    {
        //
    }

    template<class __FromData, class __FromStorage>
    UnitBase( const UnitBase<__FromData, __FromStorage> & other_unit )
    {
        operator=( other_unit );
    }

    _UnitBase & operator=( __Storage const & value )
    {
        value_ = value;
        return *this;
    }

    _UnitBase & operator=( _UnitBase const & other_unit )
    {
        if( this != &other_unit )
        {
            // value_ = other_unit.value_;
            *this = other_unit;
        }
        return *this;
    }

    template<class __FromData, class __FromStorage>
    _UnitBase & operator=( const UnitBase<__FromData, __FromStorage> & other_unit )
    {
        value_ = UnitConverter<__FromData, __Data, __Storage>::convert( other_unit.getValue() );
        return *this;
    }

    __Storage & value()
    {
        return value_;
    }

    const __Storage & getValue() const
    {
        return value_;
    }

    operator __Storage()
    {
        return value();
    }

    operator __Storage() const
    {
        return getValue();
    }
};

template<class __Data, class __Storage = double>
class NumericUnit : public UnitBase<__Data, __Storage>
{
    static_assert( std::is_arithmetic<__Storage>::value, "NumericUnit can only store simple numeric types." );
public:
    typedef UnitBase<__Data, __Storage> _Parent;

    template<class... __ParentArgs>
    NumericUnit( __ParentArgs&&... parent_args )
    :
        _Parent( std::forward<__ParentArgs>( parent_args )... )
    {
        //
    }
};

template<class __Data, bool __IsNumeric__>
class UnitHelper : public UnitBase<__Data, __Data>
{
public:
    typedef UnitBase<__Data, __Data> _Parent;
    typedef _Parent _Unit;

    template<class... __ParentArgs>
    UnitHelper( __ParentArgs&&... parent_args )
    :
        _Parent( std::forward<__ParentArgs>( parent_args )... )
    {
        //
    }

    template<class __OtherData, typename std::enable_if<(!boost::is_base_of<UnitBaseType, __OtherData>::value), int>::type = 0>
    _Unit & operator=( __OtherData const & value )
    {
        operator=( UnitBase<__OtherData, __OtherData>( value ) );
        return *this;
    }

    // unroll any Unit<Unit<Type> > instances
    template<class __OtherData, typename std::enable_if<(boost::is_base_of<UnitBaseType, __OtherData>::value), int>::type = 0>
    _Unit & operator=( __OtherData const & value )
    {
        operator=( value.getValue() );
        return *this;
    }

    template<class __ToData>
    typename std::enable_if<(!boost::is_base_of<UnitBaseType, __ToData>::value), __ToData>::type
    static get_first_basic_type()
    {
        return __ToData();
    }

    template<class __ToData>
    typename std::enable_if<(boost::is_base_of<UnitBaseType, __ToData>::value), decltype( get_first_basic_type<typename __ToData::_Storage>() )>::type
    static get_first_basic_type()
    {
        return get_first_basic_type<typename __ToData::_Storage>();
    }

    template<class __ToData, class __FromData>
    typename std::enable_if<(!boost::is_base_of<UnitBaseType, __FromData>::value), __ToData>::type
    operatorCastHelper( __FromData const & value )
    {
        return UnitConverter<__FromData, __ToData, __ToData>::convert( value );
    }

    // unroll any Unit<Unit<Type> > instances
    template<class __ToData, class __FromData>
    typename std::enable_if<(boost::is_base_of<UnitBaseType, __FromData>::value), __ToData>::type
    operatorCastHelper( __FromData const & value )
    {
        return operatorCastHelper<__ToData>( value.getValue() );
    }

    template<class __ToData>//, typename std::enable_if<(!boost::is_base_of<UnitBaseType, __ToData>::value), int>::type = 0>
    operator __ToData()
    {
        return operatorCastHelper<__ToData>( this->value_ );
    }
};

template<class __Data>
class UnitHelper<__Data, true> : public NumericUnit<__Data>
{
public:
    typedef NumericUnit<__Data> _Parent;

    template<class... __ParentArgs>
    UnitHelper( __ParentArgs&&... parent_args )
    :
        _Parent( std::forward<__ParentArgs>( parent_args )... )
    {
        //
    }
};

template<class __Data>
class Unit : public UnitHelper< __Data, unit_traits<__Data>::is_numeric_ >
{
public:
    typedef UnitHelper< __Data, unit_traits<__Data>::is_numeric_ > _Parent;

    template<class... __ParentArgs>
    Unit( __ParentArgs&&... parent_args )
    :
        _Parent( std::forward<__ParentArgs>( parent_args )... )
    {
        //
    }
};

template<class __Data>
typename std::enable_if<(!unit_traits<__Data>::is_numeric_), Unit<__Data> >::type
make_unit( __Data const & data )
{
    return Unit<__Data>( data );
}

template<class __Data, class __Storage = double>
typename std::enable_if<(unit_traits<__Data>::is_numeric_), NumericUnit<__Data, __Storage> >::type
make_unit( __Storage const & value )
{
    return NumericUnit<__Data, __Storage>( value );
}

template<class __Data>
auto implicit_convert( __Data const & data ) -> decltype( make_unit( data ) )
{
    return make_unit( data );
}

template<class __ToData, class __FromData>
typename std::enable_if<(!unit_traits<__ToData>::is_numeric_ && !unit_traits<__FromData>::is_numeric_), __ToData >::type
convert_unit( Unit<__FromData> const & unit )
{
    return Unit<__ToData>( unit );
}

template<class __ToData, class __FromData>
typename std::enable_if<(!unit_traits<__ToData>::is_numeric_ && !unit_traits<__FromData>::is_numeric_), __ToData >::type
convert( Unit<__FromData> const & unit )
{
    return Unit<__ToData>( unit ).getValue();
}

template<class __ToData, class __FromData>
typename std::enable_if<(!unit_traits<__ToData>::is_numeric_ && !unit_traits<__FromData>::is_numeric_), __ToData >::type
convert( __FromData const & data )
{
    return Unit<__ToData>( make_unit( data ) ).getValue();
}

} // DECLARE_UNIT_NS()

#endif // QUICKDEVCPP_QUICKDEV_UNIT_H_
