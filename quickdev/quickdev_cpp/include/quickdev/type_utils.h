/***************************************************************************
 *  include/quickdev/type_utils.h
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

#ifndef QUICKDEVCPP_QUICKDEV_TYPEUTILS_H_
#define QUICKDEVCPP_QUICKDEV_TYPEUTILS_H_

#include <quickdev/console.h>
#include <quickdev/macros.h>
#include <quickdev/container.h>
#include <type_traits>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <boost/shared_ptr.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <ros/message.h>
//#include <ros/message_traits.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{
    //! Simple utility struct to determine if two statically-known data types contain the same value
    /*! Specialization for two different values */
    template<class __Data, __Data __Id1__, __Data __Id2__>
    struct is_same_value
    {
        //! The false type for differing IDs
        const static bool value = false;
    };

    //! Simple utility struct to determine if two statically-known data types contain the same value
    /*! Specialization for two identical values */
    template<class __Data, __Data __Id__>
    struct is_same_value<__Data, __Id__, __Id__>
    {
        //! The true type for matching IDs
        const static bool value = true;
    };

    //! Type designed to provide somewhat useful compiler output when using other type traits
    /*! Specifically, when using getFristOfType, getMetaParam, or getMetaParamDef, this type will be returned when
     *  no statically-known matches are found.*/
    struct TYPE_NOT_FOUND{};

    //! Specialization of getFirstOfType enabled when no matching types were found
    /*! This function is called after iterating through all types in the variadic template passed to getFirstOfType.
     *  The fact that any execution has arrived here signifies that the desired type does not exist in the given
     *  list of types. Therefore, we return the struct TYPE_NOT_FOUND to give some useful compiler output but
     *  still fail at compile time (unless the user was searching for a TYPE_NOT_FOUND in the list of types)
     *  \return TYPE_NOT_FOUND, a type designed to signify that no match was found but also clarify compiler output*/
    template<class __Desired>
    static __Desired getFirstOfType(){ return TYPE_NOT_FOUND(); }

    //! Specialization of getFirstOfType enabled when __Desired == __Current; returns the first item in a variadic template whose type matches __Desired
    /*! This function is called at most once during recursion when __Desired == __Current, which signifies that
     *  the requested type has been located in the the given variadic template and should be returned.
     *  \param current the item at the front of the variadic template
     *  \param rest the remaining items, if any, in the variadic template
     *  \return current the value of the current item in the list */
    template<class __Desired, class __Current, class... __Rest>
    static typename std::enable_if<std::is_same<__Desired, __Current>::value, __Desired>::type
    getFirstOfType( __Current & current, __Rest&&... rest )
    {
        return current;
    }

    //! Specialization of getFirstOfType enabled when __Desired != __Current
    /*! This function is called any time during recursion when __Desired != __Current, which signifies that
     *  a matching type has not yet been located in the given variadic template. This function will recursively
     *  check all types in __Rest until a better specialization is found (ie if a match is found or all types
     *  have been checked). Note that this function will fail to compile if __Desired != __Current for all __Rest.
     *  \param current the item at the front of the variadic template
     *  \param rest the remaining items, if any, in the variadic template
     *  \return getFirstOfType<__Desired>( rest... ) the best specialization of getFirstOfType for rest */
    template<class __Desired, class __Current, class... __Rest>
    static typename std::enable_if<!std::is_same<__Desired, __Current>::value, __Desired>::type
    getFirstOfType( __Current & current, __Rest&&... rest )
    {
        return getFirstOfType<__Desired>( rest... );
    }

    //! Specialization of tryGetMetaParamRec when a value of type __Desired was in the list but its key did not match the given key
    /*! This function is called after recursing through all key-value pairs in a variadic template containing at least
     *  one value of type __Desired without finding a matching key. This signifies that the requested key-value pair
     *  does not exist in the given list.
     *  \param name the name of the desired key
     *  \return __Desired() the default constructor value of the requested type */
    template<class __Desired>
    bool tryGetMetaParamRec( const std::string & name, __Desired const & desired )
    {
        PRINT_ERROR( ">>> Failed to find key [ %s ]", name.c_str() );
        return false;
    }

    // #### Forward declarations of all recursive variants of tryGetMetaParamRec

    template<class __Desired, class __Current, class... __Rest>
    static typename std::enable_if<(!std::is_same<__Desired, __Current>::value), bool>::type
    tryGetMetaParamRec( const std::string & name, __Desired & desired, const std::string & current_name, __Current & current, __Rest&&... rest );

    template<class __Desired, class __Current, class... __Rest>
    static typename std::enable_if<(std::is_same<__Desired, __Current>::value), bool>::type
    tryGetMetaParamRec( const std::string & name, __Desired & desired, const std::string & current_name, __Current & current, __Rest&&... rest );

    // #### Declarations of all recursive variants of tryGetMetaParamRec

    //! Specialization of tryGetMetaParamRec enabled if __Desired != __Current
    /*! This function is called any time during recursion when __Desired != __Current, which signifies that
     *  a matching type has not yet been located in the given variadic template. This function will recursively
     *  check all types in __Rest until a better specialization is found (ie if a match is found or all types
     *  have been checked). Note that this function will fail to compile if __Desired != __Current for all __Rest.
     *  \param name the value of the desired key
     *  \param current_name the value of the current key
     *  \param current the item at the front of the variadic template
     *  \param rest the remaining items, if any, in the variadic template
     *  \return getMetaParamRec<__Desired>( name, rest... ) the next best specialization of getMetaParamRec for the given arguments */
    template<class __Desired, class __Current, class... __Rest>
    static typename std::enable_if<(!std::is_same<__Desired, __Current>::value), bool>::type
    tryGetMetaParamRec( const std::string & name, __Desired & desired, const std::string & current_name, __Current & current, __Rest&&... rest )
    {
        return tryGetMetaParamRec<__Desired>( name, desired, rest... );
    }

    //! Specialization of tryGetMetaParamRec enabled when __Desired == __Current
    /*! This function is called any time during recursion when __Desired == __Current, which signifies that
     *  the requested type has been located in the the given variadic template and the corresponding key should be checked.
     *  If the keys do not match, the next matching type is recursively selected.
     *  \param name the value of the desired key
     *  \param current_name the value of the current key
     *  \param current the item at the front of the variadic template
     *  \param rest the remaining items, if any, in the variadic template
     *  \return current if the given keys match or getMetaParamRec<__Desired>( name, rest... ), the next best specialization of getMetaParamRec for the given arguments */
    template<class __Desired, class __Current, class... __Rest>
    static typename std::enable_if<(std::is_same<__Desired, __Current>::value), bool>::type
    tryGetMetaParamRec( const std::string & name, __Desired & desired, const std::string & current_name, __Current & current, __Rest&&... rest )
    {
        if( name == current_name )
        {
            std::stringstream ss;
            ss << current;
            PRINT_INFO( "Found key [ %s ] with value [ %s ]", name.c_str(), ss.str().c_str() );
            desired = current;
            return true;
        }
        return tryGetMetaParamRec<__Desired>( name, desired, rest... );
    }

    // #### Entry points for getMetaParam and getMetaParamDef

    //! A function that, given a variadic list of key-value pairs, and a desired type, will return the first item of the given type with a matching key
    /*! Entry point. Given a list of key-value pairs ( key, value, key, value, ... ), this function will recursively
     *  locate the first key-value pair whose value component matches the desired type, then try to match the key-component
     *  to the given key. If the keys do not match, the next type is located until there are no more types to check.
     *
\verbatim
Usage:
    getMetaParam<DesiredType>( "<desired_key>", "some_key", some_value, "some_other_key", some_other_value, ... );
    getMetaParam<DesiredType>( "<desired_key>" ); // evaluates to DesiredType()
\endverbatim
     *
     *  \param name the value of the desired key
     *  \param rest the list of key-value pairs
     *  \return getMetaParamRec<__Desired>( name, rest... ) the next best specialization of getMetaParamRec for the given arguments */
    template<class __Desired, class... __Rest>
    static bool tryGetMetaParam( const std::string & name, __Desired & desired, __Rest&&... rest )
    {
        // make sure desired type exists in list; otherwise fail at compile time
        //getFirstOfType<__Desired>( rest... );
        return tryGetMetaParamRec<__Desired>( name, desired, rest... );
    }

    //! A function that, given a variadic list of key-value pairs, and a desired type, will return the first item of the given type with a matching key
    /*! Entry point. Given a list of key-value pairs ( key, value, key, value, ... ), this function will recursively
     *  locate the first key-value pair whose value component matches the desired type, then try to match the key-component
     *  to the given key. If the keys do not match, the next type is located until there are no more types to check.
     *
\verbatim
Usage:
    getMetaParam<DesiredType>( "<desired_key>", "some_key", some_value, "some_other_key", some_other_value, ... );
    getMetaParam<DesiredType>( "<desired_key>" ); // evaluates to DesiredType()
\endverbatim
     *
     *  \param name the value of the desired key
     *  \param rest the list of key-value pairs
     *  \return getMetaParamRec<__Desired>( name, rest... ) the next best specialization of getMetaParamRec for the given arguments */
    template<class __Desired, class... __Rest>
    static __Desired getMetaParam( const std::string & name, __Rest&&... rest )
    {
        // make sure desired type exists in list; otherwise fail at compile time
        //getFirstOfType<__Desired>( rest... );
        __Desired desired;
        tryGetMetaParamRec<__Desired>( name, desired, rest... );
        return desired;
    }

    //! A function that, given a variadic list of key-value pairs, and a desired type, will return the first item of the given type with a matching key or a default value if no match was found
    /*! Entry point. Given a list of key-value pairs ( key, value, key, value, ... ), this function will recursively
     *  locate the first key-value pair whose value component matches the desired type, then try to match the key-component
     *  to the given key. If the keys do not match, the next type is located until there are no more types to check. If a
     *  matching key-value pair is not found, the function will return a given default value.
     *
\verbatim
Usage:
    getMetaParamDef<DesiredType>( "<desired_key>", default_value, "some_key", some_value, "some_other_key", some_other_value, ... );
    getMetaParamDef<DesiredType>( "<desired_key>", default_value ); // evaluates to default_value
\endverbatim
     *
     *  \param name the value of the desired key
     *  \param default_value the value to return if no matching key-value pairs were found
     *  \param rest the list of key-value pairs
     *  \return getMetaParamDefRec<__Desired>( name, default_value, rest... ) the best specialization of getMetaParamDefRec for the given arguments */
    template<class __Desired, class... __Rest>
    static __Desired getMetaParamDef( const std::string & name, const __Desired & default_value, __Rest&&... rest )
    {
        __Desired desired;
        return tryGetMetaParamRec<__Desired>( name, desired, rest... ) ? desired : default_value;
    }

    template<class... __Rest>
    static void make_key_rec( std::stringstream const & ss ){}

    template<class __Current, class... __Rest>
    static void make_key_rec( std::stringstream & ss, __Current const & current, __Rest... rest )
    {
        ss << current;
        make_key_rec( ss, rest... );
    }

    template<class... __Args>
    static std::string make_key( __Args... args )
    {
        std::stringstream ss;
        make_key_rec( ss, args... );
        return ss.str();
    }

    template<class... __Args>
    static std::string make_key_if( bool const & enable, __Args... args )
    {
        if( enable ) return make_key( args... );
        return make_key( variadic::at<0>( args... ) );
    }

    //! Simple struct to hold useful pointer types
    template<class __Type>
    struct ptr_types
    {
        //! A shared pointer for __Type
        typedef boost::shared_ptr<__Type> _Shared;
        //! A const shared pointer for __Type
        typedef boost::shared_ptr<__Type const> _Const;
    };

    /*template<class __MessagePtr>
    typename std::enable_if<(boost::is_base_of<ros::Message, typename __MessagePtr::element_type>::value), typename __MessagePtr::element_type>::type
    getMessageType( const __MessagePtr & message_ptr )
    {
        return typename __MessagePtr::element_type();
    }*/

    //! Specialization of getMessageType for any class instance typed on a non-const __Message derived from ros::Message
    /*! Returns the type of the ROS message on which the given class is typed. As an example, if we have an instance
     *  of some_pkg::SomeMessage::Ptr called some_msg_ptr, getMessageType( some_msg_ptr ) will resolve to some_pkg::SomeMessage().
     *  \param message an instance of the class
     *  \return __Message() the default constructor for __Message */
    template<class __Message, template<typename> class __Ptr>
    typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), __Message>::type
    getMessageType( const __Ptr<__Message const> & message )
    {
        return __Message();
    }

    //! Specialization of getMessageType for any class instance typed on a const __Message derived from ros::Message
    /*! Returns the type of the ROS message on which the given class is typed. As an example, if we have an instance
     *  of some_pkg::SomeMessage::ConstPtr called some_msg_ptr, getMessageType( some_msg_ptr ) will resolve to some_pkg::SomeMessage().
     *  \param message an instance of the class
     *  \return __Message() the default constructor for __Message */
    template<class __Message, template<typename> class __Ptr>
    typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), __Message>::type
    getMessageType( const __Ptr<__Message> & message )
    {
        return __Message();
    }

    //! Specialization of getMessageType for any __Message derived from ros::Message
    /*! Returns the given message instance. As an example, if we have an instance of some_pkg::SomeMessage
     *  called some_msg, getMessageType( some_msg ) will resolve to some_msg.
     *  \param message an instance of __Message
     *  \return message */
    template<class __Message>
    typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), __Message>::type
    getMessageType( const __Message & message )
    {
        return message;
    }

    //! Converts a ROS message into a ::ConstPtr
    /*! Given an instance of __Message derived from ros::Message, copies the message to a new __Message::ConstPtr
     *  \param message the message instance to use
     *  \return a __Message::ConstPtr with a copy of the data in message */
    template<class __Message>
    typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), typename __Message::ConstPtr>::type
    make_const_shared( const __Message & message )
    {
        return typename __Message::ConstPtr( new __Message( message ) );
    }

    //! Converts a ROS message into a ::Ptr
    /*! Given an instance of __Message derived from ros::Message, copies the message to a new __Message::Ptr
     *  \param message the message instance to use
     *  \return a __Message::Ptr with a copy of the data in message */
    template<class __Message>
    typename std::enable_if<(boost::is_base_of<ros::Message, __Message>::value), typename __Message::Ptr>::type
    make_shared( const __Message & message )
    {
        return typename __Message::Ptr( new __Message( message ) );
    }

    template<
        class __Data,
        typename std::enable_if<(!boost::is_base_of<ros::Message, __Data>::value), int>::type = 0
    >
    boost::shared_ptr<__Data> make_shared( __Data * const data )
    {
        return boost::shared_ptr<__Data>( data );
    }
}

#endif // QUICKDEVCPP_QUICKDEV_TYPEUTILS_H_
