/***************************************************************************
 *  include/quickdev/image_proc_policy.h
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

#ifndef QUICKDEVCPP_QUICKDEV_IMAGEPROCPOLICY_H_
#define QUICKDEVCPP_QUICKDEV_IMAGEPROCPOLICY_H_

#include <quickdev/node_handle_policy.h>
#include <quickdev/multi_publisher.h>
#include <quickdev/multi_subscriber.h>
#include <quickdev/opencv_conversion.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>

// ## PublisherAdapterStorage for image_transport::Publisher ###########
namespace ros
{

template<>
struct PublisherAdapterStorage<image_transport::Publisher> : PublisherAdapterStorage<ros::Publisher>
{
    typedef PublisherAdapterStorage<ros::Publisher> _Parent;
    typedef PublisherAdapterStorage<image_transport::Publisher> _PublisherAdapterStorage;
    typedef boost::shared_ptr<_PublisherAdapterStorage> _Ptr;

    image_transport::ImageTransport * image_transport_;

    template<class... __ParentArgs>
    PublisherAdapterStorage( const decltype( image_transport_ ) & image_transport, __ParentArgs&&... parent_args )
    :
        _Parent( std::forward<__ParentArgs>( parent_args )... ),
        image_transport_( image_transport )
    {}
};

// ## PublisherAdapter for image_transport::Publisher ##################
template<>
class PublisherAdapter<image_transport::Publisher>
{
public:
    typedef image_transport::Publisher _Publisher;
    typedef PublisherAdapterStorage<_Publisher> _Storage;
    typedef _Storage::_Ptr _StoragePtr;

    _StoragePtr storage_;
    _Publisher publisher_;

    PublisherAdapter( _StoragePtr storage = _StoragePtr() )
    :
        storage_( storage )
    {
        //
    }

    template<class __Message>
    _Publisher & createPublisher(
        ros::NodeHandle & nh,
        std::string const & topic )
    {
        if( !storage_ ) PRINT_ERROR( "Storage not allocated for publisher adapter!" );
        publisher_ = storage_->image_transport_->advertise(
            topic,
            storage_->cache_size_ );

        return publisher_;
    }
};

// ## SubscriberAdapterStorage for image_transport::Subscriber #########
template<>
struct SubscriberAdapterStorage<image_transport::Subscriber> : SubscriberAdapterStorage<ros::Subscriber>
{
    typedef SubscriberAdapterStorage<ros::Subscriber> _Parent;
    typedef SubscriberAdapterStorage<image_transport::Subscriber> _SubscriberAdapterStorage;
    typedef boost::shared_ptr<_SubscriberAdapterStorage> _Ptr;

    image_transport::ImageTransport * image_transport_;

    template<class... __ParentArgs>
    SubscriberAdapterStorage( const decltype( image_transport_ ) & image_transport, __ParentArgs&&... parent_args )
    :
        _Parent( std::forward<__ParentArgs>( parent_args )... ),
        image_transport_( image_transport )
    {}
};

// ## SubscriberAdapter for image_transport::Subscriber ################
template<>
class SubscriberAdapter<image_transport::Subscriber>
{
public:
    typedef image_transport::Subscriber _Subscriber;
    typedef SubscriberAdapterStorage<_Subscriber> _Storage;
    typedef _Storage::_Ptr _StoragePtr;

    _StoragePtr storage_;
    _Subscriber subscriber_;

    SubscriberAdapter( _StoragePtr storage = _StoragePtr() )
    :
        storage_( storage )
    {
        //
    }

    template<class __Message>
    _Subscriber & createSubscriber(
        ros::NodeHandle & nh,
        std::string const & topic,
        __QUICKDEV_FUNCTION_TYPE<void( boost::shared_ptr<__Message const> const & )> const & callback )
    {
        if( !storage_ ) PRINT_ERROR( "Storage not allocated for subscriber adapter!" );
        subscriber_ = storage_->image_transport_->subscribe(
            topic,
            storage_->cache_size_,
            boost::function< void( boost::shared_ptr< __Message const> const & )>( callback ) );

        return subscriber_;
    }
};

}

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

QUICKDEV_DECLARE_POLICY( ImageProc, NodeHandlePolicy )

QUICKDEV_DECLARE_POLICY_CLASS( ImageProc )
{
    QUICKDEV_MAKE_POLICY_FUNCS( ImageProc )

    typedef ros::PublisherAdapterStorage<image_transport::Publisher> _PublisherAdapterStorage;
    typedef ros::SubscriberAdapterStorage<image_transport::Subscriber> _SubscriberAdapterStorage;
    typedef __QUICKDEV_FUNCTION_TYPE<void(cv_bridge::CvImageConstPtr const &)> _ImageCallback;

protected:
    ros::MultiPublisher<image_transport::Publisher> image_pubs_;
    ros::MultiSubscriber<image_transport::Subscriber> image_subs_;

    image_transport::ImageTransport image_transport_;

    std::map<std::string, _ImageCallback> image_callbacks_map_;

    QUICKDEV_DECLARE_POLICY_CONSTRUCTOR( ImageProc ),
        image_transport_( NodeHandlePolicy::getNodeHandle() ),
//        publisher_storage_( &image_transport_, 1 ),
//        subscriber_storage_( &image_transport_, 1 ),
        initialized_( false )
    {
        printPolicyActionStart( "create", this );
        printPolicyActionDone( "create", this );
    }

    QUICKDEV_ENABLE_INIT()
    {
        //publisher_storage.image_transport_ = &image_transport_;
        //subscriber_storage.image_transport_ = &image_transport_;

        //auto & nh_rel = NodeHandlePolicy::getNodeHandle();
        QUICKDEV_GET_NODEHANDLE( nh_rel );

        auto const subscribe_to_image = policy::readPolicyParam<bool>( nh_rel, "subscribe_to_image_param", "subscribe_to_image", true, std::forward<__Args>( args )... );
        auto const publish_image = policy::readPolicyParam<bool>( nh_rel, "publish_image_param", "publish_image", true, std::forward<__Args>( args )... );

        if( subscribe_to_image )
        {
            auto const image_topic = policy::readPolicyParam<std::string>( nh_rel, "image_topic_param", "image_topic", "image", std::forward<__Args>( args )... );
            auto const image_cache_size = policy::readPolicyParam<int>( nh_rel, "image_cache_size_param", "image_cache_size", 1, std::forward<__Args>( args )... );
            addImageSubscriber( image_topic, getMetaParam<_ImageCallback>( "image_callback_param", std::forward<__Args>( args )... ), image_cache_size );
        }

        if( publish_image )
        {
            auto const output_image_topic = policy::readPolicyParam<std::string>( nh_rel, "output_image_topic_param", "output_image_topic", "output_image", std::forward<__Args>( args )... );
            addImagePublisher( output_image_topic );
        }

        QUICKDEV_SET_INITIALIZED();
    }

    void addImageSubscriber( std::string const & topic_name, __QUICKDEV_FUNCTION_TYPE<void(cv_bridge::CvImageConstPtr const &)> const & callback, size_t const & cache_size = 1 )
    {
        QUICKDEV_GET_NODEHANDLE( nh_rel );

        auto const callback_it = image_callbacks_map_.find( topic_name );

        if( callback_it != image_callbacks_map_.end() ) PRINT_WARN( "Topic %s is already registered; overwriting with new callback.", topic_name.c_str() );

        image_callbacks_map_[topic_name] = callback;

        image_subs_.addSubscriber( nh_rel, topic_name, auto_bind( auto_bind( &ImageProcPolicy::imageCB, this ), topic_name ), quickdev::make_shared( new _SubscriberAdapterStorage( &image_transport_, cache_size ) ) );
    }

    void addImagePublisher( std::string const & topic_name, size_t const & cache_size = 1 )
    {
        QUICKDEV_GET_NODEHANDLE( nh_rel );

        image_pubs_.addPublishers<sensor_msgs::Image>( nh_rel, { topic_name }, { quickdev::make_shared( new _PublisherAdapterStorage( &image_transport_, cache_size ) ) } );
    }

/*
    virtual IMAGE_PROC_PROCESS_IMAGE( image_ptr )
    {

        // Usual behavior: grab an IplImage * or a cvMat from image_ptr and do your thing
        // When done, if applicable, publish a debug image on the given topic
        //
        // IplImage * image = &IplImage( image_ptr->image );
        // image_pubs_.publish( "output_image", image_ptr->toImageMsg() );

    }
*/
    void imageCB( sensor_msgs::Image::ConstPtr const & image_msg, std::string const & topic_name )
    {
        auto callback_it = image_callbacks_map_.find( topic_name );

        if( callback_it != image_callbacks_map_.end() ) callback_it->second( opencv_conversion::fromImageMsg( image_msg ) );
        else PRINT_WARN( "Topic %s is not registered.", topic_name.c_str() );
    }

    // publish specializaiton for sensor_msgs::Image::Ptr
    void publishImages( std::string const & topic, sensor_msgs::Image::Ptr const & image_ptr ) const
    {
        //PRINT_INFO( "publishing image (CvImageConstPtr) on topic %s", topic.c_str() );

        image_pubs_.publish( topic, image_ptr );
    }

    // publish specializaiton for cv_bridge::CvImageConstPtr
    void publishImages( std::string const & topic, cv_bridge::CvImageConstPtr & image_ptr ) const
    {
        //PRINT_INFO( "publishing image (CvImageConstPtr) on topic %s", topic.c_str() );

        publishImages( topic, image_ptr->toImageMsg() );
    }

    // generic recursive publish with topic/image pairs
    // enabled only if __Rest&&... is not empty
    template<class __Image, class... __Rest>
    typename std::enable_if<(sizeof...(__Rest) > 0), void>::type
    publishImages( std::string const & topic, __Image & image, __Rest&&... rest ) const
    {
        // publish using specialization for __Image
        publishImages( topic, image );
        publishImages( std::forward<__Rest>( rest )... );
    }

    //void publishImage( IplImage * image_ptr ){}


};

}

#endif // QUICKDEVCPP_QUICKDEV_IMAGEPROCPOLICY_H_
