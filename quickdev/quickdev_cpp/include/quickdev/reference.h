#ifndef QUICKDEVCPP_QUICKDEV_REFERENCE_H_
#define QUICKDEVCPP_QUICKDEV_REFERENCE_H_

#include <quickdev/macros.h>
#include <boost/type_traits/is_base_of.hpp>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

template<class __Data>
class Ref
{
protected:
    __Data * ptr_;

public:
    Ref( __Data & ref )
    :
        ptr_( &ref )
    {
        //
    }

    Ref()
    :
        ptr_( NULL )
    {
        //
    }

    Ref<__Data> & operator=( Ref<__Data> const & other )
    {
        return copyFrom( other );
    }

    Ref<__Data> & operator=( __Data const & ref )
    {
        return copyFrom( ref );
    }

    Ref<__Data> & operator=( __Data & ref )
    {
        return copyFrom( ref );
    }

    __Data const & cget() const
    {
        return *ptr_;
    }

    __Data & get()
    {
        return *ptr_;
    }

    Ref<__Data> & copyFrom( Ref<__Data> const & other )
    {
        ptr_ = other.ptr_;

        return *this;
    }

    Ref<__Data> & copyFrom( __Data const & ref )
    {
        if( ptr_ ) *ptr_ = ref;

        return *this;
    }

    Ref<__Data> & copyFrom( __Data & ref )
    {
        ptr_ = &ref;

        return *this;
    }

    void copyTo( Ref<__Data> & other ) const
    {
        other.ptr_ = ptr_;
    }

    __Data copy() const
    {
        return cget();
    }

    operator __Data() const
    {
        return copy();
    }
};

//! Recursively strip Ref classes; base case
template<class __Data>
struct remove_ref
{
    //! Given __Data = Ref<Ref<...<T> > >; type will be T
    typedef __Data type;
};

//! Recursively strip Ref classes; recursive case; pop one Ref off of __Data
template<class __Data>
struct remove_ref<Ref<__Data> >
{
    //! Given __Data = Ref<Ref<...<T> > > >; type will be Ref<...<T> >
    typedef typename remove_ref<__Data>::type type;
};

} // quickdev


#endif // QUICKDEVCPP_QUICKDEV_REFERENCE_H_
