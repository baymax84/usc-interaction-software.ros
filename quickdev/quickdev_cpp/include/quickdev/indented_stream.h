#ifndef QUICKDEVCPP_QUICKDEV_INDENTEDSTREAM_H_
#define QUICKDEVCPP_QUICKDEV_INDENTEDSTREAM_H_

#include <quickdev/macros.h>

#include <iostream>
#include <stack>
#include <string>
#include <sstream>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

namespace IndentedStreamCounter
{
    static size_t level_;
    static std::string level_text_;
    static std::stack<std::string> message_;
};

static void make_stream_rec( std::stringstream const & ss ){}

template<class __Arg, class... __Args>
static void make_stream_rec( std::stringstream & ss, __Arg const & arg, __Args&&... args )
{
    ss << arg;
    make_stream_rec( ss, args... );
}

template<class... __Args>
static std::string make_stream( __Args&&... args )
{
    std::stringstream ss;
    make_stream_rec( ss, args... );
    return ss.str();
}

template<class... __Args>
static void start_stream_indented( __Args&&... args )
{
    std::string stream = make_stream( args... );

    std::cout << IndentedStreamCounter::level_text_ << ">>> " << stream << std::endl;

    IndentedStreamCounter::level_text_ += "    ";
    IndentedStreamCounter::level_ ++;
    IndentedStreamCounter::message_.push( stream );
}

template<class... __Args>
static void print_stream_indented( __Args&&... args )
{
    std::string stream = make_stream( args... );

    std::cout << IndentedStreamCounter::level_text_ << stream << std::endl;
}

static void end_stream_indented()
{
    if( IndentedStreamCounter::level_ == 0 ) return;

    IndentedStreamCounter::level_text_ = IndentedStreamCounter::level_text_.substr( 0, IndentedStreamCounter::level_text_.size() - 4 );

    std::cout << IndentedStreamCounter::level_text_ << "<<< done " << IndentedStreamCounter::message_.top() << std::endl;

    IndentedStreamCounter::level_ --;
    IndentedStreamCounter::message_.pop();
}

} // quickdev

#endif // QUICKDEVCPP_QUICKDEV_INDENTEDSTREAM_H_
