/***************************************************************************
 *  include/humanoid/soft_classification.h
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

#ifndef HUMANOID_SOFTCLASSIFICATION_H_
#define HUMANOID_SOFTCLASSIFICATION_H_

#include <quickdev/param_reader.h>
#include <gsl/gsl_cdf.h>
#include <set>

#include <humanoid_models/SoftClassification.h>

class SoftClassification
{
public:
    typedef humanoid_models::SoftClassification _SoftClassificationMsg;

    int id_;
    double likelihood_;
    double log_likelihood_;
    std::string name_;

    typedef std::set<SoftClassification> _SoftClassificationSet;

    SoftClassification( int const & id = 0, double const & likelihood = 0, std::string const & name = "" )
    :
        id_( id ),
        likelihood_( likelihood ),
        log_likelihood_( log( likelihood ) ),
        name_( name )
    {
        //
    }

    static _SoftClassificationSet sampleIntervals( XmlRpc::XmlRpcValue * const begin, XmlRpc::XmlRpcValue * const end, double const & mean, double const & sigma )
    {
        _SoftClassificationSet result;

        if( begin )
        {
            for( XmlRpc::XmlRpcValue * interval_it = begin; interval_it != end; ++interval_it )
            {
                auto & interval = *interval_it;

                auto const interval_id = quickdev::ParamReader::getXmlRpcValue<int>( interval, "id" );
                auto const interval_min = quickdev::ParamReader::getXmlRpcValue<double>( interval, "min" );
                auto const interval_max = quickdev::ParamReader::getXmlRpcValue<double>( interval, "max" );
                auto const interval_name = quickdev::ParamReader::getXmlRpcValue<std::string>( interval, "name" );

                //PRINT_INFO( "evaluating interval %s (%i); u: %f, s: %f; [%f, %f]", interval_name.c_str(), interval_id, mean, sigma, interval_min, interval_max );

                result.insert( SoftClassification( interval_id, gsl_cdf_gaussian_P( interval_max - mean, sigma ) - gsl_cdf_gaussian_P( interval_min - mean, sigma ), interval_name ) );
            }
        }
        else
        {
            PRINT_WARN( "interval pointer is not valid" );
        }

        return result;
    }

    bool operator<( SoftClassification const & other ) const
    {
        return ( ( likelihood_ == other.likelihood_ || std::isnan( likelihood_ ) || std::isnan( other.likelihood_ ) ) && id_ > other.id_ ) || likelihood_ > other.likelihood_;
    }

    operator _SoftClassificationMsg() const
    {
        _SoftClassificationMsg classification_msg;
        classification_msg.name = name_;
        classification_msg.likelihood = likelihood_;
        classification_msg.log_likelihood = log_likelihood_;
        return classification_msg;
    }
};

#endif // HUMANOID_SOFTCLASSIFICATION_H_
