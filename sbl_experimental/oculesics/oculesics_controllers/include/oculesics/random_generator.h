#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <array>
#include <vector>

#ifndef OCULESICS_RANDOM_GENERATOR_H_
#define OCULESICS_RANDOM_GENERATOR_H_

template<class __DataType = double>
class DiscreteDistribution
{
public:
	typedef __DataType _DataType;
	typedef std::vector<__DataType> _Array;
	// probabilities will be normalized beforehand
	_Array probabilities_;
	
	DiscreteDistribution( const _Array & probabilities = _Array() ) : probabilities_( probabilities ) {}
	
	template<class __Engine>
	unsigned int operator() ( __Engine & engine )
	{
		__DataType total = 0;
		const __DataType selection ( __DataType ( engine() - engine.min() ) / __DataType( engine.max() - engine.min() ) );
		
		printf( "selection: %f\n", selection );
		
		for( unsigned int i = 0; i < probabilities_.size(); ++i )
		{
			total += probabilities_[i];
			printf( "interval %u: %f\n", i, total );
			if( selection <= total ) return i;
		}
		return 0;
	}
};

template<class __DataType>
class Range
{
public:
	typedef __DataType _DataType;
	
	__DataType min, max;
};

template<class __Distribution, unsigned int __Dim__ = 1, class __Engine = boost::mt19937 >
class RandomGeneratorBase
{
public:
	typedef std::array<__Distribution, __Dim__> _DistributionArray;
	typedef __Distribution _Distribution;
	const static unsigned int dim_ = __Dim__;
	typedef __Engine _Engine;
	
	__Engine engine_;
	_DistributionArray distributions_;
};

template<class __Distribution, unsigned int __Dim__, class __Engine>
class RandomGenerator : public RandomGeneratorBase<__Distribution, __Dim__, __Engine>{};

template<unsigned int __Dim__, class __Engine>
class RandomGenerator<boost::uniform_real<>, __Dim__, __Engine> : public RandomGeneratorBase<boost::uniform_real<>, __Dim__, __Engine>
{
public:
	typedef RandomGeneratorBase<boost::uniform_real<>, __Dim__, __Engine> _RandomGeneratorBase;
	typedef std::array<double, __Dim__> _OutputArray;
	
	template<class __DataType>
	void update( unsigned int index, __DataType & min, __DataType & max )
	{
		this->distributions_[index] = typename _RandomGeneratorBase::_Distribution( min, max );
	}
	
	_OutputArray sample()
	{
		_OutputArray outputs;
		for( unsigned int i = 0; i < this->distributions_.size(); ++i )
		{
			outputs[i] = this->distributions_[i]( this->engine_ );
		}
		return outputs;
	}
};

template<unsigned int __Dim__, class __Engine>
class RandomGenerator<DiscreteDistribution<>, __Dim__, __Engine> : public RandomGeneratorBase<DiscreteDistribution<>, __Dim__, __Engine>
{
public:
	typedef RandomGeneratorBase<DiscreteDistribution<>, __Dim__, __Engine> _RandomGeneratorBase;
	typedef std::array<int, __Dim__> _OutputArray;
	
	void update( unsigned int index, const typename _RandomGeneratorBase::_Distribution::_Array & probabilities )
	{
		this->distributions_[index] = typename _RandomGeneratorBase::_Distribution( probabilities );
	}
	
	_OutputArray sample()
	{
		_OutputArray outputs;
		for( unsigned int i = 0; i < this->distributions_.size(); ++i )
		{
			outputs[i] = this->distributions_[i]( this->engine_ );
		}
		return outputs;
	}
	
	typename _RandomGeneratorBase::_Distribution::_Array::value_type getProbability( unsigned int distribution_index, unsigned int probability_index )
	{
		return this->distributions_[distribution_index].probabilities_[probability_index];
	}
};

#endif // OCULESICS_RANDOM_GENERATOR_H_
