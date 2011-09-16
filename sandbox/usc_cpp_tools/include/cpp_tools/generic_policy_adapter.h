#ifndef USC_CPP_TOOLS_CPP_TOOLS_GENERIC_POLICY_ADAPTER_H_
#define USC_CPP_TOOLS_CPP_TOOLS_GENERIC_POLICY_ADAPTER_H_

#include <type_traits>

template<bool __Enable__, class... __Types>
struct ContainerHelper
{
	//typedef void _Type;
};

template<class __Type>
struct ContainerHelper<true, __Type>
{
	typedef __Type _Type;
};

// stors a list of types; provides tools for accessing that list of types
template<class... __Types>
struct Container
{
	const static unsigned int num_types_ = sizeof...( __Types );
	
	// get a container for the type at the front of the container
	template<class __Front, class... __Rest>
	static Container<__Front> front( Container<__Front, __Rest...> container )
	{
		return Container<__Front>();
	}

	// get a container for the types after the front of the container
	template<class __Front, class... __Rest>
	static Container<__Rest...> rest( Container<__Front, __Rest...> container )
	{
		return Container<__Rest...>();
	}
	
	// typedef for the front type
	typedef typename ContainerHelper<num_types_ == 1, __Types...>::_Type _Type;
	// front() will always return a container of with a __Types... of size 1
	typedef decltype( front( Container<__Types...>() ) ) _Front;
	typedef decltype( rest( Container<__Types...>() ) ) _Rest;
};

template<class... __Policies>
class GenericPolicyAdapter : public __Policies...
{
	// Construct a policy adapter that inherits from all specified
	// policies. Note that all policies in the inheritance group take
	// the same set of args during construction.
	template<class... __Args>
	GenericPolicyAdapter( __Args&&... args ) : __Policies( args... )...
	{
		//
	}
	
	// Do any post-construction initialization. Note that all policies
	// receive the same set of args.
	template<class... __Args>
	void init( __Args&&... args )
	{
		init( Container<__Policies...>(), args... );
	}
	
	// initialize the first policy in the subset
	// recurse through the remaining policies in the subset
	template<class __PoliciesSubset, class... __Args>
	void init( __PoliciesSubset policies_cont, __Args&&... args )
	{
		__PoliciesSubset::_Front::init( args... );
		init( __PoliciesSubset::_Rest(), args... );
	}
};

#endif // USC_CPP_TOOLS_CPP_TOOLS_GENERIC_POLICY_ADAPTER_H_
