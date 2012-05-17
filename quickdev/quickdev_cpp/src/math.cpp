#include <quickdev/math.h>

QUICKDEV_DECLARE_INTERNAL_NAMESPACE()
{

// =============================================================================================================================================
void seedRand( unsigned int const & value )
{
    static bool initialized = false;

    if( !initialized )
    {
        srand( value );
        initialized = true;
    }
}

// =============================================================================================================================================
void seedRand()
{
    quickdev::seedRand( time( NULL ) );
}

} // quickdev
