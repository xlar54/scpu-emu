/*
   SCPU-EMU - test runner.
*/
#include "test_framework.h"

int g_TestsRun = 0;
int g_TestsFailed = 0;
const char *g_CurrentTest = "";

#define MAX_TESTS 256
static const char *s_Names[ MAX_TESTS ];
static TestFn      s_Fns[ MAX_TESTS ];
static int         s_Count = 0;

void registerTest( const char *name, TestFn fn )
{
	if ( s_Count < MAX_TESTS )
	{
		s_Names[ s_Count ] = name;
		s_Fns[ s_Count ]   = fn;
		s_Count++;
	}
}

int runAllTests()
{
	std::printf( "SCPU-EMU host tests: %d test cases\n\n", s_Count );

	for ( int i = 0; i < s_Count; i++ )
	{
		int before = g_TestsFailed;
		g_CurrentTest = s_Names[ i ];
		std::printf( "%-44s", s_Names[ i ] );
		std::fflush( stdout );
		s_Fns[ i ]();
		std::printf( "%s\n", ( g_TestsFailed == before ) ? "ok" : "FAILED" );
	}

	std::printf( "\n%d checks, %d failed\n", g_TestsRun, g_TestsFailed );
	return g_TestsFailed ? 1 : 0;
}

int main()
{
	return runAllTests();
}
