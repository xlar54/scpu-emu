/*
   SCPU-EMU - minimal test framework.

   Deliberately tiny and dependency-free so the tests build with nothing but a
   host C++ compiler. No gtest, no CMake, no package manager.
*/
#ifndef _scpu_test_framework_h
#define _scpu_test_framework_h

#include <cstdio>
#include <cstring>

extern int g_TestsRun;
extern int g_TestsFailed;
extern const char *g_CurrentTest;

#define TEST( name )                                            \
	static void name();                                         \
	static struct name##_reg {                                  \
		name##_reg() { registerTest( #name, name ); }           \
	} name##_reg_inst;                                          \
	static void name()

typedef void (*TestFn)();
void registerTest( const char *name, TestFn fn );
int  runAllTests();

#define CHECK( cond )                                                       \
	do {                                                                    \
		g_TestsRun++;                                                       \
		if ( !( cond ) ) {                                                  \
			g_TestsFailed++;                                                \
			std::printf( "  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond ); \
		}                                                                   \
	} while ( 0 )

#define CHECK_EQ( actual, expected )                                        \
	do {                                                                    \
		g_TestsRun++;                                                       \
		long long _a = (long long)( actual );                               \
		long long _e = (long long)( expected );                             \
		if ( _a != _e ) {                                                   \
			g_TestsFailed++;                                                \
			std::printf( "  FAIL %s:%d  %s\n         got $%llX (%lld),"     \
			             " expected $%llX (%lld)\n",                        \
			             __FILE__, __LINE__, #actual, _a, _a, _e, _e );     \
		}                                                                   \
	} while ( 0 )

#endif
