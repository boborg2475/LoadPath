add_library(doctest INTERFACE)
target_include_directories(doctest INTERFACE ${CMAKE_SOURCE_DIR}/third_party/doctest)
target_compile_definitions(doctest INTERFACE DOCTEST_CONFIG_NO_EXCEPTIONS_BUT_WITH_ALL_ASSERTS)

add_executable(loadpath_tests
	TestMain.cpp
	SmokeTest.cpp
	HandlesTest.cpp
	ProfileTest.cpp
	BodyTest.cpp
	InterfaceTest.cpp
	StructureTest.cpp
	StructureFileTest.cpp
	CliTest.cpp
	StandingAuthorityTest.cpp
	SingleBlockAnchorsTest.cpp
	StackAnchorsTest.cpp
	StandingSolverTest.cpp
	StandingLocalityTest.cpp
	StaticsOracleTest.cpp
	CollapseCertificateTest.cpp
	CertificateAnchorsTest.cpp)
target_link_libraries(loadpath_tests PRIVATE loadpath doctest)

add_test(NAME loadpath_tests COMMAND loadpath_tests)
