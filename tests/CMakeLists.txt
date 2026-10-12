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
	CertificateAnchorsTest.cpp
	CertificateShearCapTest.cpp
	StructuralAuthorityTest.cpp
	ArchAnchorsTest.cpp
	InterfaceStateTest.cpp
	CrackedJointTest.cpp
	CascadeTest.cpp
	CatalogueTest.cpp
	CatalogueAnchorsTest.cpp
	GeometryContactTest.cpp
	GeometryWallTest.cpp
	GeometryAnchorsTest.cpp
	DryCornerNoiseFloorTest.cpp
	CascadeFuzzTest.cpp
	CascadeFloatingWallTest.cpp
	GeometryWarehouseTest.cpp
	ScaleMeasurementTest.cpp
	FactorTest.cpp
	SparseTest.cpp
	ComponentResultsTest.cpp
	CertificateSparseTest.cpp
	CertificateStallTest.cpp
	EventRegionTest.cpp
	CertificateIterationsTest.cpp
	CapSiteFactorTest.cpp)
target_link_libraries(loadpath_tests PRIVATE loadpath doctest)
target_include_directories(loadpath_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(loadpath_tests SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party/eigen)
target_compile_definitions(loadpath_tests PRIVATE EIGEN_MPL2_ONLY)
target_compile_definitions(loadpath_tests PRIVATE LOADPATH_CATALOGUE_PATH="${CMAKE_SOURCE_DIR}/data/catalogue.json")

add_test(NAME loadpath_tests COMMAND loadpath_tests)
