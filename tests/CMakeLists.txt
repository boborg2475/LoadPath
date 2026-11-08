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
	Slice2bWedgeArchAnchorsTest.cpp
	StandingSolverTest.cpp
	StandingLocalityTest.cpp
	StandingUngroundedTest.cpp
	StaticsOracleTest.cpp
	StructuralAuthorityTest.cpp
	ElasticAuthorityTest.cpp
	ArchAnchorsTest.cpp
	InterfaceStateTest.cpp
	CrackedJointTest.cpp
	CrushingTest.cpp
	CascadeTest.cpp
	CatalogueTest.cpp
	CatalogueAnchorsTest.cpp
	GeometryContactTest.cpp
	GeometryWallTest.cpp
	GeometryAnchorsTest.cpp
	DryCornerNoiseFloorTest.cpp
	StandingFieldPropertyTest.cpp
	StandingLateralDryWallTest.cpp
	StandingLateralLimeWallTest.cpp
	CascadeFuzzTest.cpp
	CascadeFloatingWallTest.cpp
	GeometryWarehouseTest.cpp
	ScaleMeasurementTest.cpp
	FactorTest.cpp
	SparseTest.cpp
	ComponentResultsTest.cpp
	EventRegionTest.cpp
	CascadeScanTest.cpp
	FactorPanelBoundaryTest.cpp
	Slice2bAnchorsTest.cpp
	Slice4AnchorsTest.cpp
	ConcurrencyTest.cpp
	ParallelSolveTest.cpp
	StaleLoadTest.cpp
	ProfileTimingsTest.cpp
	EndToEndTest.cpp
	ScenarioTest.cpp
	ScenarioFilesTest.cpp
	VisualSupportTest.cpp
	SettingsTest.cpp
	PassRecorderTest.cpp
	CrushingRubbleTest.cpp)
target_link_libraries(loadpath_tests PRIVATE loadpath doctest)
target_include_directories(loadpath_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_include_directories(loadpath_tests SYSTEM PRIVATE ${CMAKE_SOURCE_DIR}/third_party/eigen)
target_compile_definitions(loadpath_tests PRIVATE EIGEN_MPL2_ONLY)
target_compile_definitions(loadpath_tests PRIVATE LOADPATH_CATALOGUE_PATH="${CMAKE_SOURCE_DIR}/data/catalogue.json")
target_compile_definitions(loadpath_tests PRIVATE LOADPATH_DATA_PATH="${CMAKE_SOURCE_DIR}/data")
target_compile_definitions(loadpath_tests PRIVATE LOADPATH_SCENARIO_DIR="${CMAKE_SOURCE_DIR}/data/scenarios")

add_test(NAME loadpath_tests COMMAND loadpath_tests)
