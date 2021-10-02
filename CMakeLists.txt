cmake_minimum_required(VERSION 3.10)

# set the project name
project(ifc_project CXX)

# find packages
find_package(OpenCASCADE REQUIRED)
find_package(Boost 1.77 REQUIRED)

# set location for IFCOpenShell
# set(IFC_INCLUDE_DIR "C:/Program Files/IfcOpenShell/IfcOpenShell-0.6.0/src")
# set(IFC_LIBRARY_DIR "C:/Program Files/IfcOpenShell/IfcOpenShell-0.6.0/_installed-vs2019-Win32/lib")

#set(IFC_INCLUDE_DIR "C:/Users/jaspe/Downloads/IfcOpenShell-0.6.0/src")
#set(IFC_LIBRARY_DIR "C:/Users/jaspe/Downloads/IfcOpenShell-0.6.0/_installed-vs2019-x64/lib")

set(IFC_INCLUDE_DIR "C:/Program Files/IfcOpenShell/IfcOpenShell-0.6.0/_installed-vs2019-x64/include")
set(IFC_LIBRARY_DIR "C:/Program Files/IfcOpenShell/IfcOpenShell-0.6.0/_installed-vs2019-x64/lib")

include_directories("/usr/local/include" ${OpenCASCADE_INCLUDE_DIR} ${Boost_INCLUDE_DIR} ${IFC_INCLUDE_DIR})
link_directories("/usr/local/lib" ${IFC_LIBRARY_DIR})

set(IFC_LIBRARY_NAMES IfcGeom IfcParse)
string(REGEX REPLACE "([^;]+)" "${IFC_LIBRARY_DIR}/\\1.lib" IFC_LIBRARIES "${IFC_LIBRARY_NAMES}")

set(OCC_LIBRARY_NAMES TKernel TKMath TKBRep TKGeomBase TKGeomAlgo TKG3d TKG2d TKShHealing TKTopAlgo TKMesh TKOffset
        TKPrim TKBool TKBO TKFillet TKSTEP TKSTEPBase TKSTEPAttr TKXSBase TKSTEP209 TKIGES TKHLR TKFeat)

add_executable(${PROJECT_NAME} 
main.cpp
)

foreach (LIB ${OCC_LIBRARY_NAMES})
  target_link_libraries(${PROJECT_NAME} ${OpenCASCADE_LIBRARY_DIR}/${LIB}.lib)
endforeach()

foreach(LIB ${IFC_LIBRARIES})
    message(STATUS "IFC libraries are ${LIB}")
endforeach()


foreach(LIB ${Boost_LIBRARIES})
    message(STATUS "Boost libraries are ${LIB}")
endforeach()

target_link_libraries(${PROJECT_NAME} ${IFC_LIBRARIES} ${Boost_LIBRARIES})


# Adjust runtime environment
set_property(TARGET ${PROJECT_NAME} PROPERTY VS_DEBUGGER_ENVIRONMENT "PATH=$<$<CONFIG:DEBUG>:${OpenCASCADE_BINARY_DIR}d>$<$<NOT:$<CONFIG:DEBUG>>:${OpenCASCADE_BINARY_DIR}>;%PATH%")