project(App)

set( EXE_NAME BlockMatching )

find_package(OpenCV REQUIRED)

add_executable(${EXE_NAME} main.cpp)
include_directories(${Lib_SOURCE_DIR})
target_link_libraries( ${EXE_NAME} ${OpenCV_LIBS} Lib)

set_target_properties(${EXE_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
