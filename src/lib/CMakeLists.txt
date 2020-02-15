project(Lib)
set(LIB_NAME Lib)

find_package(OpenCV REQUIRED)

file(GLOB BASE_SRC_FILES "*.cpp")
file(GLOB BASE_INC_FILES "*.h")

set(SRC_FILES ${BASE_SRC_FILES})
set(INC_FILES ${BASE_INC_FILES})

add_library(${LIB_NAME} STATIC ${SRC_FILES} ${INC_FILES})
target_link_libraries(${LIB_NAME} ${OpenCV_LIBS})
