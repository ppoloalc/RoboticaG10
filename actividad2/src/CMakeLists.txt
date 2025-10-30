# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
        ../src/hungarian.cpp
        ../src/ransac_line_detector.cpp
        ../src/room_detector.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/common_types.h
  ../src/hungarian.h
  ../src/ransac_line_detector.h
  ../src/room_detector.h
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    list(APPEND LIBS ${OpenCV_LIBS})
endif()

LIST(APPEND LIBS ${LIBS} tbb)

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-fmax-errors=1 -fno-char8_t)

