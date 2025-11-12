# Sources set
LIST(APPEND SOURCES
  ../src/specificworker.cpp
        ../src/hungarian.cpp
        ../src/ransac_line_detector.cpp
        ../src/room_detector.cpp
        ../src/time_series_plotter.cpp
        ../src/door_detector.cpp
        ../src/qcustomplot.cpp
)

# Headers set
LIST(APPEND HEADERS
  ../src/specificworker.h
  ../src/common_types.h
  ../src/hungarian.h
  ../src/ransac_line_detector.h
  ../src/room_detector.h
  ../src/time_series_plotter.h
        ../src/door_detector.h
        ../src/image_processor.h
        ../src/nominal_room.h
        ../src/qcustomplot.h
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)
if(OpenCV_FOUND)
    include_directories(${OpenCV_INCLUDE_DIRS})
    list(APPEND LIBS ${OpenCV_LIBS})
endif()

LIST(APPEND LIBS ${LIBS} tbb)
SET(LIBS ${LIBS} tbb Qt6PrintSupport)

# no-char8_t to avoid a problem with ICE Connection lib.
add_definitions(-O4 -fmax-errors=1 -fno-char8_t)

