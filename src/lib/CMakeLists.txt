

include_directories(/usr/include/boost)
find_package(Boost COMPONENTS graph serialization REQUIRED)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

ADD_LIBRARY(floorplan SHARED
GraphStatistics.cpp
GraphDatabase.cpp
GraphFileOperations.hpp
GraphUtils.hpp
FloorPlanGraph.hpp)

SET_TARGET_PROPERTIES(floorplan PROPERTIES LINKER_LANGUAGE C)
TARGET_LINK_LIBRARIES(floorplan 
    ${Boost_GRAPH_LIBRARY}  
    ${Boost_SERIALIZATION_LIBRARY}
    ${OpenCV_LIBS}
)

install(TARGETS floorplan DESTINATION ${OUTPUT})
install(TARGETS floorplan DESTINATION lib)
install(DIRECTORY . DESTINATION include
                  FILES_MATCHING PATTERN "*.hpp")

