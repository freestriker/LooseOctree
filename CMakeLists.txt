cmake_minimum_required (VERSION 3.25)

if (POLICY CMP0091)
  cmake_policy(SET CMP0091 NEW)
endif (POLICY CMP0091)

if (POLICY CMP0140)
  cmake_policy(SET CMP0140 NEW)
endif (POLICY CMP0140)

set(CMAKE_TOOLCHAIN_FILE $ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake)
set(VCPKG_TARGET_TRIPLET x64-windows-static)

function(GROUP_FILES FILE_PATHS)
  message(STATUS "-----GROUP_FILES: START-----")
  message(STATUS "-----CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}-----")
  message(STATUS "-----foreach: START-----")
  foreach(FILE_PATH ${FILE_PATHS}) 
    get_filename_component(ABSOLUTE_FILE_PATH "${FILE_PATH}" ABSOLUTE)
    message(STATUS "-----ABSOLUTE_FILE_PATH: ${ABSOLUTE_FILE_PATH}-----")
    get_filename_component(ABSOLUTE_FILE_DIRECTORY_PATH "${ABSOLUTE_FILE_PATH}" DIRECTORY)
    message(STATUS "-----ABSOLUTE_FILE_DIRECTORY_PATH: ${ABSOLUTE_FILE_DIRECTORY_PATH}-----")
    string(REPLACE "${CMAKE_CURRENT_SOURCE_DIR}" "" FILE_GROUP "${ABSOLUTE_FILE_DIRECTORY_PATH}")
    message(STATUS "-----FILE_GROUP: ${FILE_GROUP}-----")
    source_group("${FILE_GROUP}" FILES "${FILE_PATH}")
  endforeach()
  message(STATUS "-----endforeach: END-----")
  message(STATUS "-----GROUP_FILES: END-----")
endfunction()

function(FIND_SOURCE_FILE_PATHS SOURCE_DIRECTORY_PATH RESULT_SOURCE_FILE_PATHS)
  message(STATUS "-----FIND_SOURCE_FILE_PATHS: START-----")
  file(GLOB_RECURSE ALL_HEADER_FILE_PATHS "${SOURCE_DIRECTORY_PATH}/*.hpp")
  message(STATUS "-----ALL_HEADER_FILE_PATHS: ${ALL_HEADER_FILE_PATHS}-----")
  file(GLOB_RECURSE ALL_SOURCE_FILE_PATHS "${SOURCE_DIRECTORY_PATH}/*.cpp")
  message(STATUS "-----ALL_SOURCE_FILE_PATHS: ${ALL_SOURCE_FILE_PATHS}-----")
  set(${RESULT_SOURCE_FILE_PATHS} ${ALL_HEADER_FILE_PATHS} ${ALL_SOURCE_FILE_PATHS})
  message(STATUS "-----RESULT_SOURCE_FILE_PATHS: ${RESULT_SOURCE_FILE_PATHS}-----")
  message(STATUS "-----FIND_SOURCE_FILE_PATHS: END-----")
  return(PROPAGATE ${RESULT_SOURCE_FILE_PATHS})
endfunction()

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /SUBSYSTEM:CONSOLE")

add_definitions(-DNOMINMAX)

project(LooseOctree)

message(STATUS "-----PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR}-----")
# FIND_SOURCE_FILE_PATHS(${PROJECT_SOURCE_DIR} LOOSE_OCTREE_SOURCE_FILE_PATHS)
set(LOOSE_OCTREE_SOURCE_FILE_PATHS "main.cpp" "LooseOctree.hpp")
message(STATUS "-----LOOSE_OCTREE_SOURCE_FILE_PATHS: ${LOOSE_OCTREE_SOURCE_FILE_PATHS}-----")
GROUP_FILES("${LOOSE_OCTREE_SOURCE_FILE_PATHS}")

set(ALL_SOURCE_FILE_PATHS ${LOOSE_OCTREE_SOURCE_FILE_PATHS})
message(STATUS "-----ALL_SOURCE_FILE_PATHS: ${ALL_SOURCE_FILE_PATHS}-----")

add_executable(Test)
target_sources(Test PRIVATE "${ALL_SOURCE_FILE_PATHS}")

find_package(glm CONFIG REQUIRED)
target_link_libraries(Test PUBLIC glm::glm)

# find_package(Boost REQUIRED COMPONENTS type-traits)
# target_link_libraries(Test PRIVATE Boost::boost Boost::type-traits)
# find_package(Boost COMPONENTS type-traits REQUIRED)
# target_link_libraries(Test PUBLIC Boost::boost Boost::type-traits)
find_package(Boost REQUIRED)
target_link_libraries(Test PRIVATE Boost::boost)
    
set_property(TARGET Test PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")