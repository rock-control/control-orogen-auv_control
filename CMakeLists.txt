PROJECT(auv_control)
cmake_minimum_required(VERSION 2.6)

SET (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/.orogen/config")
INCLUDE(auv_controlBase)

if (ROCK_TEST_ENABLED)
    enable_testing()
    find_package(Rock REQUIRED)
    include(RockRuby)
    rock_ruby_test(test WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()

# FIND_PACKAGE(KDL)
# FIND_PACKAGE(OCL)

