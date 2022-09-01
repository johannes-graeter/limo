set(MRT_PKG_VERSION 2.2)
# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(demo_keyframe_bundle_adjustment_meta)

###################
## find packages ##
###################
find_package(mrt_cmake_modules REQUIRED)
include(UseMrtStdCompilerFlags)
include(UseMrtAutoTarget)

include(GatherDeps)
# Remove libs which cannot be found automatically
#list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})

# Manually resolve removed dependend packages
#find_package(...)

# Mark files or folders for display in IDEs
mrt_add_to_ide(README.md .gitlab-ci.yml)

########################
## add python modules ##
########################
# This adds a python module if located under src/{PROJECT_NAME)
mrt_python_module_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# Add message, service and action files
glob_ros_files(add_message_files msg)
glob_ros_files(add_service_files srv)
glob_ros_files(add_action_files action)

# Generate added messages and services with any dependencies listed here
if (ROS_GENERATE_MESSAGES)
    generate_messages(
        DEPENDENCIES
        #add dependencies here
        #std_msgs
        )
endif()

# Generate dynamic reconfigure options
file(GLOB PARAMS_FILES RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "cfg/*.params" "cfg/*.cfg" "cfg/*.mrtcfg" "cfg/*.rosif")
if (PARAMS_FILES)
    generate_ros_parameter_files(${PARAMS_FILES})
    generate_ros_interface_files(${PARAMS_FILES})
endif ()

###################################
## catkin specific configuration ##
###################################
catkin_package(
    )

###########
## Build ##
###########
# Add include and library directories
include_directories(
    ${mrt_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

link_directories(
    ${mrt_LIBRARY_DIRS}
    )

add_executable(main_program ${CMAKE_CURRENT_SOURCE_DIR}/apps/main_program.cpp)

glob_folders(SRC_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/src")
if (SRC_DIRECTORIES)
    # Found subfolders, add executable for each subfolder
    foreach(SRC_DIR ${SRC_DIRECTORIES})
        mrt_add_node_and_nodelet(${SRC_DIR} FOLDER "src/${SRC_DIR}")
    endforeach()
else()
    # No subfolder found, add executable and python modules for src folder
    mrt_add_node_and_nodelet(${PROJECT_NAME} FOLDER "src")
endif()

#############
## Install ##
#############
# Install all targets, headers by default and scripts and other files if specified (folders or files)
mrt_install(PROGRAMS scripts FILES launch rviz maps res data nodelet_plugins.xml plugin_description.xml)

#############
## Testing ##
#############
# Add test targets for cpp and python tests
if (CATKIN_ENABLE_TESTING)
    # Include header in src folder via src/<pkgname>/<myheader>.h
    include_directories("src")
    mrt_add_ros_tests(test)
    mrt_add_nosetests(test)
endif()
