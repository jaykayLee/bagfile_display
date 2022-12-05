cmake_minimum_required(VERSION 3.12)

SET(ARCH "Unknown" CACHE STRING "in case of not S32R45, use a build option for host system")

message("--- Toolchain is ${CMAKE_TOOLCHAIN_FILE}")
message("--- Arch is ${ARCH}")

project(titrk)

IF (${ARCH} STREQUAL "S32R45")
    # set stuff for arm 
    add_definitions(-DS32R45)
ELSEIF (WIN32)
    # set stuff for windows
    add_definitions(-D_WIN32)
ELSE()
    # set stuff for other systems
    add_definitions(-D_LINUX)
ENDIF()

add_definitions(-DGTRACK_LOG_ENABLED)
#add_definitions(-DGTRACK_3D)

# add_executable(titrk	"${PROJECT_SOURCE_DIR}/src/gtrack_create.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_module.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_unit_event.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_unit_score.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_utilities_2d.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_delete.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_step.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_unit_get.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_unit_start.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_utilities_3d.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_listlib.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_unit_create.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_unit_predict.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_unit_stop.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_utilities.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_math.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_unit_delete.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_unit_report.c"
# 						"${PROJECT_SOURCE_DIR}/src/gtrack_unit_update.c"
# 						"${PROJECT_SOURCE_DIR}/gtrack_support.c"
# 						"${PROJECT_SOURCE_DIR}/gtracker.c"
# 						)

add_library(titrk SHARED "${PROJECT_SOURCE_DIR}/src/gtrack_create.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_module.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_unit_event.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_unit_score.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_utilities_2d.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_delete.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_step.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_unit_get.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_unit_start.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_utilities_3d.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_listlib.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_unit_create.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_unit_predict.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_unit_stop.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_utilities.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_math.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_unit_delete.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_unit_report.c"
						 "${PROJECT_SOURCE_DIR}/src/gtrack_unit_update.c"
						 "${PROJECT_SOURCE_DIR}/gtrack_support.c"
)

#set(TARGET_SHARED titrk)
target_include_directories(titrk PUBLIC 
						"${PROJECT_SOURCE_DIR}" 
						"${PROJECT_SOURCE_DIR}/include"
						)

IF (WIN32)
	# set stuff for windows
ELSE()
	# set stuff for other systems
	target_link_libraries(titrk PUBLIC m)
ENDIF()

# set the project name
#project(gtracker)
#add_executable(gtracker gtracker.c)
