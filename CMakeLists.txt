
cmake_minimum_required (VERSION 3.8)

project ("InverseKinematicsSolution")

# set EIGEN_DIR variable to Eigen Library Path
set(EIGEN_DIR "C:\\toolbox\\eigen-3.4.0")

#find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${EIGEN_DIR})

# Add source to this project's executable.
add_executable (InverseKinematicsSolution "task.cpp")

add_subdirectory(InverseKinematicSolutionTest)

target_link_libraries (InverseKinematicsSolution ${EIGEN_DIR})

if(NOT googletest)
	include(FetchContent)
	FetchContent_Declare(
		googletest
		# Specify the commit you depend on and update it regularly.
		URL https://github.com/google/googletest/archive/refs/heads/main.zip
	)
	# For Windows: Prevent overriding the parent project's compiler/linker settings
	set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
	FetchContent_MakeAvailable(googletest)
endif()