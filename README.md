## Introduction

Provided is a skeleton code to with main() function and forward kinematics of a planar 3 DOF robot arm. 
A visual depiction is available in the attached .pdf
Your mission is to implement the Inverse Kinematics function

## Instructions
1. Init an empty GIT repository and commit the provided files
2. Create a CMake project for the task
3. Implement the inverse kinematics function and describe your approach
4. Create approrpiate test(s) to show the correct behavior of the Inverse Kinematics solver
5. Optionally, add visualization to the solving mechanism

##Note: 
Please modify CMakeLists.txt for Eigen library as find_package command does not fetch the library files.  

![Cmake_Config](https://github.com/VP168/inverse_kinematics_solution/assets/71966193/1de22e31-b473-471d-816d-160e8f1d73de)

-If the library is already installed on the system, copy the directory path to line 7 in image above set(EIGEN_DIR "directory_path_here").

-If library is not on the system. download the library from https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip and extract to specific folder and provide the path as mentioned above.
