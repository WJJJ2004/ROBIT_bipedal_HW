cmake_minimum_required(VERSION 3.10)

# 프로젝트 이름 및 언어 설정
project(TransformMatrixEigen LANGUAGES CXX)

# C++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen 라이브러리 경로 설정 (시스템에 설치된 경우 기본 경로 사용)
find_package(Eigen3 3.3 REQUIRED)

# 실행 파일 생성
add_executable(transform main.cpp)

# Eigen 라이브러리 포함
target_include_directories(transform PRIVATE ${EIGEN3_INCLUDE_DIR})

