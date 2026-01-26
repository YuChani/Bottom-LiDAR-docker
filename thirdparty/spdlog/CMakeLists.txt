cmake_minimum_required(VERSION 3.10)
# 1. "spdlog_custom"이라는 이름의 'IMPORTED' STATIC 라이브러리 타겟 생성
add_library(static_spdlog STATIC IMPORTED GLOBAL)
# 2. 이 타겟의 속성(Properties)을 설정
set_target_properties(static_spdlog PROPERTIES
    # 실제 .a 파일 위치 알려주기
    IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/libspdlog.a"
    # 1. 헤더 경로
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
    # 2. 컴파일 정의
    INTERFACE_COMPILE_DEFINITIONS "SPDLOG_COMPILED_LIB"

    INTERFACE_LINK_LIBRARIES "pthread"
)