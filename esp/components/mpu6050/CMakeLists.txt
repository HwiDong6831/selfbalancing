# mpu6050 컴포넌트 빌드 규칙
#  - SRCS        : 컴파일할 .c
#  - INCLUDE_DIRS: 이 컴포넌트를 쓰는 쪽에 노출할 헤더 경로 (include/mpu6050.h)
#  - REQUIRES    : 공개 헤더(mpu6050.h)가 레거시 i2c 타입을 쓰므로 driver 는 PUBLIC 의존
idf_component_register(SRCS "mpu6050.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver)
