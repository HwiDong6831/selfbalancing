# encoder 컴포넌트 — MT6701 자기 인코더 (ABZ 증분 출력, PCNT)
idf_component_register(SRCS "encoder.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver)
