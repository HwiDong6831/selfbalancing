# voting 컴포넌트. 순수 C 라 ESP-IDF 의존이 없다 (test/ 참조).
idf_component_register(SRCS "voting.c"
                       INCLUDE_DIRS ".")
