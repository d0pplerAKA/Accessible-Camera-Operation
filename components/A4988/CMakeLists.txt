idf_component_register(
    SRCS            "A4988.c"
    INCLUDE_DIRS    "include"
    REQUIRES driver freertos Remote_Motion
    PRIV_REQUIRES
)

