idf_component_register(
    SRCS            "GY95T.c"
    INCLUDE_DIRS    "include"
    REQUIRES driver freertos i2c_peripheral Remote_Motion
    PRIV_REQUIRES
)
