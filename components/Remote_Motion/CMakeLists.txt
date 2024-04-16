idf_component_register(
    SRCS            "Remote_Motion.c"
    INCLUDE_DIRS    "include"
    REQUIRES driver nvs_flash freertos struct_component  
    PRIV_REQUIRES arduino 
)

#add_definitions(-DC3_TX)
add_definitions(-DC3_RX)