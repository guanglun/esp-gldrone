idf_component_register(SRCS "ak8963.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES i2c_bus 
                       PRIV_REQUIRES hal utils platform)