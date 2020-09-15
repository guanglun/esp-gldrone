idf_component_register(SRCS "sbus.c"
                       INCLUDE_DIRS "include"
                       REQUIRES utils platform 
                       PRIV_REQUIRES config)