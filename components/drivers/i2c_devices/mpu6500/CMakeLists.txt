idf_component_register(SRCS "mpu6500.c"
                       INCLUDE_DIRS "include"
                        REQUIRES i2c_bus
                        PRIV_REQUIRES utils platform config)