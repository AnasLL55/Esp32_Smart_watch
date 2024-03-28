idf_component_register(SRCS "i2c_example_main.c"  "LSM6DO_capteur.c" "LIS2MDL_Capteur.c" "./LSM6DSO/lsm6dso_reg.c" "./LIS2MDL/lis2mdl_reg.c"
                       INCLUDE_DIRS "." "./LSM6DSO" "./LIS2MDL")
