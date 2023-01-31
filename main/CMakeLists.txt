#set(COMPONENT_REQUIRES esp_unit)
#idf_build_get_property(project_dir PROJECT_DIR)
#idf_component_register(SRCS "main.c" "led_strip_encoder.c" INCLUDE_DIRS "includes")
idf_component_register(SRCS "main.c" "led_strip_encoder.c"
                    INCLUDE_DIRS "" "includes"
                    REQUIRES soc ulp esp_moduleunit)

if(CONFIG_HARDWARE_MODEL_EPS32S3)

#
# ULP support additions to component CMakeLists.txt.
#
# 1. The ULP app name must be unique (if multiple components use ULP).
#set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_app_name ulp_main)
#
# 2. Specify all C and Assembly source files.
#    Files should be placed into a separate directory (in this case, ulp/),
#    which should not be added to COMPONENT_SRCS.
set(ulp_riscv_sources "ulp/main.c")

#
# 3. List all the component source files which include automatically
#    generated ULP export file, ${ulp_app_name}.h:
set(ulp_exp_dep_srcs "main.c")

#
# 4. Call function to build ULP binary and embed in project using the argument
#    values above.
ulp_embed_binary(${ulp_app_name} "${ulp_riscv_sources}" "${ulp_exp_dep_srcs}")

endif()