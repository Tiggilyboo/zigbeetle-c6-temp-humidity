idf_component_register(SRCS "zigbeetle_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver nvs_flash ieee802154 ulp esp_timer)

set(ulp_app_name lp_core_${COMPONENT_NAME})
set(ulp_lp_core_sources "lp_core/main.c")
set(ulp_exp_dep_srcs "zigbeetle_main.c")
ulp_embed_binary(${ulp_app_name} "${ulp_lp_core_sources}" "${ulp_exp_dep_srcs}")
