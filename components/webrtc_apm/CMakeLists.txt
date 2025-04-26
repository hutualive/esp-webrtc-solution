set(component_srcdirs "src")

# Gather our wrapper and all audio_processing module sources
file(GLOB_RECURSE APM_MODULE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/webrtc_src/modules/audio_processing/*.cc")
list(FILTER APM_MODULE_SRCS EXCLUDE REGEX ".*_unittest\\.cc$")
list(FILTER APM_MODULE_SRCS EXCLUDE REGEX ".*_integration_test\\.cc$")
list(FILTER APM_MODULE_SRCS EXCLUDE REGEX ".*/aec_dump/.*")
list(FILTER APM_MODULE_SRCS EXCLUDE REGEX ".*/logging/.*")
list(FILTER APM_MODULE_SRCS EXCLUDE REGEX ".*_avx2\\.cc$")
list(FILTER APM_MODULE_SRCS EXCLUDE REGEX ".*/aec3/.*")
list(FILTER APM_MODULE_SRCS EXCLUDE REGEX ".*_mips\\.cc$")
list(FILTER APM_MODULE_SRCS EXCLUDE REGEX ".*_neon\\.cc$")
set(COMPONENT_SRCS "src/webrtc_apm.cpp" ${APM_MODULE_SRCS})

idf_component_register(
    SRCS ${COMPONENT_SRCS}
    INCLUDE_DIRS "include"
                   "${CMAKE_CURRENT_SOURCE_DIR}/webrtc_src"
    PRIV_REQUIRES
)
