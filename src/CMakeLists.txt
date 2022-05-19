if(WIN32 OR CYGWIN)
   include_directories(
     ${CUSTOM_GEOTRANZ_DIR}
     ${CUSTOM_XGETOPT_DIR}
     )
else()
  include_directories(
   ${CUSTOM_GEOTRANZ_DIR}
    )
endif()

# wbms_georef
list(APPEND wbms_georef_SOURCES
    wbms_georef.c
    crc32.c
    cmath.c
    reson7k.c
    wbms_data.c
    velodyne.c
    time_functions.c
    linalg.c
    raytracing.c
    posmv.c
    xtf_nav.c
    wbm_tool_nav.c
    sim_nav.c
    sbet_nav.c
    nmea_nav.c
    proj_wrapper.c
    georef_tools.c
    sbf_output.c
    csv_output.c
    bin_output.c
    intensity_scaling.c
  )
if(WIN32 OR CYGWIN)
    list(APPEND wbms_georef_SOURCES
      non_posix.c
    )
endif()

add_executable(wbms_georef
  ${wbms_georef_SOURCES}
  )

if(WIN32 OR CYGWIN)
  target_link_libraries(wbms_georef
    ${GEOTRANZ_LIBRARIES}
    ${PROJ_LIBRARY}
    ${XGETOPT_LIBRARIES}
    )
else()
  target_link_libraries(wbms_georef
    ${GEOTRANZ_LIBRARIES}
    ${PROJ_LIBRARY}
    )
endif()

if(WIN32 OR CYGWIN)
  set_target_properties(wbms_georef
    PROPERTIES COMPILE_FLAGS "-DUSE_REGEX_STATIC"
    )
  target_link_libraries(wbms_georef winmm ws2_32 setupapi)
endif()


install(TARGETS wbms_georef DESTINATION ${INSTALL_BIN_DIR})
