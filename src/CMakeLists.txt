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
    reson7k_output.c
    wbms_data.c
    3dss_dx.c
    sim_data.c
    velodyne.c
    time_functions.c
    linalg.c
    raytracing.c
    posmv.c
    xtf_nav.c
    wbm_tool_nav.c
    sim_nav.c
    sbet_nav.c
    eelume_sbd_nav.c
    nmea_nav.c
    proj_wrapper.c
    georef_tools.c
    sbf_output.c
    nmea_output.c
    json_output.c
    csv_output.c
    bin_output.c
    intensity_scaling.c
    misc.c
    navdata_abstractor.c
    sensordata_abstractor.c
    svpdata.c
    gsf_wrapper.c
    gsf.c  
    gsf_compress.c  
    gsf_dec.c  
    gsf_enc.c  
    gsf_geo.c  
    gsf_indx.c  
    gsf_info.c
  )

# wbms_georef_identify
list(APPEND wbms_georef_identify_SOURCES
    wbms_georef_identify.c
    crc32.c
    cmath.c
    reson7k.c
    reson7k_output.c
    wbms_data.c
    3dss_dx.c
    sim_data.c
    velodyne.c
    time_functions.c
    linalg.c
    raytracing.c
    posmv.c
    xtf_nav.c
    wbm_tool_nav.c
    sim_nav.c
    sbet_nav.c
    eelume_sbd_nav.c
    nmea_nav.c
    proj_wrapper.c
    georef_tools.c
    sbf_output.c
    nmea_output.c
    json_output.c
    csv_output.c
    bin_output.c
    intensity_scaling.c
    misc.c
    navdata_abstractor.c
    sensordata_abstractor.c
    svpdata.c
    gsf_wrapper.c
    gsf.c  
    gsf_compress.c  
    gsf_dec.c  
    gsf_enc.c  
    gsf_geo.c  
    gsf_indx.c  
    gsf_info.c
  )

# sv2csv
list(APPEND wbms_georef_sv2csv_SOURCES
    wbms_georef_sv2csv.c
    cmath.c
    time_functions.c
    misc.c
    svpdata.c
  )

if(WIN32 OR CYGWIN)
    list(APPEND wbms_georef_SOURCES
      non_posix.c
    )
    list(APPEND wbms_georef_identify_SOURCES
      non_posix.c
    )
endif()


add_executable(wbms_georef
  ${wbms_georef_SOURCES}
  )

add_executable(wbms_georef_identify
  ${wbms_georef_identify_SOURCES}
  )

add_executable(wbms_georef_sv2csv
  ${wbms_georef_sv2csv_SOURCES}
  )

if(WIN32 OR CYGWIN)
  target_link_libraries(wbms_georef
    ${GEOTRANZ_LIBRARIES}
    ${PROJ_LIBRARY}
    ${XGETOPT_LIBRARIES}
    )
  target_link_libraries(wbms_georef_identify
    ${GEOTRANZ_LIBRARIES}
    ${PROJ_LIBRARY}
    ${XGETOPT_LIBRARIES}
    )
  target_link_libraries(wbms_georef_sv2csv
    ${GEOTRANZ_LIBRARIES}
    ${XGETOPT_LIBRARIES}
    )
else()
  target_link_libraries(wbms_georef
    ${GEOTRANZ_LIBRARIES}
    ${PROJ_LIBRARY}
    )
  target_link_libraries(wbms_georef_identify
    ${GEOTRANZ_LIBRARIES}
    ${PROJ_LIBRARY}
    )
  target_link_libraries(wbms_georef_sv2csv
    ${GEOTRANZ_LIBRARIES}
    )
endif()

if(WIN32 OR CYGWIN)
  set_target_properties(wbms_georef
    PROPERTIES COMPILE_FLAGS "-DUSE_REGEX_STATIC"
    )
  target_link_libraries(wbms_georef winmm ws2_32 setupapi)
endif()


install(TARGETS wbms_georef DESTINATION ${INSTALL_BIN_DIR})
install(TARGETS wbms_georef_identify DESTINATION ${INSTALL_BIN_DIR})
install(TARGETS wbms_georef_sv2csv DESTINATION ${INSTALL_BIN_DIR})
