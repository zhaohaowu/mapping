
INSTALL(DIRECTORY conf #data  dag launch #data/perception/parking/parking_slam/etc
        DESTINATION ${CMAKE_INSTALL_PREFIX}/runtime_service/hz_mapping)
INSTALL(FILES scripts/hz_mapping_start.sh
        DESTINATION ${CMAKE_INSTALL_PREFIX}/script/test/)

set(AP_RELEASE_HOST_0DPMACHINE
        ${CMAKE_SOURCE_DIR}/depend/ap-release/outputcfg/Host0DpMachine/hz_mapping)
install(DIRECTORY
        ${AP_RELEASE_HOST_0DPMACHINE}/hz_mappingProcess
        DESTINATION ${CMAKE_INSTALL_PREFIX}/runtime_service/hz_mapping/etc)
install(FILES ${AP_RELEASE_HOST_0DPMACHINE}/MANIFEST.json
        DESTINATION ${CMAKE_INSTALL_PREFIX}/runtime_service/hz_mapping/etc)
