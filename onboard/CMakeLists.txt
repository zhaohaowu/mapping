project(mapping_onboard)

if(MIDDLEWARE MATCHES CYBER)
   add_subdirectory(onboard_cyber)
elseif(MIDDLEWARE MATCHES LITE)
  add_subdirectory(onboard_lite)
elseif(MIDDLEWARE MATCHES ORIN)
  add_subdirectory(onboard_lite)
elseif(MIDDLEWARE MATCHES ADF)
  add_subdirectory(onboard_adc)
endif()

