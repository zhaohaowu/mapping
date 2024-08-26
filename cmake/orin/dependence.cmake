set(BAIDU_LD_LIBS_PATH ${THIRDPARTY_ROOT_PATH}/third_party/${LOCAL_TARGET_PLATFORM}/baiduhdmap/lib)

file(GLOB LD_BAIDU_LIBS "${BAIDU_LD_LIBS_PATH}/lib*.so")

set(DEPENDS_LIBS
  protobuf
  ${LD_BAIDU_LIBS}
  # neta_phm
)