include(ExternalProject)

ExternalProject_Add(
  libmetal                 # Name for custom target
  SOURCE_DIR $ENV{ZEPHYR_BASE}/../libmetal/
  INSTALL_COMMAND ""      # This particular build system has no install command
  CMAKE_ARGS -DWITH_ZEPHYR=ON -DBOARD=${BOARD} -DWITH_DEFAULT_LOGGER=OFF
  )

ExternalProject_Get_property(libmetal BINARY_DIR)
set(LIBMETAL_INCLUDE_DIR ${BINARY_DIR}/lib/include)
set(LIBMETAL_LIBRARY     ${BINARY_DIR}/lib/libmetal.a)

ExternalProject_Add(
  open-amp
  SOURCE_DIR $ENV{ZEPHYR_BASE}/../open-amp/
  DEPENDS libmetal
  INSTALL_COMMAND ""      # This particular build system has no install command
  CMAKE_ARGS -DWITH_ZEPHYR=ON -DWITH_PROXY=OFF -DBOARD=${BOARD} -DLIBMETAL_INCLUDE_DIR=${LIBMETAL_INCLUDE_DIR} -DLIBMETAL_LIB=${LIBMETAL_LIBRARY}
)

ExternalProject_Get_property(open-amp SOURCE_DIR)
set(OPENAMP_INCLUDE_DIR  ${SOURCE_DIR}/lib/include CACHE PATH "Path to the OpenAMP header files")
ExternalProject_Get_property(open-amp BINARY_DIR)
set(OPENAMP_LIBRARY      ${BINARY_DIR}/lib/libopen_amp.a CACHE FILEPATH "Path to the OpenAMP library")

