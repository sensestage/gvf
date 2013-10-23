message( "=== gvf src ===" )


# message( "hidapi parser source dir is: ${hidapi_SOURCE_DIR}" )
# 
# set( hidapi_parser_INCLUDE_DIRS ${hidapi_SOURCE_DIR}/hidapi_parser/)
# # set( hidapi_LIBS ${UDEV_LIBRARIES})
# set( hidapi_parser_SRCS hidapi_parser.c )
# 
# message( "hidapi_parser include dirs are: ${hidapi_parser_INCLUDE_DIRS}" )

include_directories( ${gvf_SOURCE_DIR}/Eigen/ )
add_library( gvf STATIC GestureVariationFollower.cpp )
# target_link_libraries( gvf )