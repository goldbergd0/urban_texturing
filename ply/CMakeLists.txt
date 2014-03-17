set(SUBSYS_NAME framework_io_ply)
set(SUBSYS_DEPS framework_types_point_cloud)

set(LIB_NAME dpf_${SUBSYS_NAME})

foreach(TMP_LIB ${SUBSYS_DEPS})
  set(LINK_LIBS ${LINK_LIBS} dpf_${TMP_LIB})
endforeach()

set(srcs ply_writer.cc)
set(hdrs ply_writer.h)

if (PCL_FOUND)
  set(srcs ${srcs} pcl_ply_reader.cc)
  set(hdrs ${hdrs} pcl_ply_reader.h)

  set(LINK_LIBS ${LINK_LIBS} ${PCL_COMMON_LIBRARY} ${PCL_IO_LIBRARY})
  add_definitions("-DPCL_FOUND")
  include_directories(${PCL_INCLUDE_DIRS})
endif()

add_library(${LIB_NAME} ${srcs} ${hdrs})
target_link_libraries(${LIB_NAME} ${LINK_LIBS})
