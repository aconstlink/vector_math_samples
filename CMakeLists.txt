cmake_minimum_required( VERSION 3.6 )
project( "vector_math_samples" )

set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build )
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build )

add_subdirectory( natus )
list( APPEND CMAKE_MODULE_PATH 
  "${CMAKE_CURRENT_LIST_DIR}/natus/cmake/macros" )

include( visual_studio_generate_bat_dll )
include( emacs_default_directory )

natus_emacs_default_directory( "${CMAKE_CURRENT_LIST_DIR}/natus" )

#set( CMAKE_CXX_STANDARD 17 )

set( subdirs
    "vector_05_orthogonalization"
    "collision_01_point_circle"
    )

foreach( _subdir ${subdirs} )

  set( app_name "${_subdir}" )
  add_subdirectory( ${_subdir} )
  set_target_properties( ${app_name} PROPERTIES FOLDER "examples" )
endforeach()


natus_vs_gen_bat_for_dlls()

