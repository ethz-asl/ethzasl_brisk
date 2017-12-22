set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_OPENCV -std=c++0x")

# Detect the preprocessor directives which are set by the compiler.
execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dM -E -x c /dev/null
                OUTPUT_VARIABLE PREPROCESSOR_DIRECTIVES)

set(IS_SSE_ENABLED FALSE)
if (PREPROCESSOR_DIRECTIVES MATCHES "__SSE2__")
  add_definitions(-mssse3)
  set(IS_SSE_ENABLED TRUE)
# For both armv7 and armv8, __ARM_NEON is used as preprocessor directive.
elseif (PREPROCESSOR_DIRECTIVES MATCHES "__ARM_ARCH 7")
  add_definitions(-mfpu=neon) # Needs to be set for armv7.
elseif (PREPROCESSOR_DIRECTIVES MATCHES "__ARM_ARCH 8")
  # Add other potential compiler flags here.
endif()
