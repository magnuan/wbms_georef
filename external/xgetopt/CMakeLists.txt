set(XGETOPT_LIBRARIES xgetopt CACHE INTERNAL "xgetopt")

list(APPEND xgetopt_SOURCES
  xgetopt.c
  )

add_library(xgetopt STATIC
  ${xgetopt_SOURCES}
  )
