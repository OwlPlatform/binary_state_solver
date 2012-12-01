SET(SourceFiles
  binary_state_solver.cpp
)

add_executable (binary_state_solver ${SourceFiles})
target_link_libraries (binary_state_solver owl-common owl-solver pthread)

INSTALL(TARGETS binary_state_solver RUNTIME DESTINATION bin/owl)
