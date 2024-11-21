if(TARGET rmcl-embree)
message(STATUS "Building tests for Embree backend")

# ADD TESTS FOR EMBREE HERE
add_subdirectory(embree)

endif(embree_FOUND)
