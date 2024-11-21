

# 6. RCC CORRECTION
add_executable(rmcl_tests_embree_correction_rcc correction_rcc.cpp)
target_link_libraries(rmcl_tests_embree_correction_rcc
    rmcl-embree
)

add_test(NAME embree_correction_rcc COMMAND rmcl_tests_embree_correction_rcc)
