### COMPILE OPTIX KERNEL ###
# Problem: nvcc cannot compile optix launches in kernels
# but we can precompile to ptx files + include the strings in optix programs
# Next problem: pathes to ptx files differ for installation and raw build
# Solution:
# 1. Compile to ptx 
# Here:
# 2. Copy contents of ptx to temporary header file with R("...")
# 3. Include temporary header file static into a char* variable

# message(STATUS "Precomputing ${OPTIX_KERNEL_FILES}")


string(REPLACE " " ";" OPTIX_KERNEL_FILES "${OPTIX_KERNEL_FILES}")

# message(STATUS "Capsule: ${OPTIX_KERNEL_FILES}")

# Capsule PTX-Strings in files
foreach(OPTIX_KERNEL_FILE ${OPTIX_KERNEL_FILES})
    # message(STATUS "Precomputing ${OPTIX_KERNEL_FILE}")
    # Get Name of Kernel
    get_filename_component(OPTIX_KERNEL_NAME ${OPTIX_KERNEL_FILE} NAME_WLE)
    # Read Compiled Kernel to String
    file(READ "${RMCL_OPTIX_PTX_DIR}/cuda_compile_ptx_1_generated_${OPTIX_KERNEL_NAME}.cu.ptx" INCLUDE_STRING)
    # Write to static readable file e.g. R("")
    configure_file(${RMCL_SOURCE_DIR}/cmake/FileToString.h.in "include/kernels/${OPTIX_KERNEL_NAME}String.h")
    message(STATUS "Preprocessed ${OPTIX_KERNEL_NAME}")
endforeach()
