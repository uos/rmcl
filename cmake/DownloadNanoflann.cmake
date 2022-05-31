set(NANOFLANN_DIR ${PROJECT_SOURCE_DIR}/ext/nanoflann)
ExternalProject_Add(nanoflann
                    PREFIX ${NANOFLANN_DIR}
                    GIT_REPOSITORY "https://github.com/jlblancoc/nanoflann.git"
                    GIT_TAG master
                    SOURCE_DIR ""
                    # Disable configure, build and install steps
                    CONFIGURE_COMMAND ""
                    UPDATE_COMMAND ""
                    BUILD_COMMAND ""
                    INSTALL_COMMAND "")

set(nanoflann_INCLUDE_DIR "ext/nanoflann/src/nanoflann/include")
set(nanoflann_INCLUDE_DIRS "ext/nanoflann/src/nanoflann/include")
# include_directories(ext/nanoflann/include)