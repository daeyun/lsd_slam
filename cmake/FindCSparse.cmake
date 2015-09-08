# CSPARSE_LIBRARIES
# CSPARSE_INCLUDE_DIR

# Look for csparse; note the difference in the directory specifications!
FIND_PATH(CSPARSE_INCLUDE_DIR NAMES cs.h
        PATHS
        /usr/include/suitesparse
        /usr/include
        /opt/local/include
        /usr/local/include
        /sw/include
        /usr/include/ufsparse
        /opt/local/include/ufsparse
        /usr/local/include/ufsparse
        /sw/include/ufsparse
)

FIND_LIBRARY(CSPARSE NAMES csparse
        PATHS
        /usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
)

FIND_LIBRARY(CXSPARSE NAMES cxsparse
        PATHS
        /usr/lib
        /usr/local/lib
        /opt/local/lib
        /sw/lib
)

SET(CSPARSE_LIBRARIES ${CSPARSE} ${CXSPARSE})
FIND_LIBRARY(CSPARSE_LIBRARIES ${CSPARSE_LIBRARIES})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CSPARSE DEFAULT_MSG
        CSPARSE_INCLUDE_DIR CSPARSE_LIBRARIES)

