# Add sources and include
set(SOURCE
	sPIDController.cpp
)

set(HEADER
    sPIDController.h
)

add_library(sPIDController STATIC
	${SOURCE}
    ${HEADER}
)

# Add gtests to project (Google Test framework for unit testing)
add_subdirectory(gtests)
