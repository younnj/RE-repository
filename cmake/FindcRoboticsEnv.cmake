# Detect Conda environment path dynamically (cross-platform, robust)
execute_process(
    COMMAND conda env list
    OUTPUT_VARIABLE CONDA_ENV_LIST_RAW
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

# Normalize line endings and split to lines
string(REPLACE "\r" "" CONDA_ENV_LIST "${CONDA_ENV_LIST_RAW}")
string(REGEX REPLACE "\n" ";" CONDA_ENV_LINES "${CONDA_ENV_LIST}")

# Debug: print raw
message(STATUS "Raw output of 'conda env list':\n${CONDA_ENV_LIST_RAW}")

unset(CONDA_ENV_PATH)

# Prefer explicit environment name match (cRobotics), grab last whitespace-separated token as path
foreach(line IN LISTS CONDA_ENV_LINES)
    # skip comments/headers and empty lines
    if(line MATCHES "^#|^$") 
        continue()
    endif()

    # match lines that contain the env name
    if(line MATCHES "(^|[[:space:]])cRobotics([[:space:]]|$)")
        # remove leading markers like '*' and extra spaces
        string(REGEX REPLACE "^[[:space:]]*\\*?[[:space:]]*" "" cleaned "${line}")
        # collapse multiple spaces to semicolons -> tokens
        string(REGEX REPLACE "[[:space:]]+" ";" tokens "${cleaned}")
        # take the last token as the candidate path
        list(GET tokens -1 maybe_path)

        # On Windows, backslashes are fine; EXISTS works cross-platform
        if(EXISTS "${maybe_path}")
            set(CONDA_ENV_PATH "${maybe_path}")
            break()
        endif()
    endif()
endforeach()

# Fallbacks: try CONDA_PREFIX if running inside the env; else try typical locations
if(NOT CONDA_ENV_PATH)
    if(DEFINED ENV{CONDA_PREFIX} AND EXISTS "$ENV{CONDA_PREFIX}")
        # ensure it's the cRobotics env, not just base
        get_filename_component(_envname "$ENV{CONDA_PREFIX}" NAME)
        if("${_envname}" STREQUAL "cRobotics")
            set(CONDA_ENV_PATH "$ENV{CONDA_PREFIX}")
        endif()
    endif()
endif()

# Finalize or fail
if(CONDA_ENV_PATH)
    set(CONDA_ENV_PATH "${CONDA_ENV_PATH}" CACHE PATH "Path to the cRobotics Conda environment")
    message(STATUS "Detected Conda environment path: ${CONDA_ENV_PATH}")
else()
    message(STATUS "Raw output of 'conda env list':\n${CONDA_ENV_LIST_RAW}")
    message(FATAL_ERROR "cRobotics environment not found. Please ensure the environment is created.")
endif()
