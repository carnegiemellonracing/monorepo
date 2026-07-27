# Expects: SRC_DIR, IN_FILE, OUT_FILE, USER_VALUE_ABC to be passed in via -D

find_package(Git QUIET)

set(_hash "00000000")
set(_dirty FALSE)

if(GIT_EXECUTABLE)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short=8 HEAD
        WORKING_DIRECTORY ${SRC_DIR}
        OUTPUT_VARIABLE _hash_out
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE _rc
        ERROR_QUIET
    )
    if(_rc EQUAL 0 AND _hash_out)
        set(_hash ${_hash_out})
    endif()

    # --porcelain catches staged, unstaged, AND untracked changes,
    # with no need to `git add` or commit anything.
    execute_process(
        COMMAND ${GIT_EXECUTABLE} status --porcelain
        WORKING_DIRECTORY ${SRC_DIR}
        OUTPUT_VARIABLE _status_out
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(NOT "${_status_out}" STREQUAL "")
        set(_dirty TRUE)
    endif()
else()
    message(WARNING "Git not found - USER_VALUE will be a fallback constant")
endif()

# Turn the 8 hex chars into a uint32_t. If the tree is dirty, flip the
# top bit as a "dirty" marker so the value still visibly differs.
math(EXPR USER_VALUE "0x${_hash}" OUTPUT_FORMAT DECIMAL)
if(_dirty)
    math(EXPR USER_VALUE "${USER_VALUE} ^ 0x80000000")
endif()

configure_file(${IN_FILE} ${OUT_FILE} @ONLY)