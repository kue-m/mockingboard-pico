# Generate mb_version_gen.h: the VERSION file refined with git state (tag -> bare, else +hash, +"-dirty").

set(base "0.0.0")
if(EXISTS ${VERSION_FILE})
    file(READ ${VERSION_FILE} base)
    string(STRIP "${base}" base)
endif()
set(ver "${base}")

if(GIT_EXECUTABLE)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${SRC_DIR}
        OUTPUT_VARIABLE hash OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET RESULT_VARIABLE rc_hash)
    if(rc_hash EQUAL 0 AND NOT hash STREQUAL "")
        execute_process(
            COMMAND ${GIT_EXECUTABLE} describe --exact-match --match "v${base}" HEAD
            WORKING_DIRECTORY ${SRC_DIR}
            OUTPUT_QUIET ERROR_QUIET RESULT_VARIABLE rc_tag)
        if(rc_tag EQUAL 0)
            set(ver "${base}")
        else()
            set(ver "${base}+${hash}")
        endif()
        # refresh the index first so a fresh checkout's new mtimes don't read as modified
        execute_process(COMMAND ${GIT_EXECUTABLE} update-index -q --refresh
            WORKING_DIRECTORY ${SRC_DIR} OUTPUT_QUIET ERROR_QUIET)
        execute_process(
            COMMAND ${GIT_EXECUTABLE} diff-index --quiet HEAD --
            WORKING_DIRECTORY ${SRC_DIR}
            RESULT_VARIABLE rc_dirty)
        if(NOT rc_dirty EQUAL 0)
            set(ver "${ver}-dirty")
        endif()
    endif()
endif()

set(content "#pragma once\n#define MB_VERSION_STR \"${ver}\"\n")

# rewrite only on change, so an unchanged version doesn't force a relink
set(existing "")
if(EXISTS ${OUT})
    file(READ ${OUT} existing)
endif()
if(NOT existing STREQUAL content)
    file(WRITE ${OUT} "${content}")
    message(STATUS "mb_version: ${ver}")
endif()
