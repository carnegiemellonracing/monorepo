# Get the right gitcommit
if(CMAKE_HOST_WIN32)
    add_custom_target(generate_gitcommit ALL
        COMMAND powershell -ExecutionPolicy Bypass -File ${CMAKE_CURRENT_SOURCE_DIR}/export_git.ps1
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating gitcommit.h using PowerShell"
    )
elseif(CMAKE_HOST_UNIX)
    add_custom_target(generate_gitcommit ALL
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/export_git.sh
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        COMMENT "Generating gitcommit.h on Linux/Unix"
    )
endif()