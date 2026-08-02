# Findmujoco.cmake
# 查找 MuJoCo 库，支持多种安装方式：
#   - 系统安装（/usr/local, /usr）
#   - MUJOCO_DIR 环境变量
#   - CMAKE_PREFIX_PATH
#   - 用户目录 ~/.mujoco/mujoco-*
#   - macOS Homebrew
#
# 输出变量：
#   mujoco_FOUND        — 是否找到
#   mujoco_INCLUDE_DIR  — 头文件路径
#   mujoco_LIBRARY      — 库文件路径
#   mujoco_VERSION      — 版本号
#   mujoco::mujoco      — 导入目标

# ---- 搜索路径 ----
# 1. MUJOCO_DIR 环境变量（用户自定义安装路径）
if(DEFINED ENV{MUJOCO_DIR})
    list(APPEND __mujoco_hints "$ENV{MUJOCO_DIR}")
endif()

# 2. 常见系统安装路径
list(APPEND __mujoco_hints
    /usr/local
    /usr
    /opt/mujoco
    /opt/homebrew      # macOS Apple Silicon
    /usr/local/opt/mujoco  # macOS Intel Homebrew
)

# 3. 用户目录下的 MuJoCo 版本目录
file(GLOB __mujoco_user_dirs
    $ENV{HOME}/.mujoco/mujoco-*
)
list(APPEND __mujoco_hints ${__mujoco_user_dirs})

# 4. CMAKE_PREFIX_PATH
foreach(_prefix ${CMAKE_PREFIX_PATH})
    list(APPEND __mujoco_hints ${_prefix})
endforeach()

# ---- 查找头文件 ----
find_path(mujoco_INCLUDE_DIR
    NAMES mujoco/mujoco.h
    HINTS ${__mujoco_hints}
    PATH_SUFFIXES
        include
        Include
)

# ---- 查找库文件 ----
find_library(mujoco_LIBRARY
    NAMES mujoco libmujoco
    HINTS ${__mujoco_hints}
    PATH_SUFFIXES
        lib
        lib64
        lib/x86_64-linux-gnu
        Lib
)

# ---- 提取版本号（从文件名中的数字，如 libmujoco.so.3.9.0） ---
if(mujoco_LIBRARY)
    # 尝试从路径中提取版本号（如 .../mujoco-3.9.0/...）
    string(REGEX MATCH "mujoco-([0-9]+\\.[0-9]+\\.[0-9]+)" _ver_match "${mujoco_LIBRARY}")
    if(_ver_match)
        set(mujoco_VERSION "${CMAKE_MATCH_1}")
    else()
        # 尝试从 .so 文件名提取（如 libmujoco.so.3.9.0）
        get_filename_component(_ext "${mujoco_LIBRARY}" LAST_EXT)
        if(_ext MATCHES "^\\.[0-9]")
            string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" _ver "${_ext}")
            set(mujoco_VERSION "${_ver}")
        endif()
    endif()
endif()

# ---- 处理结果 ----
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(mujoco
    REQUIRED_VARS mujoco_LIBRARY mujoco_INCLUDE_DIR
    VERSION_VAR mujoco_VERSION
)

if(mujoco_FOUND AND NOT TARGET mujoco::mujoco)
    add_library(mujoco::mujoco UNKNOWN IMPORTED)
    set_target_properties(mujoco::mujoco PROPERTIES
        IMPORTED_LOCATION "${mujoco_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${mujoco_INCLUDE_DIR}"
    )
    message(STATUS "Found MuJoCo: ${mujoco_LIBRARY} (${mujoco_VERSION})")
endif()

mark_as_advanced(mujoco_INCLUDE_DIR mujoco_LIBRARY)
