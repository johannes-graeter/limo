include(FetchContent)
FetchContent_Declare(rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/download/0.28.1/rerun_cpp_sdk.zip
    URL_HASH SHA256=f46d3ff28f0f181c41c7fcb7ddcf7bbc6c3e8a138f153511a07d2b090940e60e
)
FetchContent_MakeAvailable(rerun_sdk)
