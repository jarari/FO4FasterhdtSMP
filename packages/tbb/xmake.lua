package("tbb")
    set_homepage("https://github.com/oneapi-src/oneTBB")
    set_description("Threading Building Blocks built static for FO4FasterHdtSMP.")
    set_license("Apache-2.0")

    add_urls("https://github.com/oneapi-src/oneTBB/archive/refs/tags/v$(version).tar.gz")
    add_versions("2022.1.0", "ed067603ece0dc832d2881ba5c516625ac2522c665d95f767ef6304e34f961b5")

    add_configs("shared", {description = "Build shared library.", default = false, type = "boolean", readonly = true})
    add_deps("cmake", "ninja")

    on_load(function (package)
        if package:has_tool("cxx", "cl", "clang_cl") then
            package:add("defines", "__TBB_NO_IMPLICIT_LINKAGE")
        end
        if package:is_debug() then
            package:add("links", "tbb_debug")
        else
            package:add("links", "tbb")
        end
    end)

    on_install("windows", function (package)
        local configs = {
            "-DTBB_TEST=OFF",
            "-DTBB_STRICT=OFF",
            "-DBUILD_SHARED_LIBS=OFF",
            "-DCMAKE_BUILD_TYPE=" .. (package:debug() and "Debug" or "Release")
        }
        import("package.tools.cmake").install(package, configs)
    end)

    on_test(function (package)
        assert(package:check_cxxsnippets({test = [[
            void test() {
                tbb::parallel_for(0, 1, [](int) {});
            }
        ]]}, {configs = {languages = "c++17"}, includes = "tbb/parallel_for.h"}))
    end)
