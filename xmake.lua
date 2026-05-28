-- include subprojects
includes("lib/commonlibf4")

-- set project constants
set_project("FO4FasterHdtSMP")
set_version("0.0.0")
set_license("GPL-3.0")
set_languages("c++23")
set_warnings("allextra")

-- add common rules
add_rules("mode.debug", "mode.releasedbg")
add_rules("plugin.vsxmake.autoupdate")

add_defines("COMMONLIB_RUNTIMECOUNT=3")
if is_mode("debug") then
    add_requires("bullet3", {configs = {runtimes = "MDd", debug = true}})
else
    add_requires("bullet3", {configs = {runtimes = "MD"}})
end
add_requires("tbb")
add_requires("xbyak")
add_requires("tinyxml2")
add_requires("microsoft-detours")
add_requires("spdlog v1.16.0", {configs = {header_only = false, wchar = true, std_format = true}})

-- define targets
target("FO4FasterHdtSMP")
    add_rules("commonlibf4.plugin", {
        name = "FO4FasterHdtSMP",
        author = "Bingle",
        description = "Fallout 4 Faster HDT-SMP prototype using CommonLibF4",
        plugin_template = path.join(os.projectdir(), "res/commonlibf4-plugin.cpp.in"),
    })

    -- add src files
    add_files("src/**.cpp")
    add_headerfiles("src/**.h")
    add_includedirs("src")
    add_packages("bullet3", "tbb", "xbyak", "tinyxml2", "microsoft-detours", "spdlog")
    add_installfiles("res/configs.xml", "res/defaultBBPs.xml", "res/prototype-sample.xml", { prefixdir = "F4SE/Plugins/FO4FasterHdtSMP" })
    set_pcxxheader("src/pch.h")
