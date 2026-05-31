package("bullet3-hdt")
    set_homepage("http://bulletphysics.org")
    set_description("Bullet Physics SDK with hdtSMP Bullet patches.")
    set_license("zlib")

    set_urls("https://github.com/bulletphysics/bullet3/archive/$(version).zip",
             "https://github.com/bulletphysics/bullet3.git")
    add_versions("3.25", "b9bc8d1443637a9084e2b585ed582abf2da3ddad7d768acccfe4ee17aca56bf7")

    add_configs("double_precision", {description = "Enable double precision floats", default = false, type = "boolean"})
    add_configs("extras",           {description = "Build the extras", default = false, type = "boolean"})
    add_configs("non_hookean",      {description = "Apply hdtSMP non-Hookean generic spring patches.", default = true, type = "boolean"})
    add_configs("sse_in_api",       {description = "Build Bullet with BT_USE_SSE_IN_API.", default = true, type = "boolean"})
    add_configs("profile_gate",     {description = "Build BT_PROFILE support behind the hdtSMP runtime gate.", default = true, type = "boolean"})
    add_configs("opencl",           {description = "Build Bullet3OpenCL_clew.", default = false, type = "boolean"})
    add_configs("multithreading",   {description = "Build Bullet internal multithreading with oneTBB.", default = true, type = "boolean"})

    if is_plat("windows", "mingw") then
        add_configs("shared", {description = "Build shared library.", default = false, type = "boolean", readonly = true})
    end

    add_deps("cmake", "tbb")

    on_load(function (package)
        package:add("links",
            "Bullet2FileLoader",
            "Bullet3Collision",
            "Bullet3Common",
            "Bullet3Dynamics",
            "Bullet3Geometry",
            "BulletDynamics",
            "BulletCollision",
            "BulletInverseDynamics",
            "BulletSoftBody",
            "LinearMath")
        if package:config("opencl") then
            package:add("links", "Bullet3OpenCL_clew")
        end
        package:add("includedirs", "include", "include/bullet")
        if package:config("sse_in_api") then
            package:add("defines", "BT_USE_SSE_IN_API")
        end
        if package:config("profile_gate") then
            package:add("defines", "BT_ENABLE_PROFILE")
        end
    end)

    on_install(function (package)
        local function replace_exact(file, old, new)
            local content = io.readfile(file):gsub("\r\n", "\n")
            local start_index, end_index = content:find(old, 1, true)
            assert(start_index, "missing Bullet patch anchor in " .. file)
            io.writefile(file, content:sub(1, start_index - 1) .. new .. content:sub(end_index + 1))
        end

        local function replace_exact_once(file, marker, old, new)
            local content = io.readfile(file):gsub("\r\n", "\n")
            if content:find(marker, 1, true) then
                io.writefile(file, content)
                return
            end
            local start_index, end_index = content:find(old, 1, true)
            assert(start_index, "missing Bullet patch anchor in " .. file)
            io.writefile(file, content:sub(1, start_index - 1) .. new .. content:sub(end_index + 1))
        end

        local function patch_non_hookean()
            local header = path.join("src", "BulletDynamics", "ConstraintSolver", "btGeneric6DofSpring2Constraint.h")
            replace_exact_once(header,
                "btScalar m_nonHookeanDamping;",
                "btScalar m_currentPosition;\n\tint m_currentLimit;",
                "btScalar m_currentPosition;\n\tint m_currentLimit;\n\n\tbtScalar m_nonHookeanDamping;\n\tbtScalar m_nonHookeanStiffness;")
            replace_exact_once(header,
                "m_nonHookeanDamping = 0.f;",
                "m_currentPosition = 0;\n\t\tm_currentLimit = 0;",
                "m_currentPosition = 0;\n\t\tm_currentLimit = 0;\n\n\t\tm_nonHookeanDamping = 0.f;\n\t\tm_nonHookeanStiffness = 0.f;")
            replace_exact_once(header,
                "m_nonHookeanDamping = limot.m_nonHookeanDamping;",
                "m_currentPosition = limot.m_currentPosition;\n\t\tm_currentLimit = limot.m_currentLimit;",
                "m_currentPosition = limot.m_currentPosition;\n\t\tm_currentLimit = limot.m_currentLimit;\n\n\t\tm_nonHookeanDamping = limot.m_nonHookeanDamping;\n\t\tm_nonHookeanStiffness = limot.m_nonHookeanStiffness;")
            replace_exact_once(header,
                "btVector3 m_nonHookeanDamping;",
                "btVector3 m_currentLinearDiff;\n\tint m_currentLimit[3];",
                "btVector3 m_currentLinearDiff;\n\tint m_currentLimit[3];\n\n\tbtVector3 m_nonHookeanDamping;\n\tbtVector3 m_nonHookeanStiffness;")
            replace_exact_once(header,
                "m_nonHookeanDamping[i] = btScalar(0.f);",
                "m_currentLimit[i] = 0;",
                "m_currentLimit[i] = 0;\n\n\t\t\tm_nonHookeanDamping[i] = btScalar(0.f);\n\t\t\tm_nonHookeanStiffness[i] = btScalar(0.f);")
            replace_exact_once(header,
                "m_nonHookeanDamping[i] = other.m_nonHookeanDamping[i];",
                "m_currentLimit[i] = other.m_currentLimit[i];",
                "m_currentLimit[i] = other.m_currentLimit[i];\n\n\t\t\tm_nonHookeanDamping[i] = other.m_nonHookeanDamping[i];\n\t\t\tm_nonHookeanStiffness[i] = other.m_nonHookeanStiffness[i];")
            replace_exact_once(header,
                "void setNonHookeanDamping(int index, btScalar factor);",
                "void setStiffness(int index, btScalar stiffness, bool limitIfNeeded = true);  // if limitIfNeeded is true the system will automatically limit the stiffness in necessary situations where otherwise the spring would move unrealistically too widely\n\tvoid setDamping",
                "void setStiffness(int index, btScalar stiffness, bool limitIfNeeded = true);  // if limitIfNeeded is true the system will automatically limit the stiffness in necessary situations where otherwise the spring would move unrealistically too widely\n\tvoid setNonHookeanDamping(int index, btScalar factor);\n\tvoid setNonHookeanStiffness(int index, btScalar factor);\n\tvoid setDamping")

            local cpp = path.join("src", "BulletDynamics", "ConstraintSolver", "btGeneric6DofSpring2Constraint.cpp")
            replace_exact_once(cpp,
                "limot.m_nonHookeanDamping = m_linearLimits.m_nonHookeanDamping[i];",
                "limot.m_motorERP = (flags & BT_6DOF_FLAGS_ERP_MOTO2) ? m_linearLimits.m_motorERP[i] : info->erp;\n\n\t\t\t//rotAllowed",
                "limot.m_motorERP = (flags & BT_6DOF_FLAGS_ERP_MOTO2) ? m_linearLimits.m_motorERP[i] : info->erp;\n\n\t\t\tlimot.m_nonHookeanDamping = m_linearLimits.m_nonHookeanDamping[i];\n\t\t\tlimot.m_nonHookeanStiffness = m_linearLimits.m_nonHookeanStiffness[i];\n\n\t\t\t//rotAllowed")
            replace_exact_once(cpp,
                "btScalar dampingFactor = limot->m_nonHookeanDamping;",
                "\t\tbtScalar cfm = BT_ZERO;",
                "\t\tbtScalar dampingFactor = limot->m_nonHookeanDamping;\n\t\tbtScalar stiffnessFactor = limot->m_nonHookeanStiffness;\n\t\tif (!btFuzzyZero(error) && (!btFuzzyZero(dampingFactor) || !btFuzzyZero(stiffnessFactor)))\n\t\t{\n\t\t\tbtScalar range = limot->m_currentPosition < limot->m_equilibriumPoint ? limot->m_equilibriumPoint - limot->m_loLimit : limot->m_hiLimit - limot->m_equilibriumPoint;\n\t\t\tbtScalar rf = !btFuzzyZero(range) ? btFabs(error) / range : BT_ZERO;\n\t\t\tbtScalar t = btClamped(rf, BT_ZERO, BT_ONE);\n\n\t\t\tkd *= BT_ONE - (dampingFactor * t);\n\t\t\tks *= BT_ONE - (stiffnessFactor * t);\n\t\t}\n\n\t\tbtScalar cfm = BT_ZERO;")
            replace_exact_once(cpp,
                "void btGeneric6DofSpring2Constraint::setNonHookeanDamping",
                "}\n}\n\nvoid btGeneric6DofSpring2Constraint::setDamping(int index, btScalar damping, bool limitIfNeeded)",
                "}\n}\n\nvoid btGeneric6DofSpring2Constraint::setNonHookeanDamping(int index, btScalar damping)\n{\n\tbtAssert((index >= 0) && (index < 6));\n\tif (index < 3)\n\t{\n\t\tm_linearLimits.m_nonHookeanDamping[index] = damping;\n\t}\n\telse\n\t{\n\t\tm_angularLimits[index - 3].m_nonHookeanDamping = damping;\n\t}\n}\n\nvoid btGeneric6DofSpring2Constraint::setNonHookeanStiffness(int index, btScalar damping)\n{\n\tbtAssert((index >= 0) && (index < 6));\n\tif (index < 3)\n\t{\n\t\tm_linearLimits.m_nonHookeanStiffness[index] = damping;\n\t}\n\telse\n\t{\n\t\tm_angularLimits[index - 3].m_nonHookeanStiffness = damping;\n\t}\n}\n\nvoid btGeneric6DofSpring2Constraint::setDamping(int index, btScalar damping, bool limitIfNeeded)")
        end

        local function patch_opencl_toggle()
            replace_exact_once(path.join("src", "Bullet3OpenCL", "CMakeLists.txt"),
                "option(BUILD_OPENCL \"Build Bullet3OpenCL_clew\" ON)",
                "INCLUDE_DIRECTORIES( ${BULLET_PHYSICS_SOURCE_DIR}/src  )",
                "option(BUILD_OPENCL \"Build Bullet3OpenCL_clew\" ON)\nif(NOT BUILD_OPENCL)\n\treturn()\nendif()\n\nINCLUDE_DIRECTORIES( ${BULLET_PHYSICS_SOURCE_DIR}/src  )")
        end

        local function patch_onetbb()
            local file = path.join("src", "LinearMath", "btThreads.cpp")
            replace_exact_once(file, "#include <tbb/global_control.h>", "#include <tbb/task_scheduler_init.h>\n", "#include <tbb/global_control.h>\n#include <tbb/info.h>\n")
            replace_exact_once(file, "tbb::global_control* m_tbbSchedulerInit;", "tbb::task_scheduler_init* m_tbbSchedulerInit;", "tbb::global_control* m_tbbSchedulerInit;")
            replace_exact_once(file, "return tbb::info::default_concurrency();", "return tbb::task_scheduler_init::default_num_threads();", "return tbb::info::default_concurrency();")
            replace_exact_once(file, "m_tbbSchedulerInit = new tbb::global_control", "m_tbbSchedulerInit = new tbb::task_scheduler_init(m_numThreads);", "m_tbbSchedulerInit = new tbb::global_control(tbb::global_control::max_allowed_parallelism, m_numThreads);")
        end

        local function patch_profile_gate()
            local header = path.join("src", "LinearMath", "btQuickprof.h")
            replace_exact_once(header,
                "extern bool gBtProfileEnabled;",
                "void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc);\n\n#ifndef BT_ENABLE_PROFILE",
                "void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc);\n\nvoid btEnterProfileZone(const char* name);\nvoid btLeaveProfileZone();\n\nextern bool gBtProfileEnabled;\nvoid btSetProfileEnabled(bool enabled);\nbool btGetProfileEnabled();\nSIMD_FORCE_INLINE bool btIsProfileEnabled()\n{\n\treturn gBtProfileEnabled;\n}\n\n#ifndef BT_ENABLE_PROFILE")
            replace_exact_once(header,
                "m_enabled(btIsProfileEnabled())",
                "class CProfileSample\n{\npublic:\n\tCProfileSample(const char* name);\n\n\t~CProfileSample(void);\n};",
                "class CProfileSample\n{\npublic:\n\tCProfileSample(const char* name) :\n\t\tm_enabled(btIsProfileEnabled())\n\t{\n\t\tif (m_enabled)\n\t\t{\n\t\t\tbtEnterProfileZone(name);\n\t\t}\n\t}\n\n\t~CProfileSample(void)\n\t{\n\t\tif (m_enabled)\n\t\t{\n\t\t\tbtLeaveProfileZone();\n\t\t}\n\t}\n\nprivate:\n\tbool m_enabled;\n};")

            local cpp = path.join("src", "LinearMath", "btQuickprof.cpp")
            replace_exact_once(cpp,
                "bool gBtProfileEnabled = false;",
                "static btEnterProfileZoneFunc* bts_enterFunc = btEnterProfileZoneDefault;\nstatic btLeaveProfileZoneFunc* bts_leaveFunc = btLeaveProfileZoneDefault;",
                "bool gBtProfileEnabled = false;\n\nstatic btEnterProfileZoneFunc* bts_enterFunc = btEnterProfileZoneDefault;\nstatic btLeaveProfileZoneFunc* bts_leaveFunc = btLeaveProfileZoneDefault;")
            replace_exact_once(cpp,
                "void btSetProfileEnabled(bool enabled)",
                "void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc)\n{\n\tbts_leaveFunc = leaveFunc;\n}\n\nCProfileSample::CProfileSample(const char* name)\n{\n\tbtEnterProfileZone(name);\n}\n\nCProfileSample::~CProfileSample(void)\n{\n\tbtLeaveProfileZone();\n}\n",
                "void btSetCustomLeaveProfileZoneFunc(btLeaveProfileZoneFunc* leaveFunc)\n{\n\tbts_leaveFunc = leaveFunc;\n}\n\nvoid btSetProfileEnabled(bool enabled)\n{\n\tgBtProfileEnabled = enabled;\n}\n\nbool btGetProfileEnabled()\n{\n\treturn gBtProfileEnabled;\n}\n")
        end

        replace_exact_once("CMakeLists.txt", "cmake_minimum_required(VERSION 3.11)", "cmake_minimum_required(VERSION 2.4.3)", "cmake_minimum_required(VERSION 3.11)")

        if package:config("non_hookean") then
            patch_non_hookean()
        end
        patch_opencl_toggle()
        if package:config("multithreading") then
            patch_onetbb()
        end
        if package:config("profile_gate") then
            patch_profile_gate()
        end

        os.rm(path.join("examples", "ThirdPartyLibs"))

        local configs = {
            "-DBUILD_CPU_DEMOS=OFF",
            "-DBUILD_OPENGL3_DEMOS=OFF",
            "-DBUILD_BULLET2_DEMOS=OFF",
            "-DBUILD_UNIT_TESTS=OFF",
            "-DINSTALL_LIBS=ON",
            "-DCMAKE_DEBUG_POSTFIX=",
            "-DBUILD_OPENCL=" .. (package:config("opencl") and "ON" or "OFF")
        }
        table.insert(configs, "-DCMAKE_BUILD_TYPE=" .. (package:debug() and "Debug" or "Release"))
        table.insert(configs, "-DBUILD_SHARED_LIBS=" .. (package:config("shared") and "ON" or "OFF"))
        table.insert(configs, "-DUSE_DOUBLE_PRECISION=" .. (package:config("double_precision") and "ON" or "OFF"))
        table.insert(configs, "-DBUILD_EXTRAS=" .. (package:config("extras") and "ON" or "OFF"))
        table.insert(configs, "-DUSE_MSVC_RUNTIME_LIBRARY_DLL=ON")
        if package:is_plat("windows") and not package:config("vs_runtime"):endswith("d") then
            table.insert(configs, "-DUSE_MSVC_RELEASE_RUNTIME_ALWAYS=ON")
        end
        local compile_defines = {}
        if package:config("sse_in_api") then
            table.insert(compile_defines, "/DBT_USE_SSE_IN_API")
        end
        if package:config("profile_gate") then
            table.insert(compile_defines, "/DBT_ENABLE_PROFILE")
        end
        if #compile_defines > 0 then
            local flags = table.concat(compile_defines, " ")
            table.insert(configs, "-DCMAKE_C_FLAGS=" .. flags)
            table.insert(configs, "-DCMAKE_CXX_FLAGS=" .. flags)
        end
        if package:config("multithreading") then
            local tbb = package:dep("tbb")
            local tbb_inc = tbb:installdir("include")
            local tbb_lib = tbb:installdir("lib")
            table.insert(configs, "-DBULLET2_MULTITHREADING=ON")
            table.insert(configs, "-DBULLET2_USE_TBB_MULTITHREADING=ON")
            table.insert(configs, "-DBULLET2_TBB_INCLUDE_DIR=" .. tbb_inc)
            table.insert(configs, "-DBULLET2_TBB_LIB_DIR=" .. tbb_lib)
            table.insert(configs, "-DTBB_LIBRARY=" .. path.join(tbb_lib, package:debug() and "tbb_debug.lib" or "tbb.lib"))
            table.insert(configs, "-DTBBMALLOC_LIBRARY=" .. path.join(tbb_lib, package:debug() and "tbbmalloc_debug.lib" or "tbbmalloc.lib"))
        end

        import("package.tools.cmake").install(package, configs)
    end)

    on_test(function (package)
        assert(package:check_cxxsnippets({test = [[
            void test() {
                btDefaultCollisionConfiguration collisionConfiguration;
                btCollisionDispatcher dispatcher(&collisionConfiguration);
                btDbvtBroadphase broadphase;
                btSequentialImpulseConstraintSolver constraintSolver;
                btDiscreteDynamicsWorld dynamicWorld(&dispatcher, &broadphase, &constraintSolver, &collisionConfiguration);
                dynamicWorld.setGravity(btVector3(0, -10, 0));
                btGeneric6DofSpring2Constraint* constraint = nullptr;
                (void)constraint;
            }
        ]]}, {includes = "bullet/btBulletDynamicsCommon.h"}))
    end)
