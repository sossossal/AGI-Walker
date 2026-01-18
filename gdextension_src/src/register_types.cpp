// register_types.cpp
// GDExtension 注册入口点

#include "register_types.h"
#include "enhanced_motor_joint.h"
#include "enhanced_physics_material.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_robot_sim_toolkit_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }

    // 注册增强电机关节类
    ClassDB::register_class<EnhancedMotorJoint>();
    
    // 注册增强物理材质类
    ClassDB::register_class<EnhancedPhysicsMaterial>();
    
    // 打印加载信息
    UtilityFunctions::print("✅ Robot Simulation Toolkit GDExtension loaded");
}

void uninitialize_robot_sim_toolkit_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
    
    UtilityFunctions::print("🔌 Robot Simulation Toolkit GDExtension unloaded");
}

extern "C" {
    // 初始化函数
    GDExtensionBool GDE_EXPORT robot_sim_toolkit_library_init(
        GDExtensionInterfaceGetProcAddress p_get_proc_address,
        GDExtensionClassLibraryPtr p_library,
        GDExtensionInitialization *r_initialization
    ) {
        godot::GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);

        init_obj.register_initializer(initialize_robot_sim_toolkit_module);
        init_obj.register_terminator(uninitialize_robot_sim_toolkit_module);
        init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);

        return init_obj.init();
    }
}
