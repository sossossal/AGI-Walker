// enhanced_physics_material.cpp
#include "enhanced_physics_material.h"
#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void EnhancedPhysicsMaterial::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_rolling_friction", "friction"), 
                        &EnhancedPhysicsMaterial::set_rolling_friction);
    ClassDB::bind_method(D_METHOD("get_rolling_friction"), 
                        &EnhancedPhysicsMaterial::get_rolling_friction);
    
    ClassDB::bind_method(D_METHOD("set_spinning_friction", "friction"), 
                        &EnhancedPhysicsMaterial::set_spinning_friction);
    ClassDB::bind_method(D_METHOD("get_spinning_friction"), 
                        &EnhancedPhysicsMaterial::get_spinning_friction);
    
    ClassDB::bind_method(D_METHOD("set_surface_roughness", "roughness"), 
                        &EnhancedPhysicsMaterial::set_surface_roughness);
    ClassDB::bind_method(D_METHOD("get_surface_roughness"), 
                        &EnhancedPhysicsMaterial::get_surface_roughness);
    
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "rolling_friction", PROPERTY_HINT_RANGE, "0,1,0.01"), 
                 "set_rolling_friction", "get_rolling_friction");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spinning_friction", PROPERTY_HINT_RANGE, "0,1,0.01"), 
                 "set_spinning_friction", "get_spinning_friction");
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "surface_roughness", PROPERTY_HINT_RANGE, "0,1,0.01"), 
                 "set_surface_roughness", "get_surface_roughness");
}

EnhancedPhysicsMaterial::EnhancedPhysicsMaterial() {
}

EnhancedPhysicsMaterial::~EnhancedPhysicsMaterial() {
}
