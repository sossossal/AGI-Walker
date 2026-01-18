// enhanced_physics_material.h
// 增强物理材质 - 添加滚动摩擦等高级特性

#ifndef ENHANCED_PHYSICS_MATERIAL_H
#define ENHANCED_PHYSICS_MATERIAL_H

#include <godot_cpp/classes/physics_material.hpp>

namespace godot {

class EnhancedPhysicsMaterial : public PhysicsMaterial {
    GDCLASS(EnhancedPhysicsMaterial, PhysicsMaterial)

private:
    float rolling_friction = 0.01f;      // 滚动摩擦系数
    float spinning_friction = 0.01f;     // 旋转摩擦系数
    float surface_roughness = 0.5f;      // 表面粗糙度 (0=光滑, 1=粗糙)

protected:
    static void _bind_methods();

public:
    EnhancedPhysicsMaterial();
    ~EnhancedPhysicsMaterial();
    
    void set_rolling_friction(float friction) { rolling_friction = friction; }
    float get_rolling_friction() const { return rolling_friction; }
    
    void set_spinning_friction(float friction) { spinning_friction = friction; }
    float get_spinning_friction() const { return spinning_friction; }
    
    void set_surface_roughness(float roughness) { surface_roughness = roughness; }
    float get_surface_roughness() const { return surface_roughness; }
};

} // namespace godot

#endif // ENHANCED_PHYSICS_MATERIAL_H
