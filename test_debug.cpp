#include <flywheel/world.h>
#include <iostream>

int main() {
    using namespace flywheel;
    
    // Create a simple world and test
    WorldSettings settings;
    settings.gravity = Vec2(0, -10);
    World world(settings);
    
    RigidBody* ground = world.CreateBox(10.0f, 1.0f, Transform(Vec2(0, -5), 0), RigidBody::Type::static_body);
    RigidBody* box = world.CreateBox(1.0f, 1.0f, Transform(Vec2(0, 5), 0), RigidBody::Type::dynamic_body);
    
    // Step the world many times
    float dt = 1.0f / 60.0f;
    for (int i = 0; i < 200; ++i) {
        world.Step(dt);
        if (i % 20 == 0 || world.GetContactCount() > 0) {
            std::cout << "Step " << i << ": contacts=" << world.GetContactCount() 
                      << " box.y=" << box->GetPosition().y << std::endl;
        }
        if (world.GetContactCount() > 0) {
            std::cout << "CONTACTS CREATED!" << std::endl;
            break;
        }
    }
    
    std::cout << "Final contact count: " << world.GetContactCount() << std::endl;
    std::cout << "Box position: " << box->GetPosition().x << ", " << box->GetPosition().y << std::endl;
    
    return 0;
}
