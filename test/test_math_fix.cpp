// Quick test to debug the issues
#include <flywheel/math.h>
#include <iostream>

using namespace flywheel;

int main() {
    // Test Mat2 multiplication
    Mat2 a(Vec2(1.0f, 2.0f), Vec2(3.0f, 4.0f));
    Mat2 b(Vec2(5.0f, 6.0f), Vec2(7.0f, 8.0f));
    Mat2 c = a * b;
    
    std::cout << "Mat2 multiplication result:\n";
    std::cout << "c.ex.x = " << c.ex.x << " (expected 19)\n";
    std::cout << "c.ex.y = " << c.ex.y << " (expected 22)\n";
    std::cout << "c.ey.x = " << c.ey.x << " (expected 43)\n";
    std::cout << "c.ey.y = " << c.ey.y << " (expected 50)\n";
    
    // Test Rotation composition
    Rotation r1(pi / 4.0f);
    Rotation r2(pi / 4.0f);
    Rotation r3 = Mul(r1, r2);
    
    std::cout << "\nRotation composition:\n";
    std::cout << "r1: s=" << r1.s << ", c=" << r1.c << "\n";
    std::cout << "r2: s=" << r2.s << ", c=" << r2.c << "\n";
    std::cout << "r3: s=" << r3.s << ", c=" << r3.c << "\n";
    std::cout << "r3 angle = " << r3.GetAngle() << " (expected " << (pi/2.0f) << ")\n";
    
    return 0;
}
