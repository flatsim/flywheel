#pragma once

#include "../core/common.h"
#include "../core/types.h"
#include "../math/math.h"

namespace flywheel
{

class Contact;

class PositionSolver
{
public:
    void Prepare(Contact* contact, int32 index);
    bool Solve();
    bool SolveTOI();

private:
    friend class Contact;
    friend class BlockSolver;

    Contact* contact;

    Vec2 localPlainPoint;
    Vec2 localClipPoint;
    Vec2 localNormal;
};

} // namespace flywheel
