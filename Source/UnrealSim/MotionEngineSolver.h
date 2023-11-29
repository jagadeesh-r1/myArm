#pragma once

#include "CoreMinimal.h"

namespace MotionEngine
{

class MotionObject;

template <class T>
class MotionEngineSolver
{
    public:
        MotionEngineSolver()
        {

        }

        virtual ~MotionEngineSolver()
        {

        }

        virtual bool InitializeSolver(TSharedPtr<T> MObject)
        {
            return false;
        }
};

}