#pragma once

#include "CoreMinimal.h"
#include "MotionEngineSolver.h"
#include "Eigen/Dense"

DECLARE_LOG_CATEGORY_EXTERN(ChainExponentialSolverLog, All, All);

namespace MotionEngine
{

class ArticulatedElement;
class Chain;

class ChainExponentialSolver : public MotionEngineSolver<Chain>
{
    public:
        ChainExponentialSolver();
        virtual ~ChainExponentialSolver();

        virtual bool InitializeSolver(TSharedPtr<Chain> MObjectChain) override;

    private:
        TSharedPtr<Chain> SerialChain;
};

} // namespace MotionEngine
