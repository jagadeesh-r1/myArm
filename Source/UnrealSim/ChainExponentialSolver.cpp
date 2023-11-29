#include "ChainExponentialSolver.h"
#include "Chain.h"
#include "ArticulatedElement.h"
#include "Joint.h"
#include "Link.h"

DEFINE_LOG_CATEGORY(ChainExponentialSolverLog);

namespace MotionEngine
{

ChainExponentialSolver::ChainExponentialSolver()
{

}

ChainExponentialSolver::~ChainExponentialSolver()
{

}

bool ChainExponentialSolver::InitializeSolver(TSharedPtr<Chain> MObjectChain)
{
    return false;
}

} // namespace MotionEngine
