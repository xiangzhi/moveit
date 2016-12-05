/**
The objective function for human aware planner
**/

#ifndef HUMAN_AWARE_OBJECTIVE
#define HUMAN_AWARE_OBJECTIVE

#include "ompl/base/objectives/StateCostIntegralObjective.h"
#include <iostream>

namespace ompl_interface
{
	class HumanAwareObjective : public ompl::base::StateCostIntegralObjective
	{
	public:	
		HumanAwareObjective(const ompl::base::SpaceInformationPtr &si);
   		ompl::base::Cost stateCost(const ompl::base::State *s) const override;
   		ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override;
   		//ompl::base::Cost motionCostHeuristic(const ompl::base::State *s1, const ompl::base::State *s2) const override;
	};
}


#endif