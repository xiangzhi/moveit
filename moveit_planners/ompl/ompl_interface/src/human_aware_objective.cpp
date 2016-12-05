
#include <moveit/ompl_interface/human_aware_objective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ros/console.h>

ompl_interface::HumanAwareObjective::HumanAwareObjective(const ompl::base::SpaceInformationPtr &si):
	ompl::base::StateCostIntegralObjective(si, true)
{
	description_ = "Human Aware";
}

ompl::base::Cost ompl_interface::HumanAwareObjective::stateCost(const ompl::base::State *s) const
{


    const ompl::base::RealVectorStateSpace::StateType* state3D =
        s->as<ompl::base::RealVectorStateSpace::StateType>();	

	double x = state3D->values[0];
	double y = state3D->values[1];
	double z = state3D->values[2];

	auto c = identityCost();

	if(abs(z) >= 0.05){
		c = ompl::base::Cost(c.value() + 50);
	}


	c = ompl::base::Cost(c.value() + x * 1000);
	return c;
	//return identityCost();
}

double movement_cost(const ompl::base::State *s1, const ompl::base::State *s2)
{
	const ompl::base::RealVectorStateSpace::StateType* state3D =
    s1->as<ompl::base::RealVectorStateSpace::StateType>();	

	double x1 = state3D->values[0];
	double y1 = state3D->values[1];
	double z1 = state3D->values[2];

	const ompl::base::RealVectorStateSpace::StateType* state3D2 =
    s2->as<ompl::base::RealVectorStateSpace::StateType>();

	double x2 = state3D2->values[0];
	double y2 = state3D2->values[1];
	double z2 = state3D2->values[2];

	double cost = 100;

	if(abs(x1-x2)/(abs(y1-y2) + 0.00001) < 0.1){
		cost = 0;
	}

	if(abs(y1-y2)/(abs(x1-x2) + 0.00001) < 0.1){
		cost = 0;
	}

	return cost;
}


ompl::base::Cost ompl_interface::HumanAwareObjective::motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    if (interpolateMotionCost_)
    {
        ompl::base::Cost totalCost = this->identityCost();
        //get the number of segments that exist between s1 and s2
        int nd = si_->getStateSpace()->validSegmentCount(s1, s2);

        ompl::base::State *test1 = si_->cloneState(s1);
        //get the cost at the first state
        ompl::base::Cost prevStateCost = this->stateCost(test1);
        //loop through all segments if there is more than 1 segment
        if (nd > 1)
        {
            ompl::base::State *test2 = si_->allocState(); //allocate an empty state
            //segment looping
            for (int j = 1; j < nd; ++j)
            {
            	//get the segment and save in test2
                si_->getStateSpace()->interpolate(s1, s2, (double)j / (double)nd, test2);
                //get the cost of test2
                ompl::base::Cost nextStateCost = this->stateCost(test2);
                totalCost = ompl::base::Cost(totalCost.value() +
                                 this->trapezoid(prevStateCost, nextStateCost, si_->distance(test1, test2)).value());

                //we also add a cost on more than one axis movements in Cartesian space
                //totalCost = ompl::base::Cost(totalCost.value() + movement_cost(test1, test2));

                //swap test1 and test2, this means the beginning of the next segment is the current left
                std::swap(test1, test2);
                prevStateCost = nextStateCost;
            }
            si_->freeState(test2);
        }

        //Lastly, add s2
        totalCost = ompl::base::Cost(totalCost.value() +
                      this->trapezoid(prevStateCost, this->stateCost(s2), si_->distance(test1, s2)).value());

        //totalCost = ompl::base::Cost(totalCost.value() + movement_cost(s1, s2));

        si_->freeState(test1);
        //ROS_INFO("here: cost:%f",totalCost.value());
        return totalCost;
    }
    else
        return this->trapezoid(this->stateCost(s1), this->stateCost(s2), si_->distance(s1, s2));
}


// {
// 	ROS_INFO("Here");
// 	si_->printProperties();
// 	si_->printSettings();
// 	return ompl::base::Cost(si_->distance(s1, s2));
// }