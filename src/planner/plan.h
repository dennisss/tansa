#ifndef PLAN_H_
#define PLAN_H_




class Plan {
public:


	/**
	 * Initialize the plan. The internal state should be reset to just before running it.
	 * Called before we want to start using this plan
	 */
	void start() = 0;

	/**
	 * For the current state of the plan, update the outputs accordingly
	 * Should return true to keep going with this plan, or false to stop it
	 */
	bool update(InputGroup *in, OutputGroup *out) = 0;


	/**
	 * Deinitialize the plan.
	 * Called after the plan is done executing
	 */
	void stop() = 0;


}
