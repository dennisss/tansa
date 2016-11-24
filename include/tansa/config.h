#ifndef TANSA_CONFIG_H
#define TANSA_CONFIG_H

#define VEHICLE_ASCENT_MS 0.2
#define VEHICLE_DESCENT_MS 0.3

struct hardware_config {
	string clientAddress;
	string serverAddress;
};

struct vehicle_config {
	unsigned net_id; // The number printed on the physical
	unsigned chor_id; // The id between 1 and 6 respesenting which drone in the choreography it w
	unsigned lport; // Usually 14550 + id*10
	unsigned rport; // For now always 14555
};

/**
 * State assigned to a vehicle controlled by a player
 */
enum PlayerVehicleState {
	/**
	 * Vehicle disarmed and not doing anything
	 *
	 * Transitions to Ready upon arm()
	 */
	StateInit,

	/**
	 * Arms the drone, transitions to Ready on completion
	 */
	StateArming,

	/**
	 * Vehicle armed but doing nothing on the ground
	 *
	 * Once all vehicles are Ready, transitions to Takeoff on load of a JOCS file
	 */
	StateReady,

	/**
	 * Vehicle going to starting point of first action
	 *
	 * If the starting point is on the ground, then:
	 * - if the vehicle is on the ground, this is a no-op
	 * - else this descends and lands without entering Landing state
	 *
	 * Transitions to Holding upon getting to the point
	 */
	StateTakeoff,

	/**
	 * Hovers/sits in one place
	 *
	 * If in the air and 20 seconds have elapsed, transitions to Landing state
	 *
	 * Transitions to Takeoff upon load of a new JOCS file
	 *
	 * Transitions to Flying on play()
	 */
	StateHolding,

	/**
	 * Performs the assigned actions
	 *
	 * After a pause(), on next safe point, transitions to Holding
	 *
	 * Transitions to Landing after completion of all actions
	 */
	StateFlying,

	/**
	 * Landing at the current position and disarming
	 *
	 * Transitions to Init upon completion
	 */
	StateLanding
};


#endif //TANSA_CONFIG_H
