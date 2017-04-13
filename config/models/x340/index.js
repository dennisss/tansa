var G = 9.81;

var diameter = 0.34;
var armlength = diameter / 2;


var k1 = 1;

var k2 = 1;

module.exports = {
	type: "multirotor",
	geometry: "x4",

	// TODO: Also define bounding box and offset and rotation of mesh w.r.t center of mass
	mesh: "hardware/x340/body.stl",

	mass: 0.68,

	diameter: 0.34, // length in meters between two motors across from each other

	// 0.007


	moments: [
		[0.007, 0, 0],
		[0, 0.007, 0],
		[0, 0, 0.012]
	],

/*
	moments: [ // kg m^2
		[1.079e-3, 0, 0],
		[0, 1.084e-3, 0],
		[0, 0, 2.039e-3]
	],
*/


	motor: {
		thrust_coefficient: 0.35*G, // newtons of thrust at maximum power (we assume linear throttle curve for now)
		torque_coefficient: 0.017, // Newton meters?

		time_constant_up: 0.0125,
		time_constant_down: 0.025,

		kv: 2300,
		internal_resistance: 0.117,
		idle_current: 0.6
	},

	imu: {
		accel_noise: 0.004,
		accel_rate: 1000,

		gyro_rate: 1000,
		gyro_noise: 0.0004,
		gyro_random_walk_noise: 0.00004,
	},

	gps: {

	},

	battery: {
		type: 'lipo',
		series: 3,
		capacity: 2700,
		internal_resistance: 0.024
	},

	sensors: [
		{
			type: "mocap",
			noise: 0.001
		}
	],

	firmware: {
		id: 0,
		rc: "config/gazebo/x340"
	}
}
