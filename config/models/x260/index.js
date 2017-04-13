var G = 9.81;

var diameter = 0.260;
var armlength = diameter / 2;
var dotheight = 0.04; // Height of motor markers above center of mass
var markerPlacement = [1, 3, 4]; // The PX4 style 1-indexed motor numbers

var motorAngles = [ -45, 135, 45, -135 ];

var markersArr = [
	{
		type: 'active',
		position: [-0.03, 0, 0.028] // top marker is located 28mm height above center and 30mm behind x-y center
	}

];

for(var i = 0; i < markerPlacement.length; i++) {
	var t = motorAngles[ markerPlacement[i] - 1 ] * Math.PI / 180;
	markersArr.push({
		type: 'passive',
		position: [ (armlength * Math.cos(t)), (armlength * Math.sin(t)), dotheight ]
	});
}

var k1 = 0.5;

var k2 = 0.6

module.exports = {
	type: "multirotor",
	geometry: "x4",

	// TODO: Also define bounding box and offset and rotation of mesh w.r.t center of mass
	mesh: "hardware/x260/body.stl",

	mass: 0.30,

	diameter: 0.260, // length in meters between two motors across from each other

	// Inertia from model
	moments: [ // kg m^2
		[1.079e-3, 0, 0],
		[0, 1.084e-3, 0],
		[0, 0, 2.039e-3]
	],


	markers: markersArr,

	motor: {
		thrust_coefficient: 0.21*G, // newtons of thrust at maximum power (we assume linear throttle curve for now)
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
		series: 2,
		capacity: 1500,
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
