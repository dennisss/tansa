'use strict';

/**
 * Manages the 3d model of a single entity (so a drone body and its propellers)
 * This updates the position, orientation and
 */
class Vehicle {

	constructor(bodyGeometry, propGeometry, material) {
		this._group = new THREE.Group();

		// Create body model rotated 45 degrees to look like X instead of +
		this._body = new THREE.Mesh(bodyGeometry, material);
		this._body.rotation.set(0, 0, Math.PI / 4);
		this._body.castShadow = true;
		this._body.receiveShadow = true;
		this._group.add(this._body);

		this._props = [];
		var angles = [45, 180+45, 90+45, -45]; // PX4 Motor order
		var length = 0.260 / 2;
		for(var i = 0; i < angles.length; i++) {
			var p = new THREE.Mesh(propGeometry, material);
			var a = angles[i] * Math.PI / 180;
			var x = length * Math.cos(a);
			var y = length * Math.sin(a);
			p.position.set(x, y, 0.03); // 30mm above origin
			this._props.push(p);
			this._group.add(p);
		}


		// State for the purposes of animation
		this._state = {
			position: new THREE.Vector3(0, 0, 0), // Position of the whole body
			orientation: new THREE.Quaternion(0, 0, 0, 1),

			speeds: [0, 0, 0, 0], // Speed of each motor from 0 to 1 for now
			angles: [0, 0, 0, 0], // Yaw angle of each propeller (in radians)
			lastTime: new Date() // Last time at which the model was rendered
		}
	}

	/**
	 * Should return the three.js object which can be rendered into the scene
	 */
	object() {
		return this._group;
	}

	/**
	 *
	 */
	update(data) {
		var s = this._state;

		var o = data.orientation;
		s.orientation = new THREE.Quaternion(o[1], o[2], o[3], o[0]);

		var p = data.position;
		s.position.set(p[0], p[1], p[2]);

		s.speeds = data.motors;
	}

	/**
	 * Called whenever the next frame as about to be drawn
	 */
	render() {
		var s = this._state;
		var t = new Date();
		var dt = (t - s.lastTime) / 1000;

		// TODO: Also make sure that the motor speeds come with the right spin
		for(var i = 0; i < this._props.length; i++) {
			//this._props[i].rotateZ(dt*s.speeds[i]);
		}
		s.lastTime = t;

		this._group.position.copy(s.position)
		this._group.rotation.setFromQuaternion(s.orientation);
	}

}

module.exports = Vehicle;
