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


		var spotLight = new THREE.SpotLight( 0x000000, 1 );
		this._spotLight = spotLight;
		spotLight.position.set( 0, 0, -0.02 );
		//spotLight.rotation.set(0, Math.PI/2, 0)
		spotLight.castShadow = true;
		spotLight.angle = 25 * (Math.PI / 180);
		spotLight.penumbra = 0.2;
		spotLight.decay = 2;
		spotLight.distance = 10;
		spotLight.shadow.mapSize.width = 1024;
		spotLight.shadow.mapSize.height = 1024;
		spotLight.shadow.camera.near = 0.5;
		spotLight.shadow.camera.far = 200;

		// Point downwards
		spotLight.target.position.set(0, 0, -1);


		this._lightHelper = new THREE.SpotLightHelper( spotLight );
		this._lightHelper.visible = false;

		//this._group.add(this._lightHelper);
		this._group.add(spotLight)
		this._group.add(spotLight.target);

		//spotLight.target.position.set(0, 0, -3)

		// State for the purposes of animation
		this._state = {
			position: new THREE.Vector3(0, 0, 2), // Position of the whole body
			orientation: new THREE.Quaternion(0, 0, 0, 1),

			speeds: [0, 0, 0, 0], // Speed of each motor from 0 to 1 for now
			angles: [0, 0, 0, 0], // Yaw angle of each propeller (in radians)
			lights: [],
			lastTime: new Date() // Last time at which the model was rendered
		}

		this._t = new Date();
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

		s.speeds = data.armed? [0.5, 0.5, 0.5, 0.5] : [0,0,0,0];  //data.motors;

		s.lights = data.lights;

		var t = new Date();
		var dt = (t - this._t) / 1000;
		this._t = t;
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
			this._props[i].rotateZ(dt*s.speeds[i]*60);
		}
		s.lastTime = t;

		this._group.position.copy(s.position)
		this._group.rotation.setFromQuaternion(s.orientation);


		var color = 0x000000;
		var channels = s.lights.slice() || [];
		if(channels.length == 1) {
			channels = [channels[0], channels[0], channels[0]];
		}
		if(channels.length == 3) {
			color = ((channels[0]*0xff) << 16) | ((channels[1]*0xff) << 8) | (channels[2]*0xff)
		}

		this._lightHelper.visible = color > 0;
		this._spotLight.color.set(color);

		this._lightHelper.update();
	}

}

module.exports = Vehicle;
