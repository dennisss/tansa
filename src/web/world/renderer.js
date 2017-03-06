'use strict';

var THREE = require('three');
global.THREE = THREE;

var async = require('async');

var Vehicle = require('./vehicle');

require('./DragControls');
require('./OrbitControls');
require('./TransformControls');

require('./STLLoader');

import { MeshText2D, textAlign } from 'three-text2d'

var pointI = 1;

// Colors to use to identify unique vehicle ids
var vehicleColors = [0x0000ff, 0xff0000, 0x00ff00, 0x00ffff, 0xff00ff, 0xffff00];


/**
	Draws the 3d world state.

	The state is of the form:
	{
		vehicles: [
			{ position: Vector3(...), orientation: Vector3(...) }
			...
		],

		lines: [
			{start: Vector3(...), end: Vector3(...), color: '#00BBFF'}
			...
		]

	}

 */
class WorldRenderer {


	constructor(el) {
		this.el = el;

		var width = $(el).width(), height = $(el).height();


		var scene = new THREE.Scene();
		var camera = new THREE.PerspectiveCamera( 70, width / height, 0.1, 20 );
		camera.position.y = -6;
		camera.position.x = -2;
		camera.position.z = 2;
		camera.up.set(0, 0, 1);
		scene.add( camera );

		scene.add( new THREE.AmbientLight( 0xf0f0f0 ) );
		var light = new THREE.SpotLight( 0xffffff, 1.5 );
		light.position.set( 0, 1500, 200 );
		light.castShadow = true;
		light.shadow = new THREE.LightShadow( new THREE.PerspectiveCamera( 70, 1, 200, 2000 ) );
		light.shadow.bias = -0.000222;
		light.shadow.mapSize.width = 1024;
		light.shadow.mapSize.height = 1024;
		scene.add( light );


		/*
		var planeGeometry = new THREE.PlaneGeometry( 2000, 2000 );
		planeGeometry.rotateX( - Math.PI / 2 );
		var planeMaterial = new THREE.ShadowMaterial();
		planeMaterial.opacity = 0.2;

		var plane = new THREE.Mesh( planeGeometry, planeMaterial );
		plane.position.y = -200;
		plane.receiveShadow = true;
		scene.add( plane );
		*/

		var matFloor = new THREE.MeshPhongMaterial({ color: 0x444444, shininess: 1 });
		var geoFloor = new THREE.BoxGeometry( 16, 9, 0.001 );
		var mshFloor = new THREE.Mesh( geoFloor, matFloor );
		mshFloor.position.set(0, 0, -0.005);
		mshFloor.receiveShadow = true;

		scene.add(mshFloor);


		// Make a 6 x 9 meter grid
		var helper = new THREE.GridHelper( 3, 6, 0x888888, 0x888888 );
		var helper2 = new THREE.GridHelper( 3, 6, 0x888888, 0x888888 );
		helper.rotateX(Math.PI / 2);
		helper.position.x = 1.5
		scene.add(helper);
		helper2.rotateX(Math.PI / 2);
		helper2.position.x = -1.5
		scene.add(helper2);


		var text = new MeshText2D("FRONT", { align: textAlign.center, font: '30px Arial', fillStyle: '#888888', antialias: false });
		text.scale.x = 0.01
		text.scale.y = 0.01;
		text.position.y = -1.75
		scene.add(text)

		var axis = new THREE.AxisHelper(1);
		axis.position.set(0, 0, 0);
		scene.add(axis);



		var renderer = new THREE.WebGLRenderer({ antialias: true });
		renderer.setClearColor(0xf0f0f0);
		renderer.setPixelRatio(window.devicePixelRatio);
		renderer.setSize(width, height);
		renderer.shadowMap.enabled = true;
		el.appendChild(renderer.domElement);




		// Controls
		var controls = new THREE.OrbitControls(camera, renderer.domElement);
		controls.damping = 0.2;
		// TODO: Ensure that this doesn't update too fast
		controls.addEventListener('change', () => this._dirty = true);

		/*
		var transformControl = new THREE.TransformControls(camera, renderer.domElement);
		transformControl.addEventListener('change', () => this.render());

		scene.add(transformControl);


		// Hiding transform situation is a little in a mess :()
		transformControl.addEventListener('change', function(e){
			cancelHideTransorm();
		} );

		transformControl.addEventListener('mouseDown', function(e){
			cancelHideTransorm();
		});

		transformControl.addEventListener('mouseUp', function(e){
			delayHideTransform();
		});

		transformControl.addEventListener('objectChange', function(e){
			// Update world state
			//updateSplineOutline();
//			console.log(this._droneObjects[0].drone.position.x);

			for(var i = pointI; i < this._paths[0].geometry.vertices.length; i++)
				this._paths[0].geometry.vertices[i] = this._droneObjects[0].drone.position.clone();

			this._paths[0].geometry.verticesNeedUpdate = true;

		}.bind(this));


		var dragControls = new THREE.DragControls(camera, [], renderer.domElement);

		dragControls.on('hoveron', function(e){
			transformControl.attach(e.object.drone);
			cancelHideTransorm();
		})

		dragControls.on('hoveroff', function(e){
			if(e) delayHideTransform();
		})

		controls.addEventListener('start', function(){
			cancelHideTransorm();
		});

		controls.addEventListener('end', function(){
			delayHideTransform();
		});

		var hiding;

		function delayHideTransform() {
			cancelHideTransorm();
			hideTransform();
		}

		function hideTransform() {
			hiding = setTimeout(function(){
				transformControl.detach(transformControl.object.drone);
			}, 2500)
		}

		function cancelHideTransorm() {
			if(hiding) clearTimeout(hiding);
		}

		*/

		this.scene = scene;
		this.camera = camera;
		this.renderer = renderer;

		this.controls = controls;
		//this.transformControl = transformControl;
		//this.dragControls = dragControls;


		this._vehicles = [];
		this._homes = [];
		this._paths = [];

		this.load(() => {
			this.create();
		});



		this._dirty = true;

		this.render();
		this.animate();

		this.options = {
			showTrajectories: true,
			showVehicles: true
		}
	}

	resize() {
		// Force it to respond to the same page size
		$(this.el).css('width', '100%').css('height', '100%');

		var w = $(this.el).width(), h = $(this.el).height();

		// Need to adjust camera aspect ratio and adjust scene size
		this.camera.aspect = w / h;
		this.camera.updateProjectionMatrix();

		this.renderer.setSize(w, h);
	}

	createGround() {



	}

	addVehicle() {

		var positions = [new THREE.Vector3(0,0,0), new THREE.Vector3(0,1,0)];

		var material = new THREE.MeshLambertMaterial({ color: vehicleColors[this._vehicles.length ] /*0xBBBBBB*/, specular: 0x111111, shininess: 200 });
		var mesh = new THREE.Mesh(this._bodyGeometry, material);

		var vehicle = new Vehicle(this._bodyGeometry, this._propGeometry, material);

		//this.transformControl.attach(mesh);
		//var bbox = new THREE.BoundingBoxHelper(mesh, 0);
		//bbox.update();
		//scene.add( bbox );

		this.scene.add(vehicle.object());
		this.scene.add(vehicle._lightHelper);
		this._vehicles.push(vehicle);

		return vehicle;
	}

	/**
	 * Removes all visual elements of the vehicle from the scene
	 */
	removeVehicle(v) {
		this.scene.remove(v.object());
		this.scene.remove(v._lightHelper);
	}


	/*
		Loads all async resources needed for displaying the scene
	*/
	load(callback){

		async
		.parallel([
			(callback) => {
				var loader = new THREE.STLLoader();
				loader.load('/models/body.stl', (geometry) => {
					this._bodyGeometry = geometry;
					callback();
				});
			},
			(callback) => {
				var loader = new THREE.STLLoader();
				loader.load('/models/propeller.stl', (geometry) => {
					this._propGeometry = geometry;
					callback();
				});

			}
		], function(){
			callback();
		})

	}


	// Create initial scene
	create() {

		this._loaded = true;

		//this.dragControls.setObjects(this._droneObjects);
		//this.render();
	}

	update(data) {

		// Not yet loaded (TODO: Have a better check of whether we are loaded)
		if(!this._loaded) {
			return;
		}

		// Add vehicles
		while(data.vehicles.length > this._vehicles.length) {
			this.addVehicle();
		}
		// Remove vehicles
		while(data.vehicles.length < this._vehicles.length) {
			var last = this._vehicles.pop();
			this.removeVehicle(last);
		}

		// Update states of all vehicles
		for(var i = 0; i < data.vehicles.length; i++) {
			this._vehicles[i].update(data.vehicles[i]);

			// Only show home positions if not armed
			if(this._homes.length > i) {
				this._homes[i].visible = !data.vehicles[i].armed;
			}
		}

		this._dirty = true;
	}

	setPaths(paths) {

		while(this._paths.length > paths.length) {
			var l = this._paths.pop();
			this.scene.remove(l);
		}
		while(this._paths.length < paths.length) {
			// Make a line
			var lineMaterial = new THREE.LineBasicMaterial({
				color: vehicleColors[this._paths.length]
			});
			var lineGeometry = new THREE.Geometry();

			var line = new THREE.Line(lineGeometry, lineMaterial);
			this.scene.add(line);
			this._paths.push(line);
		}

		for(var i = 0; i < paths.length; i++) {
			var p = this._paths[i];

			// Resize vertices
			while(p.geometry.vertices.length > paths[i].length) p.geometry.vertices.pop();
			while(p.geometry.vertices.length < paths[i].length) p.geometry.vertices.push(null);

			for(var j = 0; j < paths[i].length; j++) {
				var vec = new THREE.Vector3(paths[i][j][0], paths[i][j][1], paths[i][j][2]);
				p.geometry.vertices[j] = vec;
			}

			p.geometry.verticesNeedUpdate = true;

		}
	}

	setHomes(homes) {

		for(var i = 0; i < this._homes.length; i++) {
			this.scene.remove(this._homes[i]);
		}
		this._homes = [];

		for(var i = 0; i < homes.length; i++) {

			var v = this._vehicles[i];

			var radius = 0.2; // TODO: Base this on the vehicle size
			var height = 0.4;
			var geometry = new THREE.CylinderGeometry(radius, radius, height, 32, 1, true);
			var material = new THREE.MeshBasicMaterial({ color: vehicleColors[i], opacity: 0.2, transparent: true, side: THREE.DoubleSide });

			var obj = new THREE.Mesh(geometry, material);
			console.log(homes[i])
			obj.rotation.x = Math.PI / 2;
			obj.position.set(homes[i][0], homes[i][1], /* homes[i][2] + */ (height / 2));
			this._homes.push(obj);
			this.scene.add(obj);
		}

	}


	render() {
		for(var i = 0; i < this._vehicles.length; i++) {
			this._vehicles[i].render();
			this._vehicles[i].object().visible = this.options.showVehicles;
		}

		for(var i = 0; i < this._paths.length; i++) {
			this._paths[i].visible = this.options.showTrajectories;
		}

		this.renderer.render(this.scene, this.camera);
	}


	animate(){
		if(this._dirty) {

			this.render();
			//	stats.update();
			this.controls.update();
			//this.transformControl.update();

			this._dirty = false;
		}
		requestAnimationFrame(() => this.animate());

	}

}

module.exports = WorldRenderer;
