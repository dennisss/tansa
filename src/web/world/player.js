'use strict';

var THREE = require('three');
global.THREE = THREE;

var async = require('async');

var Vehicle = require('./vehicle');

require('./DragControls');
require('./OrbitControls');
require('./TransformControls');

require('./STLLoader');

var pointI = 1;

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
class WorldPlayer {


	constructor(el) {

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

		// scene.add( new THREE.CameraHelper( light.shadow.camera ) );


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


		// Make a 6 x 9 meter grid
		var helper = new THREE.GridHelper( 3, 0.5 );
		var helper2 = new THREE.GridHelper( 3, 0.5 );
		helper.rotateX(Math.PI / 2);
		helper.position.x = 1.5
		helper.material.opacity = 0.25;
		helper.material.transparent = true;
		scene.add(helper);
		helper2.rotateX(Math.PI / 2);
		helper2.position.x = -1.5
		helper2.material.opacity = 0.25;
		helper2.material.transparent = true;
		scene.add(helper2);

		var axis = new THREE.AxisHelper(1);
		axis.position.set(0, 0, 0);
		scene.add(axis);





		var renderer = new THREE.WebGLRenderer({ antialias: true });
		renderer.setClearColor(0xf0f0f0);
		renderer.setPixelRatio(window.devicePixelRatio);
		renderer.setSize($(el).width(), $(el).height()); //renderer.setSize( window.innerWidth, window.innerHeight );
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

		this.load(() => {
			this.create();
		});



		this._dirty = true;

		this.render();
		this.animate();
	}

	resize() {

		// Need to adjust camera aspect ratio and adjust scene size

	}


	addPoint(){
		pointI++;

	}

	addVehicle() {

		var positions = [new THREE.Vector3(0,0,0), new THREE.Vector3(0,1,0)];

		var material = new THREE.MeshLambertMaterial({ color: 0x888888, specular: 0x111111, shininess: 200 });
		var mesh = new THREE.Mesh(this._bodyGeometry, material);

		var vehicle = new Vehicle(this._bodyGeometry, this._propGeometry, material);

		//this.transformControl.attach(mesh);
		//var bbox = new THREE.BoundingBoxHelper(mesh, 0);
		//bbox.update();
		//scene.add( bbox );

		this.scene.add(vehicle.object());
		this._vehicles.push(vehicle);

		return vehicle;
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
		this.addVehicle();

		this._paths = [];


		// Make a line
		var material = new THREE.LineBasicMaterial({
			color: 0x0000ff
		});
		var geometry = new THREE.Geometry();
		for(var i = 0; i < 10; i++)
			geometry.vertices.push(new THREE.Vector3(0, 0, 0));


		var line = new THREE.Line(geometry, material);
		this.scene.add(line);

		this._paths.push(line);

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
			this.scene.remove(last.object());
		}

		// Update states of all vehicles
		for(var i = 0; i < data.vehicles.length; i++) {
			this._vehicles[i].update(data.vehicles[i]);
		}

		this._dirty = true;

		//requestAnimationFrame(() => {
		//	this.render();
		//});
	}


	render() {
		for(var i = 0; i < this._vehicles.length; i++) {
			this._vehicles[i].render();
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

module.exports = WorldPlayer;
