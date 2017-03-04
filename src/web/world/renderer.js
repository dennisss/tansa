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
		mshFloor.position.set(0, 0, -0.0005);
		mshFloor.receiveShadow = true;

		//scene.add(mshFloor);


		// Make a 6 x 9 meter grid
		var helper = new THREE.GridHelper( 3, 6, 0x888888, 0x888888 );
		var helper2 = new THREE.GridHelper( 3, 6, 0x888888, 0x888888 );
		helper.rotateX(Math.PI / 2);
		helper.position.x = 1.5
		scene.add(helper);
		helper2.rotateX(Math.PI / 2);
		helper2.position.x = -1.5
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

		this.options = {
			showTrajectories: true,
			showVehicles: true
		}
	}

	resize() {

		// Need to adjust camera aspect ratio and adjust scene size

	}


	addPoint(){
		pointI++;

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

		// Make a line
		var lineMaterial = new THREE.LineBasicMaterial({
			color: vehicleColors[this._vehicles.length - 1]
		});
		var lineGeometry = new THREE.Geometry();
		//for(var i = 0; i < 10; i++)
		//	geometry.vertices.push(new THREE.Vector3(0, 0, 0));

		var line = new THREE.Line(lineGeometry, lineMaterial);
		this.scene.add(line);
		vehicle._path = line;


		return vehicle;
	}

	/**
	 * Removes all visual elements of the vehicle from the scene
	 */
	removeVehicle(v) {
		this.scene.remove(v.object());
		this.scene.remove(v._lightHelper);
		if(v._path)
			this.scene.remove(v._path);
		if(v._homeMarker)
			this.scene.remove(v._homeMarker);
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
		this.setHomes([ [2, 3, 0] ])

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

			if(data.vehicles[i].armed && this._vehicles[i]._homeMarker) {
				this.scene.remove(this._vehicles[i]._homeMarker);
				this._vehicles[i]._homeMarker = null;
			}

		}

		this._dirty = true;

		//requestAnimationFrame(() => {
		//	this.render();
		//});
	}

	setPaths(paths) {

		for(var i = 0; i < paths.length; i++) {
			if(i <= this._vehicles.length)
				this.addVehicle();

			var p = this._vehicles[i]._path;

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

		for(var i = 0; i < homes.length; i++) {

			var v = this._vehicles[i];

			var radius = 0.2; // TODO: Base this on the vehicle size
			var height = 0.4;
			var geometry = new THREE.CylinderGeometry(radius, radius, height, 32, 1, true);
			var material = new THREE.MeshBasicMaterial({ color: vehicleColors[i], opacity: 0.2, transparent: true, side: THREE.DoubleSide });

			var obj = new THREE.Mesh(geometry, material);
			obj.rotation.x = Math.PI / 2;
			obj.position.set(homes[i][0], homes[i][1], homes[i][2] + (height / 2));
			v._homeMarker = obj;
			this.scene.add(v._homeMarker);
		}

	}


	render() {
		for(var i = 0; i < this._vehicles.length; i++) {
			this._vehicles[i].render();
			this._vehicles[i].object().visible = this.options.showVehicles;
			this._vehicles[i]._path.visible = this.options.showTrajectories;
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
