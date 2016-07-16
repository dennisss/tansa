var THREE = require('three');
global.THREE = THREE;

require('./DragControls');
require('./OrbitControls');
require('./TransformControls');

require('./STLLoader');


class WorldPlayer {


	constructor(el){

		var width = $(el).width(), height = $(el).height();


		var scene = new THREE.Scene();
		var camera = new THREE.PerspectiveCamera( 70, width / height, 1, 20 );
		camera.position.y = 6;
		camera.position.x = 2;
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
		axis.position.set( 0, 0, 0 );
		scene.add( axis );


/*
		var textShapes = THREE.FontUtils.generateShapes("Audience", {
			size: 1
		});
		var text = new THREE.ShapeGeometry( textShapes );
		var textMesh = new THREE.Mesh( text, new THREE.MeshBasicMaterial( { color: 0xff0000 } ) ) ;
		scene.add(textMesh);
		// Example text options : {'font' : 'helvetiker','weight' : 'normal', 'style' : 'normal','size' : 100,'curveSegments' : 300};
*/


		function loadFont() {

			var loader = new THREE.FontLoader();
			loader.load( '/fonts/optimer_regular.typeface.json', function(response) {

				font = response;

				refreshText();

			});

		}


		// view-source:http://threejs.org/examples/webgl_geometry_text.html

		function createText() {

			textGeo = new THREE.TextGeometry( text, {

				font: font,

				size: size,
				height: height,
				curveSegments: curveSegments,

				bevelThickness: bevelThickness,
				bevelSize: bevelSize,
				bevelEnabled: bevelEnabled,

				material: 0,
				extrudeMaterial: 1

			});

			textGeo.computeBoundingBox();
			textGeo.computeVertexNormals();

			// "fix" side normals by removing z-component of normals for side faces
			// (this doesn't work well for beveled geometry as then we lose nice curvature around z-axis)

			if ( ! bevelEnabled ) {

				var triangleAreaHeuristics = 0.1 * ( height * size );

				for ( var i = 0; i < textGeo.faces.length; i ++ ) {

					var face = textGeo.faces[ i ];

					if ( face.materialIndex == 1 ) {

						for ( var j = 0; j < face.vertexNormals.length; j ++ ) {

							face.vertexNormals[ j ].z = 0;
							face.vertexNormals[ j ].normalize();

						}

						var va = textGeo.vertices[ face.a ];
						var vb = textGeo.vertices[ face.b ];
						var vc = textGeo.vertices[ face.c ];

						var s = THREE.GeometryUtils.triangleArea( va, vb, vc );

						if ( s > triangleAreaHeuristics ) {

							for ( var j = 0; j < face.vertexNormals.length; j ++ ) {

								face.vertexNormals[ j ].copy( face.normal );

							}

						}

					}

				}

			}

			var centerOffset = -0.5 * ( textGeo.boundingBox.max.x - textGeo.boundingBox.min.x );

			textMesh1 = new THREE.Mesh( textGeo, material );

			textMesh1.position.x = centerOffset;
			textMesh1.position.y = hover;
			textMesh1.position.z = 0;

			textMesh1.rotation.x = 0;
			textMesh1.rotation.y = Math.PI * 2;

			group.add( textMesh1 );

			if ( mirror ) {

				textMesh2 = new THREE.Mesh( textGeo, material );

				textMesh2.position.x = centerOffset;
				textMesh2.position.y = -hover;
				textMesh2.position.z = height;

				textMesh2.rotation.x = Math.PI;
				textMesh2.rotation.y = Math.PI * 2;

				group.add( textMesh2 );

			}

		}





		var renderer = new THREE.WebGLRenderer( { antialias: true } );
		renderer.setClearColor( 0xf0f0f0 );
		renderer.setPixelRatio( window.devicePixelRatio );
		renderer.setSize($(el).width(), $(el).height()); //renderer.setSize( window.innerWidth, window.innerHeight );
		renderer.shadowMap.enabled = true;
		el.appendChild( renderer.domElement );




		// Controls
		var controls = new THREE.OrbitControls( camera, renderer.domElement );
		controls.damping = 0.2;
		controls.addEventListener( 'change', () => this.render() );

		var transformControl = new THREE.TransformControls( camera, renderer.domElement );
		transformControl.addEventListener( 'change', () => this.render() );

		scene.add( transformControl );

/*
		// Hiding transform situation is a little in a mess :()
		transformControl.addEventListener( 'change', function( e ) {
			cancelHideTransorm();
		} );

		transformControl.addEventListener( 'mouseDown', function( e ) {

			cancelHideTransorm();

		} );

		transformControl.addEventListener( 'mouseUp', function( e ) {

			delayHideTransform();

		} );

		transformControl.addEventListener( 'objectChange', function( e ) {

			updateSplineOutline();

		} );

*/

/*
		var dragcontrols = new THREE.DragControls( camera, splineHelperObjects, renderer.domElement ); //

		dragcontrols.on( 'hoveron', function( e ) {

			transformControl.attach( e.object );
			cancelHideTransorm(); // *

		} )

		dragcontrols.on( 'hoveroff', function( e ) {

			if ( e ) delayHideTransform();

		} )
*/

		controls.addEventListener( 'start', function() {

			cancelHideTransorm();

		} );

		controls.addEventListener( 'end', function() {

			delayHideTransform();

		} );

		var hiding;

		function delayHideTransform() {
			cancelHideTransorm();
			hideTransform();
		}

		function hideTransform() {
			hiding = setTimeout( function() {
				transformControl.detach( transformControl.object );
			}, 2500 )
		}

		function cancelHideTransorm() {
			if ( hiding ) clearTimeout( hiding );
		}


/*
		var geometry = new THREE.BoxGeometry( 200, 200, 200 );
		var material = new THREE.MeshBasicMaterial( { color: 0xff0000, wireframe: true } );

		var mesh = new THREE.Mesh( geometry, material );
		scene.add( mesh );
*/


		this.scene = scene;
		this.camera = camera;
		this.renderer = renderer;

		this.controls = controls;
		this.transformControl = transformControl;

		this.addDrone();

		this.render();
		this.animate();
	}

	addDrone(){

		var loader = new THREE.STLLoader();
		loader.load('/models/drone.stl', (geometry) => {
			var material = new THREE.MeshPhongMaterial( { color: 0x444444, specular: 0x111111, shininess: 200 } );
			var mesh = new THREE.Mesh( geometry, material );
			mesh.position.set( 0, - 0.25, 0.6 );
		//	mesh.rotation.set( 0, - Math.PI / 2, 0 );
			mesh.scale.set( 0.5, 0.5, 0.5 );
			mesh.castShadow = true;
			mesh.receiveShadow = true;
			this.scene.add( mesh );

			this.transformControl.attach(mesh);
		});

	}


	render(){
		console.log('RENDER')
		//splines.uniform.mesh.visible = uniform.checked;
		//splines.centripetal.mesh.visible = centripetal.checked;
		//splines.chordal.mesh.visible = chordal.checked;
		this.renderer.render(this.scene, this.camera);
	}


	animate(){
		console.log('animatee')
		requestAnimationFrame(() => this.animate());
		this.render();
	//	stats.update();
		this.controls.update();
		this.transformControl.update();
	}




}

module.exports = WorldPlayer;
