/*
	For loading text in three.js
*/


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
