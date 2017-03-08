/*
	Based heavy on the MIT licensed file available here as part of three.js
	https://github.com/mrdoob/three.js/blob/master/src/helpers/GridHelper.js
*/


var LineSegments = THREE.LineSegments,
	LineBasicMaterial = THREE.LineBasicMaterial,
	Float32BufferAttribute = THREE.Float32BufferAttribute,
	BufferGeometry = THREE.BufferGeometry,
	Color = THREE.Color;

import { VertexColors } from '../../../node_modules/three/src/constants';

// Both size and divisions are 2d
function GridHelper2D( size, divisions, color1, color2 ) {

	size = size || [10, 10];
	divisions = divisions || [10, 10];
	color1 = new Color( color1 !== undefined ? color1 : 0x444444 );
	color2 = new Color( color2 !== undefined ? color2 : 0x888888 );

	var center = [divisions[0] / 2, divisions[1] / 2];
	var step = [size[0] / divisions[0], size[1] / divisions[1]];
	var halfSize = [size[0] / 2, size[1] / 2];

	var vertices = [], colors = [];

	var j = 0;

	// First vertical x ticks
	for ( var i = 0, k = - halfSize[0]; i <= divisions[0]; i ++, k += step[0] ) {

		vertices.push(
			k, 0, - halfSize[1],
			k, 0, halfSize[1]
		);

		var color =  i === center ? color1 : color2;

		color.toArray( colors, j ); j += 3;
		color.toArray( colors, j ); j += 3;
	}

	// Next horizontal x ticks
	for ( var i = 0, k = - halfSize[1]; i <= divisions[1]; i ++, k += step[1] ) {

		vertices.push(
			- halfSize[0], 0, k,
			halfSize[0], 0, k
		);

		var color = i === center ? color1 : color2;

		color.toArray( colors, j ); j += 3;
		color.toArray( colors, j ); j += 3;
	}


	var geometry = new BufferGeometry();
	geometry.addAttribute( 'position', new Float32BufferAttribute( vertices, 3 ) );
	geometry.addAttribute( 'color', new Float32BufferAttribute( colors, 3 ) );

	var material = new LineBasicMaterial( { vertexColors: VertexColors } );

	LineSegments.call( this, geometry, material );

}

GridHelper2D.prototype = Object.create( LineSegments.prototype );
GridHelper2D.prototype.constructor = GridHelper2D;

module.exports = GridHelper2D;
