
var React = require('react'),
	ReactDOM = require('react-dom');

var height = 180;
var aspectRatio = 4 / 3;
var width = height * aspectRatio;

/**
 * A view for displaying the data coming in from a single camera
 *
 * TODO: Currently this will only support displaying the circles detected in each frame
 */
var CameraPreview = React.createClass({



	render: function() {

		return (
			<div style={{height: height, width: width, backgroundColor: '#000', marginRight: 10}}>

			</div>

		);

	}



});

module.exports = CameraPreview;


var CameraVideoPreview = React.createClass({


	render: function() {

		return (
			<video></video>
		)
	}

});

/**
 * Used by CameraPreview to show a view drawn from processed data
 * - Individual detected blobs
 * - Mask locations
 */
var CameraDataPreview = React.createClass({

	componentWillMount: function() {

		var el = ReactDOM.findDOMNode(this);
		this.ctx = el.getContext("2d");



	},

	// TODO: Run this in a requestAnimation frame (maybe synchronize between all previews on the page)
	renderFrame: function() {
		var ctx = this.ctx;

		var w, h; // Width and height of canvas
		var iw, ih; // Width and height of camera resolution in pixels (all blobs are in pixel units)


		// Conversion factor from camera units to ui units
		var xscale = w / iw,
			yscale = h / ih;


		// Clear the frame before
		ctx.clearRect(0, 0, w, h);

		// Draw Masks
		// TODO: I'm not sure if the mask should just be an image or a set of polygons

		// Draw blobs
		ctx.fillStyle = "#FFFFFF";
		for(var i = 0; i < blobs.length; i++) {

			var x, y, radius;

			ctx.beginPath();
			ctx.arc(x*xscale, y*yscale, radius*xscale, 0, 2*Math.PI);
			ctx.fill();
		}


	},


	render: function() {

		return (
			<canvas style={{backgroundColor: '#000'}}></canvas>
		)
	}

});
