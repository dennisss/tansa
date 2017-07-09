
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

	getInitialState: function() {
		return {};
	},


	render: function() {

		var cam = this.props.camera;

		var height = 180;
		var width = height * (cam.width / cam.height);

		return (
			<div style={{height: height, width: width, backgroundColor: '#000', marginRight: 10, display: 'inline-block'}}>
				{this.props.packet && this.props.packet.image? (
					<CameraImagePreview packet={this.props.packet} />
				) : null}
				{this.props.packet && this.props.packet.blobs? (
					<CameraDataPreview {...this.props} height={height} width={width} />
				) : null}
			</div>

		);

	}



});

module.exports = CameraPreview;


var CameraImagePreview = React.createClass({

	render: function() {
		var img = 'data:image/jpg;base64,' + this.props.packet.image

		return (
			<img style={{width: '100%', height: '100%'}} src={img} />
		);

	}

})

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

	propTypes: {
		packet: React.PropTypes.object,
		camera: React.PropTypes.object


	},

	componentDidMount: function() {
		var el = ReactDOM.findDOMNode(this);
		this.ctx = el.getContext("2d");
	},

	shouldComponentUpdate: function() {

		this.renderFrame();

		return true;
	},


	// TODO: Run this in a requestAnimation frame (maybe synchronize between all previews on the page)
	renderFrame: function() {

		var cam = this.props.camera;
		var blobs = this.props.packet.blobs;

		var ctx = this.ctx;

		var w = this.props.width, h = this.props.height; // Width and height of canvas
		var iw = cam.width, ih = cam.height; // Width and height of camera resolution in pixels (all blobs are in pixel units)


		// Conversion factor from camera units to ui units
		var xscale = w / iw,
			yscale = h / ih;


		// Clear the frame before
		ctx.clearRect(0, 0, w, h);

		// Draw Masks
		// TODO: I'm not sure if the mask should just be an image or a set of polygons
		// It should be an image with the user option to add polygons which are rasterized by a canvas


		// Draw blobs
		ctx.fillStyle = "#FFFFFF";
		for(var i = 0; i < blobs.length; i++) {
			ctx.beginPath();
			ctx.arc(blobs[i].x*xscale, blobs[i].y*yscale, blobs[i].r*xscale, 0, 2*Math.PI);
			ctx.fill();
		}


	},


	render: function() {

		return (
			<canvas style={{backgroundColor: '#000'}} height={this.props.height} width={this.props.width}></canvas>
		)
	}

});
