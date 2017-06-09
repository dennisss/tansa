var React = require('react'),
	ReactDOM = require('react-dom'),
	Renderer = require('./renderer'),
	Settings = require('../settings'),
	OverlayTools = require('./overlay_tools');


var options = Settings.setup({ version: 1 }, {
	grid: {
		units: 'meters',
		size: [6, 3],
		step: 0.5
	}
});

/*
	The 3d world and object viewer / editor built on Three.js
*/
var WorldView = React.createClass({

	componentDidMount: function(){
		this.renderer = new Renderer(ReactDOM.findDOMNode(this.refs.rendererEl), options);

		window.addEventListener('resize', this._windowResize);

		Settings.on('change', (s) => {
			if(s.changed('grid')) {
				this.renderer.updateGrid(options.grid);
			}
		})

		if(this.props.onRendererReady)
			this.props.onRendererReady(this.renderer);
	},

	componentWillUpdate: function(nextProps, nextState) {

	},

	shouldComponentUpdate: function(){
		if(this.renderer)
			this.renderer._dirty = true;

		return true;
	},

	_windowResize: function() {
		this.renderer.resize();
		this.renderer._dirty = true;
	},



	changeView: function(name){

		var cam = this.renderer.camera;

		if(name == 'top') {
			cam.position.y = 0;
			cam.position.x = 0;
			cam.position.z = 6;
		}
		else if(name == 'front') {
			cam.position.y = -6;
			cam.position.x = 0;
			cam.position.z = 0;
		}
		else if(name == 'right') {
			cam.position.y = 0;
			cam.position.x = 6;
			cam.position.z = 0;
		}

		this.renderer.controls.update();
		this.renderer.render();

	},

	render: function(){
		return (
			<div className="ts-world" style={{width: '100%', height: '100%', overflowY: 'hidden', position: 'relative'}}>
				<RendererHolder ref="rendererEl" />
				<OverlayTools parent={this} />
			</div>
		);
	}


});

module.exports = WorldView;

/**
 * A protective element to block the React lifecycle for three.js
 */
var RendererHolder = React.createClass({

	shouldComponentUpdate: function() {
		return false;
	},

	render: function() {
		return (
			<div style={{width: '100%', height: '100%'}}></div>
		);
	}


})
