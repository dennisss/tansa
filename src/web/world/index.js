var React = require('react'),
	ReactDOM = require('react-dom'),
	Renderer = require('./renderer'),
	Settings = require('../settings');


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
		this.renderer = new Renderer(ReactDOM.findDOMNode(this), options);

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

		return false;
	},

	_windowResize: function() {
		this.renderer.resize();
		this.renderer._dirty = true;
	},

	render: function(){
		return (
			<div className="ts-world" style={{width: '100%', height: '100%', overflowY: 'hidden'}}></div>
		);
	}


});

module.exports = WorldView;
