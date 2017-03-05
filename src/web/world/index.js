var React = require('react'),
	ReactDOM = require('react-dom'),
	Renderer = require('./renderer');

/*
	The 3d world and object viewer / editor built on Three.js
*/
var WorldView = React.createClass({

	componentDidMount: function(){
		this.renderer = new Renderer(ReactDOM.findDOMNode(this));

		window.addEventListener('resize', this._windowResize);

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
