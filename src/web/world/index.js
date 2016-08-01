var React = require('react'),
	ReactDOM = require('react-dom'),
	Player = require('./player');

/*
	The 3d world and object viewer / editor built on Three.js
*/
var WorldView = React.createClass({

	componentDidMount: function(){
		this.player = new Player(ReactDOM.findDOMNode(this));

		if(this.props.onPlayerReady)
			this.props.onPlayerReady(this.player);
	},

	shouldComponentUpdate: function(){
		// TODO: Manage resizing the window

		return false;
	},

	render: function(){
		return (
			<div className="ts-world" style={{width: '100%', height: '100%'}}></div>
		);
	}


});

module.exports = WorldView;
