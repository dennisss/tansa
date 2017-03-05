var React = require('react');


var StatusSign = React.createClass({

	propTypes: {
		pending: React.PropTypes.bool,
		good: React.PropTypes.bool
	},

	render: function() {

		var color = this.props.good? '#0f0' : (this.props.pending? '#ff0' : '#f00');

		return (
			<span style={{paddingRight: 5, color: color, fontSize: 11, top: -1, position: 'relative'}}>
				<i className="fa fa-circle" />
			</span>
		);

	}

});

module.exports = StatusSign;
