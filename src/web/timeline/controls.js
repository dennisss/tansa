var React = require('react');

var Controls = React.createClass({

	render: function(){

		return (
			<div className="ta-timeline-controls">
				<div style={{display: 'inline-block', width: '33%'}}>
					<div className="ta-control-btn">
						<i className="fa fa-play" />
					</div>
					<div className="ta-control-btn">
						<i className="fa fa-stop" />
					</div>
					<div className="ta-control-btn">
						<i className="fa fa-fast-backward" />
					</div>

				</div>
				<div style={{display: 'inline-block', width: '33%'}}>

					<div className="ta-control-btn">
						<i className="fa fa-plus" />
					</div>

					<div className="ta-control-btn">
						<i className="fa fa-minus" />
					</div>

				</div>
				<div style={{display: 'inline-block', width: '33%', textAlign: 'right', color: '#888'}}>
					0s / 5m
				</div>


			</div>
		);
	}

});


module.exports = Controls;
