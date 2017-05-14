var React = require('react'),
	utils = require('../utils'),
	VehicleRow = require('./vehicle');


var StatsSection = React.createClass({

	render: function() {

		var p = this.props.parent;
		var stats = p.state.stats;

		return (
			<div className="ta-pane-group">
				<div className="ta-pane-header">
					Properties

					<div style={{float: 'right'}}>
						<i className="fa fa-caret-square-o-left" />
					</div>
				</div>
				<div className="ta-pane-body">


					<div className="ta-props" style={{fontSize: 12}}>
						<div className="ta-props-header">
							<i className="fa fa-minus-square-o" />&nbsp;
							Global
						</div>

						<div className="ta-prop">
							Time: {utils.timeConvert(stats.time)}
							{/*
								<div style={{float: 'right'}}>
								<input className="transparent-input" style={{width: 40}} type="number" value={35} />%
								</div>
								</div>
								*/}
						</div>
					</div>
					<div className="ta-props" style={{fontSize: 12}}>
						<div className="ta-props-header">
							<i className="fa fa-minus-square-o" />&nbsp;
							Vehicles
						</div>
						<div>
							{stats.vehicles.map((v, i) => <VehicleRow parent={p} data={v} key={i} /> )}
						</div>

					</div>

					{/* Extra space to scroll through */}
					<br />
					<br />
				</div>
			</div>

		)

	}

});

module.exports = StatsSection;
