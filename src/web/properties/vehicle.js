var React = require('react'),
	StatusSign = require('../statusSign');

import { OverlayTrigger, Popover } from 'react-bootstrap'

// TODO: This is the same as in renderer.js
var trackColors = ['0000ff', 'ff0000', '00ff00', '00ffff', 'ff00ff', 'ffff00'];


var VehicleRow = React.createClass({

	shouldComponentUpdate: function(nextProps) {
		return JSON.stringify(nextProps.data) != JSON.stringify(this.props.data);
	},

	render: function() {

		var data = this.props.data;


		var statusOverlay = (
			<Popover id="vehicle-status">
				<div>
					<StatusSign good={data.connected} />&nbsp;Connected
				</div>
				<div>
					<StatusSign good={data.tracking} />&nbsp;Tracking
				</div>
				<div>
					<StatusSign good={data.armed} />&nbsp;Armed
				</div>
			</Popover>
		);

		var color = trackColors[data.role];

		var p = this.props.parent;

		return (
			<div className="ta-prop">
				<table style={{width: '100%'}}>
					<tbody>
						<tr>
							<td>
								{/*
								<input type="checkbox" checked style={{marginRight: 5}} />
								*/}
								<div style={{width: 10, height: 10, backgroundColor: ('#' + color), border: '1px solid #fff', marginRight: 6, display: 'inline-block'}}></div>
								<span style={{fontWeight: 'bold', paddingRight: 10}}>
									Track {data.role > 1000? 'NaN' : data.role}:
								</span>
							</td>
							<td>
								ID: {data.id}
							</td>
							<td>
								{data.state}
							</td>
							<td>

								<BatteryMonitor data={data.battery} />
							</td>
							<td style={{textAlign: 'right'}}>
								<OverlayTrigger placement="right" overlay={statusOverlay}>
									<span>
										<StatusSign pending={data.connected && data.tracking} good={data.armed && data.tracking} />
									</span>
								</OverlayTrigger>
							</td>
						</tr>
						<tr>
							<td colSpan="5" style={{paddingLeft: 20, paddingTop: 5, paddingBottom: 5}}>
								{/*
								{data.state == 'flying'? (
									<button className="btn btn-xs" style={{marginRight: '2em'}}>Stop</button>
								) : null}
								{data.state == 'holding'? (
									<button className="btn btn-xs" style={{marginRight: '2em'}}>Land</button>
								) : null}
								*/}

								<div className="btn-group">
									<button className="btn btn-xs btn-danger" onClick={() => p.kill(data.role * 1)}>Kill</button>
									<button className="btn btn-xs btn-danger" onClick={() => p.halt(data.role * 1)}>Halt</button>
								</div>



							</td>
						</tr>
					</tbody>
				</table>
			</div>
		);
	}

});

module.exports = VehicleRow;

var BatteryMonitor = React.createClass({

	shouldComponentUpdate: function(nextProps) {
		return JSON.stringify(nextProps.data) != JSON.stringify(this.props.data);
	},

	render: function() {

		var data = this.props.data;
		var percent = data.percent * 100;
		var voltage = data.voltage;

		var range = Math.floor(percent / 20);
		if(range >= 5) range = 4;
		if(range < 0) range = 0;

		var overlay = (
			<Popover id="battery-verbose">
				Percent: {Math.round(percent) + '%'}
				<br />
				Voltage: {Math.round(voltage*100) / 100}V
			</Popover>
		);

		return (
			<OverlayTrigger placement="right" overlay={overlay}>
				<i className={"fa fa-battery-" + range} style={{color: percent < 0.2? 'red' : null}} />
			</OverlayTrigger>
		);



	}


});
