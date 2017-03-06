var React = require('react'),
	StatusSign = require('../statusSign');

import { OverlayTrigger, Popover } from 'react-bootstrap'


var VehicleRow = React.createClass({

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

		return (
			<div className="ta-prop">
				<table style={{width: '100%'}}>
					<tbody>
						<tr>
							<td>
								<input type="checkbox" checked style={{marginRight: 5}} />
								<span style={{fontWeight: 'bold', paddingRight: 10}}>
									{data.role > 1000? 'NaN' : data.role}:
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
					</tbody>
				</table>
			</div>
		);
	}

});

module.exports = VehicleRow;

var BatteryMonitor = React.createClass({

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
