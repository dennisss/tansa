var React = require('react'),
	Settings = require('./settings');

import { Modal, Button } from 'react-bootstrap';


var SettingsModal = React.createClass({

	set: function(name, value) {
		Settings.set(name, value);
		this.forceUpdate();
	},

	render: function(){

		var s = Settings.get();

		return (
			<Modal {...this.props} bsSize="lg">
				<Modal.Header closeButton>
					<Modal.Title>Settings</Modal.Title>
				</Modal.Header>

				<Modal.Body>

					<h3>Grid</h3>
					<table style={{width: '100%'}} className="form-inline">
						<tbody>
							<tr>
								<td>Units</td>
								<td style={{paddingTop: 5}}>
									<select value={s.grid.units} onChange={(e) => this.set('grid.units', e.target.value)} className="form-control">
										<option value="feet">Feet</option>
										<option value="meters">Meters</option>
									</select>
								</td>
							</tr>
							<tr>
								<td>Length (along X)</td>
								<td style={{paddingTop: 5}}>
									<input value={s.grid.size[0]} onChange={(e) => this.set('grid.size[0]', e.target.value*1)} type="number" min="0" max="100" step="0.5" className="form-control" /> {s.grid.units}
								</td>
							</tr>
							<tr>
								<td>Width (along Y)</td>
								<td style={{paddingTop: 5}}>
									<input value={s.grid.size[1]} onChange={(e) => this.set('grid.size[1]', e.target.value*1)} type="number" min="0" max="100" step="0.5" className="form-control" /> {s.grid.units}
								</td>
							</tr>
							<tr>
								<td>Step Size (between grid lines)</td>
								<td style={{paddingTop: 5}}>
									<input value={s.grid.step} onChange={(e) => this.set('grid.step', e.target.value*1)} type="number" min="0" max="10" step="0.25" className="form-control" /> {s.grid.units}
								</td>
							</tr>
						</tbody>
					</table>

					{/*
					<h3>Trajectories</h3>
					<div>
						Back Time
					</div>
					<div>
						Forward Time
					</div>
					*/}

					{/*
					<h3>Network IDs</h3>
					<div>
						<input type="text" className="form-control" />
						<span>This is a comma separated list of which physical drones will be used. Order determines their order in the file.</span>
					</div>
					*/}

					{/*
						Drone configuration list
						- Set Home options

						Change theater configuration
						- Select from preset list
						- Or, allow custom entry
					*/}

				</Modal.Body>

				<Modal.Footer>
					{/*
					<button className="btn btn-primary" onClick={this.submit} >
						Save
					</button>
					*/}
					<Button onClick={this.props.onHide}>Close</Button>
				</Modal.Footer>
			</Modal>
		);

	}



});

module.exports = SettingsModal;
