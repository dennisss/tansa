var React = require('react'),
	StatsSection = require('./stats'),
	UploadButton = require('./upload'),
	Settings = require('../settings');

import { OverlayTrigger, Tooltip } from 'react-bootstrap'

require('./style.css')

var PropertiesPane = React.createClass({

	onFileChanged: function(e) {
		var p = this.props.parent;

		var i = e.target.value;

		var cue = p.state.cue;
		if(p.state.availableFiles[i].breakpoints.indexOf(cue) < 0) {
			cue = 0;
		}

		Settings.set('file.name', p.state.availableFiles[i].fileName);
		p.setState({filenameI: i, cue: cue});

	},

	onCueChange: function(e) {
		var p = this.props.parent;

		var c = e.target.value;
		Settings.set('file.cue', c);
		p.setState({cue: c});
	},

	render: function() {

		var p = this.props.parent;

		var cues = [];
		if(p.state.filenameI < p.state.availableFiles.length) {
			cues = p.state.availableFiles[p.state.filenameI].breakpoints;
		}

		return (
			<div className="ta-pane" style={{height: '100%', overflowY: 'scroll', overflowX: 'hidden', marginRight: -15 /* This margin will hide the scroll bar */}}>

				{/* TODO: Rename this from ta-pane-* */}
				<div className="ta-pane-header">
					Properties

					<div style={{float: 'right'}}>
						<i className="fa fa-caret-square-o-left" />
					</div>
				</div>


				<PropertiesSection name="Input">
					<div className="form-group">
						<span>File</span>
						<div className="input-group">
							<select className="form-control" style={{'WebkitAppearance': 'none'}} value={p.state.filenameI} onChange={this.onFileChanged}>
								{p.state.availableFiles.map((f, i) => <option key={i} value={i}>{f.fileName}</option> )}
							</select>
							<span className="input-group-btn">
								<UploadButton parent={p} />
							</span>
						</div>
					</div>
					<div className="row">
						<div className="col-sm-8" style={{paddingRight: 0}}>
							<div className="form-group">
								<span>Breakpoint</span>
								<select className="form-control" value={p.state.cue} onChange={this.onCueChange}>
									<option value={-1}>- None -</option>
									{cues.map((c, i) => <option key={i} value={c.number}>{c.name}</option>)}
								</select>
							</div>
						</div>
						<div className="col-sm-4">
							<div className="form-group">
								<span>Scale</span>
								<input className="form-control" type="number" step="0.1" min="0" max="4" value={p.state.scale*1} onChange={(e) => p.setState({scale: e.target.value*1})} />
							</div>
						</div>
					</div>

					<div className="input-group" style={{width: '100%'}}>

						<span className="input-group-btn" style={{width: 'initial'}}>
							<button className="btn btn-sm btn-primary" style={{width: '100%'}} onClick={p.load}>Load File</button>
						</span>

						{/* TODO: Hide this if the regular mode has already started */}
						<span className="input-group-btn" style={{width: 1}}>
							<OverlayTrigger overlay={<Tooltip id="preview">Preview</Tooltip>} placement="right">
								<button onClick={p.preview} className="btn btn-sm btn-primary">
									<i className="fa fa-eye" />
								</button>
							</OverlayTrigger>
						</span>
					</div>

				</PropertiesSection>

				<Controls parent={p} />

				<CameraSettings parent={p} />
				<CameraCalibration parent={p} />

				{p.state.stats? <StatsSection parent={p} /> : null}

			</div>
		);
	}

});


var Controls = React.createClass({

	render: function() {

		var p = this.props.parent;


		var state = '';
		var playing = false;
		var allConnected = false;
		if(p.state.stats && p.state.stats.vehicles.length >= 1) {
			// TODO: Consider the state of all drones
			state = p.state.stats.vehicles[0].state;
			playing = p.state.stats.global.playing;


			var vs = p.state.stats.vehicles;
			allConnected = true;
			for(var i = 0; i < vs.length; i++) {
				if(!vs[i].connected || !vs[i].tracking) {
					allConnected = false;
					break;
				}
			}

		}


		var btns = [];

		var msg = null;
		if(state == 'init') {
			if(!allConnected) {
				msg = "Some vehicles not synced"
			}
			else {
				btns.push(
					<button className="btn btn-sm btn-default" style={{width: '33%'}} onClick={p.prepare}>
						<i className="fa fa-arrow-up" style={{paddingRight: 5}} />
						Takeoff
					</button>
				);
			}
		}
		else if(state == 'holding') {
			btns.push(
				<button className="btn btn-sm btn-default" onClick={p.land}>
					<i className="fa fa-arrow-down" style={{paddingRight: 5}} />
					Land
				</button>
			);
			btns.push(
				<button className="btn btn-sm btn-default" onClick={p.play}><i className="fa fa-play" style={{paddingRight: 5}} /> Play</button>
			);
		}
		else if(playing) {
			btns.push(
				<button className="btn btn-sm btn-default" onClick={p.pause}><i className="fa fa-pause" style={{paddingRight: 5}} /> Pause</button>
			);
			btns.push(
				<button className="btn btn-sm btn-default" onClick={p.stop}><i className="fa fa-stop" style={{paddingRight: 5}} /> Stop</button>
			)
		}


		var preButtons = null;
		if(state == 'init' && p.state.stats.vehicles.length >= 1) {
			preButtons = (
				<div className="btn-group btn-group-sm" style={{width: '100%', marginTop: 5}}>
					<button className="btn btn-sm btn-default" onClick={p.calibrate} style={{width: '33%'}}>
						{p.state.calibrating? <i className="fa fa-cog fa-spin" /> : 'Calibrate'}
					</button>
					{/*
					<button className="btn btn-sm btn-default" onClick={p.resync} style={{width: '33%'}}>
						Resync
					</button>
					<button className="btn btn-sm btn-default" onClick={p.rearrange} style={{width: '33%'}}>
						Rearrange
					</button>
					*/}
				</div>
			);
		}



		return (
			<PropertiesSection name="Controls">
				<div className="btn-group btn-group-sm" style={{width: '100%'}}>
					{btns.map((b, i) => React.cloneElement(b, {key: i, style: {width: (100 / btns.length) + '%'}}))}
					{btns.length == 0? (
						<div style={{textAlign: 'center', color: '#ddd', width: '100%'}}>
							{msg || 'Pending...'}
						</div>
					) : null}
				</div>

				{preButtons}

				<button className="btn btn-sm btn-danger" onClick={() => p.kill(-1)} style={{width: '100%', marginTop: 10}}>Kill</button>
			</PropertiesSection>
		)

	}


});


var CameraSettings = React.createClass({


	getInitialState: function() {
		return {
			fps: 0,
			exposure: 0,
			threshold: 0,
			led: 0,
			mode: ''
		};
	},

	componentWillMount: function() {
		// Get inital settings from server
		// TODO:
	},


	render: function() {

		return (
			<PropertiesSection name="Camera Settings">
				<table style={{width: '100%'}}>
					<tbody>
						<tr>
							<td>Video</td>
							<td><input type="checkbox" checked={this.state.mode == 'video'} /></td>
						</tr>
						<tr>
							<td>FPS</td>
							<td>
								<input type="number" className="transparent-input" min={1} step={1} max={90}
									value={this.state.fps} />
							</td>
						</tr>
						<tr>
							<td>Exp.</td>
							<td>
								<input type="number" className="transparent-input" min={100} step={100} max={10000}
									value={this.state.exposure} />
							</td>
						</tr>
						<tr>
							<td>Thr.</td>
							<td>
								<input type="number" className="transparent-input" min={0} step={1} max={255}
									value={this.state.threshold} />
							</td>
						</tr>
						<tr>
							<td>LED</td>
							<td>
								<input type="number" className="transparent-input" min={0} step={1} max={100}
									value={this.state.led} />
							</td>
						</tr>

						<tr>
							<td>Masking</td>
							<td>
								<div className="btn-group">
									<button className="btn btn-xs btn-default"><i className="fa fa-plus" /></button>
									<button className="btn btn-xs btn-default"><i className="fa fa-minus" /></button>
								</div>
							</td>
						</tr>

					</tbody>

				</table>

			</PropertiesSection>
		);
	}

});

var CameraCalibration = React.createClass({

	getInitialState: function() {
		return {
			_started: false,
			_ncaptured: 0
		};
	},

	_startWanding: function() {
		this.setState({_started: true, _ncaptured: 0});
	},

	_setGround: function() {



	},

	render: function() {

		return (
			<PropertiesSection name="Camera Calibration">
				{this.state._started? (
					<div>
						<p>Captured: {this.state._ncaptured}</p>
						<button className="btn btn-info">Take Image</button>
						<br /><br />
						<div className="btn-group">
							<button className="btn btn-sm btn-primary">Finish</button>
							<button className="btn btn-sm btn-default">Cancel</button>
						</div>
					</div>
				) : (
					<div className="btn-group">
						<div className="btn btn-sm btn-default" onClick={this._startWanding}>Start Wanding</div>
						<div className="btn btn-sm btn-default" onClick={this._setGround}>Set Ground Plane</div>
					</div>
				)}
			</PropertiesSection>
		);


	}

});



var PropertiesSection = React.createClass({

	getInitialState: function() {
		return {
			_open: true
		}
	},

	render: function() {

		return (
			<div>
				<div className="ta-pane-header" style={{cursor: 'pointer'}} onClick={() => this.setState({_open: !this.state._open})}>
					{this.props.name}
					<i className={this.state._open? 'fa fa-minus' : 'fa fa-plus'} style={{float: 'right', position: 'relative', top: 3}} />
				</div>
				{this.state._open? (
					<div className="ta-pane-body" style={{padding: 5}}>
						{this.props.children}
					</div>
				) : null}
			</div>


		);


	}

})

module.exports = PropertiesPane;
