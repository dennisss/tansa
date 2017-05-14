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

				<div className="ta-pane-header">
					Input

					<div style={{float: 'right'}}>
						<i className="fa fa-caret-square-o-left" />
					</div>
				</div>

				<div className="ta-pane-body" style={{padding: 5}}>
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

				</div>

				<Controls parent={p} />

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
			<div>
				<div className="ta-pane-header">
					Controls
				</div>
				<div className="ta-pane-body" style={{padding: 5}}>
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
				</div>
			</div>
		)

	}


});

module.exports = PropertiesPane;
