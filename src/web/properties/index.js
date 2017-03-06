var React = require('react'),
	StatsSection = require('./stats'),
	UploadButton = require('./upload');

require('./style.css')

var PropertiesPane = React.createClass({

	onFileChanged: function(e) {
		var p = this.props.parent;

		var i = e.target.value;

		var cue = p.state.cue;
		if(p.state.availableFiles[i].breakpoints.indexOf(cue) < 0) {
			cue = 0;
		}

		p.setState({filenameI: e.target.value, cue: cue});

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
							<select className="form-control" value={p.state.filenameI} onChange={this.onFileChanged}>
								{p.state.availableFiles.map((f, i) => <option key={i} value={i}>{f.fileName}</option> )}
							</select>
							<span className="input-group-btn">
								<UploadButton parent={p} />
							</span>
						</div>
					</div>
					<div className="row">
						<div className="col-sm-6">
							<div className="form-group">
								<span>Breakpoint</span>
								<select className="form-control" value={p.state.cue} onChange={(e) => p.setState({cue: e.target.value})}>
									<option value={-1}>- None -</option>
									{cues.map((c, i) => <option key={i} value={c.number}>{c.name}</option>)}
								</select>
							</div>
						</div>
						<div className="col-sm-6">
							<div className="form-group">
								<span>Scale</span>
								<input className="form-control" type="number" step="0.1" min="0" max="4" value={p.state.scale} onChange={(e) => p.setState({scale: e.target.value})} />
							</div>
						</div>
					</div>
					<button className="btn btn-sm btn-primary" style={{width: '100%'}} onClick={p.load}>Load File</button>

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

					<button className="btn btn-sm btn-danger" onClick={p.kill} style={{width: '100%', marginTop: 5}}>Kill</button>
				</div>
			</div>
		)

	}


});

module.exports = PropertiesPane;
