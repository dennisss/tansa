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

		var initialState = false, playing = false;


		var cues = [];
		if(p.state.filenameI < p.state.availableFiles.length) {
			cues = p.state.availableFiles[p.state.filenameI].breakpoints;
		}

		var someArmed = false;
		if(p.state.stats && p.state.stats.vehicles.length >= 1) {
			if(p.state.stats.vehicles[0].armed)
				someArmed = true;
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

				<div className="ta-pane-header">
					Controls
				</div>
				<div className="ta-pane-body" style={{padding: 5}}>
					<div className="btn-group btn-group-sm" style={{width: '100%'}}>
						{someArmed? (
							<button className="btn btn-sm btn-default" style={{width: '33%'}} onClick={p.play}><i className="fa fa-play" style={{paddingRight: 5}} /> Play</button>
						) : (
							<button className="btn btn-sm btn-default" style={{width: '33%'}} onClick={p.prepare}>Takeoff</button>
						)}
						<button className="btn btn-sm btn-default" style={{width: '33%'}} onClick={p.pause}><i className="fa fa-pause" style={{paddingRight: 5}} /> Pause</button>
						<button className="btn btn-sm btn-default" style={{width: '33%'}} onClick={p.stop}><i className="fa fa-stop" style={{paddingRight: 5}} /> Stop</button>
					</div>

					<button className="btn btn-sm btn-danger" onClick={p.kill} style={{width: '100%', marginTop: 5}}>Kill</button>
				</div>

				{p.state.stats? <StatsSection parent={p} /> : null}

			</div>
		);
	}

});

module.exports = PropertiesPane;
