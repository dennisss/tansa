var React = require('react'),
	Navbar = require('./navbar'),
	WorldView = require('./world'),
	Timeline = require('./timeline'),
	PropertiesPane = require('./properties'),
	Socket = require('./socket');

import getMuiTheme from 'material-ui/styles/getMuiTheme';


var App = React.createClass({

	childContextTypes: {
		muiTheme: React.PropTypes.object
	},

	getChildContext: function() {
		return {
			muiTheme: getMuiTheme()
		};
	},


	getInitialState: function() {

		return {
			availableFiles: [],
			availableCues: [],

			filenameI: 0,
			cue: 0,
			scale: 1,

			stats: null // The full object from the 'status' message
		}

	},


	onRendererReady: function(renderer){
		this.renderer = renderer;

		var it = 0;
		Socket.on('msg', function(data) {
			data = JSON.parse(data);

			if(data.type == 'status') {
				if((it++) % 10 == 0) {
					this.setState({stats: data});
				}
				renderer.update({vehicles: data.vehicles});
			}
			else if(data.type == 'load_reply') {
				renderer.setPaths(data.paths);
				renderer.setHomes(data.target_positions);
			}
			else if(data.type == 'list_reply') {
				this.setState({availableFiles: data.files});

				// Right after an upload, set the new file as the default selected
				if(this._uploadedName) {
					for(var i = 0; i < data.files.length; i++) {
						if(data.files[i].fileName == this._uploadedName) {
							this.setState({filenameI: i});
							break;
						}
					}
				}
			}

		}.bind(this));

		// Get initial list of files
		this.load_filelist();

		this.forceUpdate();
	},

	send: function(msg) {
		Socket.emit('msg', JSON.stringify(msg));
	},


	load: function(filename, cue) {
		var message = {
			type: 'load',
			startPoint: this.state.cue*1,
			jocsPath: 'data/' + this.state.availableFiles[this.state.filenameI].fileName,
			theaterScale: this.state.scale
		};
		this.send(message);
	},

	load_filelist: function() {
		Socket.emit('msg', JSON.stringify({ type: 'list' }));
	},

	prepare: function() { this.send({ type: 'prepare' }); },
	play: function() { this.send({ type: 'play' }); },
	pause: function() { this.send({ type: 'pause' }); },
	stop: function() { this.send({ type: 'stop' }); },
	kill: function() {
		this.send({ type: 'kill', enabled: true });
	},

	upload: function(name, text) {
		Socket.emit('upload', { name: name, data: text });
		Socket.once('upload_reply', function() {
			this.load_filelist();
			this._uploadedName = name;
		}.bind(this))
	},

	changeView: function(name){

		var cam = this.renderer.camera;

		if(name == 'top') {
			cam.rotation.x = 0;
			cam.rotation.y = 0;
			cam.rotation.z = Math.Pi;

			cam.position.y = 0;
			cam.position.x = 0;
			cam.position.z = 6;
		}
		else if(name == 'front') {
			cam.rotation.x = 0;
			cam.rotation.y = 0;
			cam.rotation.z = 0;

			cam.position.y = -6;
			cam.position.x = 0;
			cam.position.z = 0;
		}
		else if(name == 'right') {
			cam.position.y = 0;
			cam.position.x = 6;
			cam.position.z = 0;
		}

		this.renderer.controls.update();
		this.renderer.render();


	},

	render: function(){

		return (
			<div className="ts-app">
				<Navbar />

				<table style={{width: '100%', height: '100%', backgroundColor: '#444', color: '#fff'}}>
					<tbody>
						<tr>
							<td>
								<table style={{width: '100%', height: '100%'}}>
									<tbody>
										<tr>
											<td style={{width: 250, borderRight: '2px solid #222', verticalAlign: 'top'}}>
												{/*
													Drone list
													- I should be able to add another drone
													- Double click on to open up a modal for more settings
													- 'SetHome' by dragging in the world (or expert coordinates)
												*/}

												<PropertiesPane parent={this} />
											</td>
											<td style={{position: 'relative'}}>
												<div style={{position: 'absolute', top: 10, right: 10}}>
													<div className="btn-group">
														<button onClick={() => this.changeView('top')} className="btn btn-default">Top</button>
														<button onClick={() => this.changeView('front')} className="btn btn-default">Front</button>
														<button onClick={() => this.changeView('right')} className="btn btn-default">Right</button>
													</div>


													<div style={{color: '#444'}}>
														<div>
															<input type="checkbox" checked={this.renderer? this.renderer.options.showTrajectories : false} onChange={(e) => { this.renderer.options.showTrajectories = e.target.checked; this.forceUpdate() } } /> Show Trajectory Line
														</div>
														<div>
															<input type="checkbox" checked={this.renderer? this.renderer.options.showVehicles : false} onChange={(e) => { this.renderer.options.showVehicles = e.target.checked; this.forceUpdate() } } /> Show Vehicles
														</div>

													</div>


												</div>

												{/*
												<div className="btn-group" style={{position: 'absolute', bottom: 10, right: 10}}>
													<button onClick={() => this.player.addPoint()} className="btn btn-default">+</button>
												</div>
												*/}
												{/*
													For viewing the current state of the drones,
													We will also want to:
													- highlight which drones are active in the current plan
													- show the stage bounds as a square grid
													- allow drones to to dragged around (transform active point)
													- stroke the current proposed trajectories for point-to-point plans
												*/}
												<WorldView onRendererReady={this.onRendererReady} />
											</td>
										</tr>
									</tbody>
								</table>
							</td>
						</tr>


						{/*
						<tr>
							<td style={{height: 200, borderTop: '2px solid #222'}}>
								{/*
									Playback controls and timeline here

									The timeline will allow the generation of plans.



									For the timeline, we will have multiple tabs o.
									- The first tab will be the root plan
									- Each sequential tab will be

								*/}
						{/*
								<Timeline parent={this} />

							</td>
						</tr>
						*/}
					</tbody>

				</table>

			</div>
		);

	}

});

module.exports = App;
