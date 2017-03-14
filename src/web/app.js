var React = require('react'),
	Navbar = require('./navbar'),
	WorldView = require('./world'),
	Timeline = require('./timeline'),
	PropertiesPane = require('./properties'),
	Socket = require('./socket'),
	Settings = require('./settings');

import getMuiTheme from 'material-ui/styles/getMuiTheme';


Settings.setup({}, {
	file: {
		roles: '0,1,2,3,4,5',
		ids: '0,1,2,3,4,5'
	}

})


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

			stats: null, // The full object from the 'status' message
			duration: null,
			time: 0,
			connected: false
		}

	},


	componentWillMount: function() {
		// Periodically check if we are connected
		this._interval = setInterval(() => {
			var t = new Date();
			var connected = (t - this.state.time) / 1000 < 3;

			if(this.state.connected != connected) {
				this.setState({connected: connected});

				if(connected) {
					this.load_filelist();
				}
			}
		}, 250);

	},

	componentWillUnmount: function() {
		clearInterval(this._interval);
	},


	onRendererReady: function(renderer){
		this.renderer = renderer;

		var it = 0;
		Socket.on('msg', function(data) {
			data = JSON.parse(data);

			if(data.type == 'status') {
				if((it++) % 10 == 0) {
					this.setState({ stats: data, time: (new Date()) });
				}
				renderer.update({vehicles: data.vehicles});
			}
			else if(data.type == 'load_reply') {
				renderer.setPaths(data.paths);
				renderer.setHomes(data.target_positions);
				this.setState({ duration: data.duration });
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

	load: function(e, previewing) {
		var message = {
			type: 'load',
			preview: previewing? true : false,
			startPoint: this.state.cue*1,
			jocsPath: 'data/' + this.state.availableFiles[this.state.filenameI].fileName,
			theaterScale: this.state.scale
		};

		var s = Settings.get('file');

		message.activeRoles = s.roles.split(',').map((n) => n*1);

		if(this.state.stats.global.mode == 'real') {
			message.vehicles = s.ids.split(',').map((n) => ({ net_id: n*1 }));
		}

		this.send(message);
	},

	// TODO: This should mimic load
	preview: function() {
		this.load(null, true);
	},

	seek: function(t) {
		this.send({ type: 'seek', time: t });
	},

	// TODO: Do this automaticaly when we detect the server is connected
	load_filelist: function() {
		this.send({ type: 'list' });
	},

	prepare: function() { this.send({ type: 'prepare' }); },
	land: function() { this.send({ type: 'land' }); },
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
				<Navbar parent={this} />

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
												<div style={{position: 'absolute', top: 10, right: 10, backgroundColor: '#fff', padding: 8, border: '1px solid #ccc', borderRadius: 4}}>
													<div className="btn-group">
														<button onClick={() => this.changeView('top')} className="btn btn-default">Top</button>
														<button onClick={() => this.changeView('front')} className="btn btn-default">Front</button>
														<button onClick={() => this.changeView('right')} className="btn btn-default">Right</button>
													</div>


													<div style={{color: '#444', marginTop: 5}}>
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


						<tr>
							{/* TODO: This is originally 200px high */}
							<td style={{height: 36, borderTop: '2px solid #222'}}>
								{/*
									Playback controls and timeline here

									The timeline will allow the generation of plans.



									For the timeline, we will have multiple tabs o.
									- The first tab will be the root plan
									- Each sequential tab will be

								*/}
								<Timeline parent={this} />

							</td>
						</tr>

					</tbody>

				</table>

			</div>
		);

	}

});

module.exports = App;
