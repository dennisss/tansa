var React = require('react'),
	Navbar = require('./navbar'),
	Socket = require('./socket'),
	Settings = require('./settings'),
	MainPage = require('./main');

import getMuiTheme from 'material-ui/styles/getMuiTheme';


Settings.setup({}, {
	file: {
		roles: '0,1,2,3,4,5',
		ids: '0,1,2,3,4,5',
		name: null,
		cue: null
	},
	trajectory: {
		forward: 10,
		backward: 10
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
			connected: false,

			calibrating: false
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


	// TODO: Some of this logic should be moved
	onRendererReady: function(renderer){
		this.renderer = renderer;

		var it = 0;
		Socket.msgs.on('status', (data) => {
			if((it++) % 20 == 0) {
				this.setState({ stats: data, time: (new Date()) });
			}
			renderer.update({vehicles: data.vehicles, time: data.time});
		});

		Socket.msgs.on('load_reply', (data) => {
			renderer.setPaths(data.paths);
			renderer.setHomes(data.target_positions);
			this.setState({ duration: data.duration });
		})

		Socket.msgs.on('list_reply', (data) => {
			this.setState({availableFiles: data.files});

			var activeName = this._uploadedName || Settings.get('file.name');
			// Right after an upload, set the new file as the default selected
			if(activeName) {
				for(var i = 0; i < data.files.length; i++) {
					if(data.files[i].fileName == activeName) {
						this.setState({filenameI: i});

						var activeCue = Settings.get('file.cue');
						if(activeCue) {
							var cues = data.files[i].breakpoints;
							for(var j = 0; j < cues.length; j++) {
								if(cues[j].number == activeCue) {
									this.setState({cue: activeCue});
									break;
								}
							}
						}


						break;
					}
				}
			}
		});

		Socket.msgs.on('calibrate_reply', (data) => {
			this.setState({calibrating: false});
		});


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
			routinePath: 'data/' + this.state.availableFiles[this.state.filenameI].fileName,
			theaterScale: this.state.scale // TODO: Rename this just a regular scale
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
	kill: function(role) {
		this.send({ type: 'kill', enabled: true, role: (role || -1) });
	},
	halt: function(role) {
		this.send({ type: 'halt', role: (role || -1) });
	},

	calibrate: function() {
		this.send({ type: 'calibrate' });
		this.setState({calibrating: true});
	},

	resync: function() { this.send({ type: 'resync' }); },
	rearrange: function() { this.send({ type: 'rearrange' }); },

	upload: function(name, text) {
		Socket.emit('upload', { name: name, data: text });
		Socket.once('upload_reply', function() {
			this.load_filelist();
			this._uploadedName = name;
		}.bind(this))
	},

	render: function(){

		return (
			<div className="ts-app">
				<Navbar parent={this} />

				<MainPage parent={this} />
			</div>
		);

	}

});

module.exports = App;
