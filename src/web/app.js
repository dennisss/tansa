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


	onPlayerReady: function(player){
		this.player = player;

		Socket.on('msg', function(data) {
			data = JSON.parse(data);

			if(data.type == 'status') {

				player.update({vehicles: data.vehicles});



			}


		})



	},

	changeView: function(name){

		var cam = this.player.camera;

		if(name == 'top'){
			cam.rotation.x = 0;
			cam.rotation.y = 0; //Math.Pi;
			cam.rotation.z = Math.Pi; //-Math.Pi/ 2;

			cam.position.y = 0;
			cam.position.x = 0;
			cam.position.z = 6;
		}
		else if(name == 'front'){
			cam.rotation.x = -Math.Pi / 2;
			cam.rotation.y = 0;
			cam.rotation.z = 0;


			cam.position.y = 6;
			cam.position.x = 0;
			cam.position.z = 0;

		}

		this.player.controls.update();
		this.player.render();


	},

	render: function(){

		// For just showing viewer
		return (
			<div className="ts-app">
				<WorldView onPlayerReady={this.onPlayerReady} />
			</div>
		)

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
											<td style={{width: 200, borderRight: '2px solid #222', verticalAlign: 'top'}}>
												{/*
													Drone list
													- I should be able to add another drone
													- Double click on to open up a modal for more settings
													- 'SetHome' by dragging in the world (or expert coordinates)
												*/}
												<PropertiesPane />
											</td>
											<td style={{position: 'relative'}}>

												<div className="btn-group" style={{position: 'absolute', top: 10, right: 10}}>
													<button onClick={() => this.changeView('top')} className="btn btn-default">Top</button>
													<button onClick={() => this.changeView('front')} className="btn btn-default">Front</button>
													<button onClick={() => this.changeView('right')} className="btn btn-default">Right</button>
												</div>

												<div className="btn-group" style={{position: 'absolute', bottom: 10, right: 10}}>
													<button onClick={() => this.player.addPoint()} className="btn btn-default">+</button>
												</div>

												{/*
													For viewing the current state of the drones,
													We will also want to:
													- highlight which drones are active in the current plan
													- show the stage bounds as a square grid
													- allow drones to to dragged around (transform active point)
													- stroke the current proposed trajectories for point-to-point plans
												*/}
												<WorldView onPlayerReady={this.onPlayerReady} />
											</td>
										</tr>
									</tbody>
								</table>
							</td>
						</tr>

						<tr>
							<td style={{height: 200, borderTop: '2px solid #222'}}>
								{/*
									Playback controls and timeline here

									The timeline will allow the generation of plans.



									For the timeline, we will have multiple tabs o.
									- The first tab will be the root plan
									- Each sequential tab will be

								*/}

								<Timeline />

							</td>
						</tr>
					</tbody>

				</table>

			</div>
		);

	}

});

module.exports = App;
