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


	onRendererReady: function(renderer){
		this.renderer = renderer;

		Socket.on('msg', function(data) {
			data = JSON.parse(data);

			if(data.type == 'status') {
				renderer.update({vehicles: data.vehicles});
			}
			else if(data.type == 'load_reply') {
				renderer.setPaths(data.paths);
			}



		})


		this.forceUpdate();

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
												<WorldView onRendererReady={this.onRendererReady} />
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
