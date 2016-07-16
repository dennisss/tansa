var React = require('react'),
	Navbar = require('./navbar'),
	WorldView = require('./world');

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

	render: function(){

		return (
			<div className="ts-app">
				<Navbar />

				<table style={{width: '100%', height: '100%'}}>
					<tbody>
						<tr>
							<td>
								<table style={{width: '100%', height: '100%'}}>
									<tbody>
										<tr>
											<td style={{width: 200}}>
												{/*
													Drone list
													- I should be able to add another drone
													- Double click on to open up a modal for more settings
													- 'SetHome' by dragging in the world (or expert coordinates)
												*/}
											</td>
											<td>
												{/*
													For viewing the current state of the drones,
													We will also want to:
													- highlight which drones are active in the current plan
													- show the stage bounds as a square grid
													- allow drones to to dragged around (transform active point)
													- stroke the current proposed trajectories for point-to-point plans
												*/}
												<WorldView />
											</td>
										</tr>
									</tbody>
								</table>
							</td>
						</tr>

						<tr>
							<td style={{height: 200}}>
								{/*
									Playback controls and timeline here

									The timeline will allow the generation of plans.



									For the timeline, we will have multiple tabs o.
									- The first tab will be the root plan
									- Each sequential tab will be

								*/}

								hello

							</td>
						</tr>
					</tbody>

				</table>

			</div>
		);

	}

});

module.exports = App;
