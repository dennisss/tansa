
var React = require('react'),
	WorldView = require('./world'),
	Timeline = require('./timeline'),
	PropertiesPane = require('./properties'),
	CamerasPane = require('./cameras');


var MainPage = React.createClass({

	render: function() {

		var p = this.props.parent;

		return (

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

											<PropertiesPane parent={p} />
										</td>
										<td style={{position: 'relative'}}>

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
							<CamerasPane />
						</td>
					</tr>


					<tr>

						{/* TODO: This is originally 200px high */}
						<td style={{height: 40, borderTop: '2px solid #222'}}>
							{/*
								Playback controls and timeline here

								The timeline will allow the generation of plans.



								For the timeline, we will have multiple tabs o.
								- The first tab will be the root plan
								- Each sequential tab will be

								*/}
								<Timeline parent={p} />

							</td>
						</tr>

				</tbody>

			</table>

		);

	}


});

module.exports = CommandPage;
