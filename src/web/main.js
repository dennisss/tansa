
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
											<WorldView onRendererReady={p.onRendererReady} />
										</td>
									</tr>
								</tbody>
							</table>
						</td>
					</tr>



					<tr>
						<td style={{height: 200, borderTop: '2px solid #222'}}>
							<TabbedPane>
								<Tab name="Cameras">
									<CamerasPane />
								</Tab>
								<Tab name="Timeline">
									{/*
										Playback controls and timeline here

										The timeline will allow the generation of plans.



										For the timeline, we will have multiple tabs o.
										- The first tab will be the root plan
										- Each sequential tab will be

										*/}
										<Timeline parent={p} />
								</Tab>

							</TabbedPane>

						</td>
					</tr>

				</tbody>

			</table>

		);

	}


});

module.exports = MainPage;


var TabbedPane = React.createClass({

	getInitialState: function() {
		return {
			_active: 0
		}
	},

	render: function() {

		var active = this.state._active;

		return (
			<table style={{width: '100%', height: '100%'}}>
				<tr>
					<td style={{borderBottom: '1px solid #ccc', height: 1}}>
						{this.props.children.map((tab, i) => {

							return (
								<div key={i} style={{display: 'inline-block', padding: '5px 10px', backgroundColor: (active == i? '#000' : null), cursor: 'pointer'}} onClick={() => this.setState({_active: i})}>
									{tab.props.name}
								</div>
							);
						})}
					</td>
				</tr>
				<tr>
					<td style={{verticalAlign: 'top'}}>{this.props.children[this.state._active]}</td>
				</tr>
			</table>

		);


	}


});

var Tab = React.createClass({

	render: function() {
		return (
			<div>
				{this.props.children}
			</div>
		);
	}
})
