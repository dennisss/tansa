var React = require('react');


var OverlayTools = React.createClass({


	render: function() {

		var p = this.props.parent;

		return (
			<div style={{position: 'absolute', top: 10, right: 10, backgroundColor: '#fff', padding: 8, border: '1px solid #ccc', borderRadius: 4}}>
				<div className="btn-group">
					<button onClick={() => p.changeView('top')} className="btn btn-default">Top</button>
					<button onClick={() => p.changeView('front')} className="btn btn-default">Front</button>
					<button onClick={() => p.changeView('right')} className="btn btn-default">Right</button>
				</div>


				<div style={{color: '#444', marginTop: 5}}>
					<div>
						<input type="checkbox" checked={p.renderer? p.renderer.options.showTrajectories : false} onChange={(e) => { p.renderer.options.showTrajectories = e.target.checked; p.forceUpdate() } } /> Show Trajectory Line
					</div>
					<div>
						<input type="checkbox" checked={p.renderer? p.renderer.options.showVehicles : false} onChange={(e) => { p.renderer.options.showVehicles = e.target.checked; p.forceUpdate() } } /> Show Vehicles
					</div>
					<div>
						<input type="checkbox" checked={p.renderer? p.renderer.options.cameraMode == 'perspective' : false} onChange={(e) => { p.renderer.setCamera(e.target.checked? 'perspective' : 'orthogonal'); p.forceUpdate() } } /> Perspective
					</div>

				</div>


			</div>
		);


	}


});


module.exports = OverlayTools;
