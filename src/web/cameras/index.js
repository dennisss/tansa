var React = require('react'),
	CameraPreview = require('./preview'),
	Socket = require('../socket');

/**
 * For showing side-by-side all the
 */
var CamerasPane = React.createClass({

	componentWillMount: function() {

		this._cameras = [];
		this._packets = {};

		Socket.msgs.on('camera_list', (data) => {
			this._cameras = data.cameras;
			this.forceUpdate();
		});

		Socket.msgs.on('camera_packet', (data) => {
			this._packets[data.id] = data;
			this.forceUpdate();
		});
	},


	render: function() {
		return (
			<div className="ta-pane" style={{padding: 10, width: '100%', height: '100%'}}>
				{this._cameras.map((c, i) => {
					return <CameraPreview key={c.id} camera={c} packet={this._packets[c.id]} />
				})}
				{this._cameras.length == 0? (
					<div style={{textAlign: 'center', padding: 40, color: '#ccc'}}>No cameras connected</div>
				) : null}
			</div>
		);
	}


});

module.exports = CamerasPane;
