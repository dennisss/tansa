var React = require('react'),
	CameraPreview = require('./preview');

/**
 * For showing side-by-side all the
 */
var CamerasPane = React.createClass({


	render: function() {
		return (
			<div className="ta-pane" style={{padding: 10}}>
				<CameraPreview />
			</div>
		);
	}


});

module.exports = CamerasPane;
