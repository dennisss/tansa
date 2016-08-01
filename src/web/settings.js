var React = require('react');

import { Modal } from 'react-bootstrap';


var SettingsModal = React.createClass({



	render: function(){

		return (
			<Modal {...this.props} bsSize="lg">
				<Modal.Header closeButton>
					<Modal.Title>Settings</Modal.Title>
				</Modal.Header>

				<Modal.Body>
					{/*
						Drone configuration list
						- Set Home options

						Change theater configuration
						- Select from preset list
						- Or, allow custom entry
					*/}

				</Modal.Body>
			</Modal>
		);

	}



});

module.exports = SettingsModal;
