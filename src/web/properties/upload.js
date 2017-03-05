var React = require('react');

import { Button, Modal } from 'react-bootstrap';

var UploadButton = React.createClass({

	getInitialState: function() {
		return {
			_open: false
		};
	},

	close: function() {
		this.setState({ _open: false });
	},

	open: function() {
		this.setState({ _open: true });
	},

	render: function() {


		return (
			<button className="btn btn-default" onClick={this.open}>
				<i className="fa fa-cloud-upload" />
				<UploadModal show={this.state._open} parent={this.props.parent} onHide={this.close} />
			</button>
		);

	}


});

module.exports = UploadButton;


var UploadModal = React.createClass({

	getInitialState: function() {
		return {
			loading: false
		};
	},

	submit: function() {
		var p = this.props.parent;

		var el = this.refs.file;

		if(el.files.length != 1) {
			alert('Please Select one file');
			return;
		}

		this.setState({loading: true});

		var f = el.files[0];

		var reader = new FileReader();

		var self = this;
		reader.onload = function(e) {
			p.upload(f.name, e.target.result);
			self.props.onHide();
			self.setState({loading: false});
		}

		reader.readAsText(f);
	},

	render: function() {
		return (

			<Modal {...this.props}>
				<Modal.Header closeButton>
					<Modal.Title>Upload a file</Modal.Title>
				</Modal.Header>
				<Modal.Body>
					<input type="file" ref="file" />
				</Modal.Body>
				<Modal.Footer>
					<button disabled={this.state.loading} className="btn btn-primary" onClick={this.submit} >
						{this.state.loading? (
							<i className="fa fa-circle-o-notch fa-spin"></i>
						) : (
							'Submit'
						)}
					</button>
					<Button onClick={this.props.onHide}>Close</Button>
				</Modal.Footer>
			</Modal>
		)
	}

})
