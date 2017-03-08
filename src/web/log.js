var React = require('react'),
	Socket = require('./socket');

import { Popover, OverlayTrigger } from 'react-bootstrap';

var LogButton = React.createClass({

	getInitialState: function() {
		return {
			messages: [],
			unread: 0
		}
	},

	componentWillMount: function() {

		Socket.on('msg', function(data) {


		})

	},

	render: function() {

		var tip = (
			<Popover id="log">
				<div style={{maxHeight: 300, overflowY: 'scroll'}}>
					<table className="table table-striped">
						<tbody>
							{this.state.messages.map((m) => (
								<tr>
									<td>{m}</td>
								</tr>
							))}
						</tbody>
					</table>
				</div>
			</Popover>
		);

		return (
			<li>
				<OverlayTrigger overlay={tip} placment='bottom' >
					<a href="#">
						Log
						{this.state.unread > 0? (
							<span style={{marginLeft: 5}} className="badge">{this.state.unread}</span>
						) : null}
					</a>
				</OverlayTrigger>
			</li>
		);

	}


});

module.exports = LogButton;
