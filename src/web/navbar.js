var React = require('react'),
	StatusSign = require('./statusSign'),
	SettingsModal = require('./settingsModal'),
	LogButton = require('./log');

import Toggle from 'material-ui/Toggle';


var Navbar = React.createClass({

	render: function(){

		var p = this.props.parent;


		return (
			<nav className="navbar navbar-inverse navbar-fixed-top">
				<div className="container-fluid">
					{/* Brand and toggle get grouped for better mobile display */}
					<div className="navbar-header">
						<button type="button" className="navbar-toggle collapsed" data-toggle="collapse" data-target="#bs-example-navbar-collapse-1" aria-expanded="false">
							<span className="sr-only">Toggle navigation</span>
							<span className="icon-bar" />
							<span className="icon-bar" />
							<span className="icon-bar" />
						</button>
						<a className="navbar-brand" href="#">Tansa</a>
					</div>
					{/* Collect the nav links, forms, and other content for toggling */}
					<div className="collapse navbar-collapse" id="bs-example-navbar-collapse-1">
						<ul className="nav navbar-nav">

							{/*
							<li className="dropdown">
								<a href="#" className="dropdown-toggle" data-toggle="dropdown" role="button" aria-haspopup="true" aria-expanded="false">File <span className="caret" /></a>
								<ul className="dropdown-menu">
									<li><a href="#">New</a></li>
									<li><a href="#">Open</a></li>
									<li><a href="#">Save</a></li>
								</ul>
							</li>
							*/}

							<SettingsButton />

							<LogButton />

							<li className="dropdown">
								<a href="#" className="dropdown-toggle" data-toggle="dropdown" role="button" aria-haspopup="true" aria-expanded="false">
									<StatusSign />
									Status
								</a>
								<ul className="dropdown-menu">
									<li><a href="#">
										<StatusSign />
										Mocap Ready
									</a></li>
									<li><a href="#">
										<StatusSign />
										Vehicle Link
									</a></li>
									<li><a href="#">
										<StatusSign />
										Object Sync
									</a></li>
									<li><a href="#">
										<StatusSign />
										Armed
									</a></li>
									<li role="separator" className="divider" />
									<li><a href="#">More...</a></li>
								</ul>
							</li>


							<li><a href="#"></a></li>
						</ul>
						{p.state.connected? (
							<ul className="nav navbar-nav navbar-right">
								<li>
									<a>
										{p.state.stats.global.mode == 'real'? 'Live' : 'Simulating'}
									</a>
								</li>
								<li>
									<div style={{display: 'inline-block', verticalAlign: 'middle', padding: 13, paddingLeft: 0}}>
										<Toggle toggled={p.state.stats.global.mode == 'real'} onToggle={(e,v) => false} />
									</div>
									{/* TODO: Add kill switch here */}
								</li>

							</ul>
						) : (
							<ul className="nav navbar-nav navbar-right">
								<li><a>Server Timeout</a></li>
							</ul>
						)}
					</div>
				</div>
			</nav>
		);
	}


});


var SettingsButton = React.createClass({

	getInitialState: function() {
		return {
			open: false
		};
	},

	open: function() {
		this.setState({open: true});
	},

	close: function() {
		this.setState({open: false});
	},

	render: function() {

		return (
			<li>
				<a href="#" onClick={this.open}>Settings</a>

				<SettingsModal show={this.state.open} onHide={this.close} />
			</li>
		);
	}

});

module.exports = Navbar;
