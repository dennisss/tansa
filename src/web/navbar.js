var React = require('react');

import Toggle from 'material-ui/Toggle';


var Navbar = React.createClass({

	getInitialState: function(){
		return {
			mode: 'simulate'
		}

	},

	render: function(){
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

							<li className="dropdown">
								<a href="#" className="dropdown-toggle" data-toggle="dropdown" role="button" aria-haspopup="true" aria-expanded="false">File <span className="caret" /></a>
								<ul className="dropdown-menu">
									<li><a href="#">New</a></li>
									<li><a href="#">Open</a></li>
									<li><a href="#">Save</a></li>
								</ul>
							</li>

							<li><a href="#">Settings</a></li>

							<li className="dropdown">
								<a href="#" className="dropdown-toggle" data-toggle="dropdown" role="button" aria-haspopup="true" aria-expanded="false">
									Status
									<span style={{paddingLeft: 5, color: '#f00',}}>
										<i className="fa fa-circle" />
									</span>
								</a>
								<ul className="dropdown-menu">
									<li><a href="#">
										Mocap Ready
										<span style={{float: 'right', color: '#f00',}}>
											<i className="fa fa-circle" />
										</span>
									</a></li>
									<li><a href="#">
										Vehicle Link
										<span style={{float: 'right', color: '#f00',}}>
											<i className="fa fa-circle" />
										</span>
									</a></li>
									<li><a href="#">
										Object Sync
										<span style={{float: 'right', color: '#f00',}}>
											<i className="fa fa-circle" />
										</span>
									</a></li>
									<li><a href="#">
										Armed
										<span style={{float: 'right', color: '#f00',}}>
											<i className="fa fa-circle" />
										</span>
									</a></li>
									<li role="separator" className="divider" />
									<li><a href="#">More...</a></li>
								</ul>
							</li>


							<li><a href="#"></a></li>
						</ul>
						<ul className="nav navbar-nav navbar-right">
							{/*
							<li><a href="#">Link</a></li>
							<li className="dropdown">
								<a href="#" className="dropdown-toggle" data-toggle="dropdown" role="button" aria-haspopup="true" aria-expanded="false">Dropdown <span className="caret" /></a>
								<ul className="dropdown-menu">
									<li><a href="#">Action</a></li>
									<li><a href="#">Another action</a></li>
									<li><a href="#">Something else here</a></li>
									<li role="separator" className="divider" />
									<li><a href="#">Separated link</a></li>
								</ul>
							</li>
							*/}
							<li>
								<a>
									{this.state.mode == 'simulate'? 'Simulating' : 'Live'}
								</a>
							</li>
							<li>
								<div style={{display: 'inline-block', verticalAlign: 'middle', padding: 13, paddingLeft: 0}}>
									<Toggle toggled={this.state.mode == 'live'} onToggle={(e,v) => this.setState({mode: v?'live' : 'simulate'})} />
								</div>
								{/* TODO: Add kill switch here */}
							</li>

						</ul>
					</div>
				</div>
			</nav>
		);
	}


});

module.exports = Navbar;
