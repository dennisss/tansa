var React = require('react');

import Toggle from 'material-ui/Toggle';


var Navbar = React.createClass({

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
							<li className="active"><a href="#">Link <span className="sr-only">(current)</span></a></li>
							<li><a href="#">Link</a></li>
							<li className="dropdown">
								<a href="#" className="dropdown-toggle" data-toggle="dropdown" role="button" aria-haspopup="true" aria-expanded="false">Dropdown <span className="caret" /></a>
								<ul className="dropdown-menu">
									<li><a href="#">Action</a></li>
									<li><a href="#">Another action</a></li>
									<li><a href="#">Something else here</a></li>
									<li role="separator" className="divider" />
									<li><a href="#">Separated link</a></li>
									<li role="separator" className="divider" />
									<li><a href="#">One more separated link</a></li>
								</ul>
							</li>
						</ul>
						<ul className="nav navbar-nav navbar-right">
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
							<li>
								<a>
									Simulating
								</a>
							</li>
							<li>
								<div style={{display: 'inline-block', verticalAlign: 'middle', padding: 13, paddingLeft: 0}}>
									<Toggle />
								</div>
							</li>

						</ul>
					</div>
				</div>
			</nav>
		);
	}


});

module.exports = Navbar;
