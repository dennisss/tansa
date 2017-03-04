var React = require('react');

require('./style.css')

var PropertiesPane = React.createClass({

	render: function(){


		return (
			<div className="ta-pane">



				<div className="ta-pane-header">
					Properties

					<div style={{float: 'right'}}>
						<i className="fa fa-caret-square-o-left" />
					</div>
				</div>
				<div class="ta-pane-body">

					<div>
						<select></select>


					</div>



					<div className="ta-props" style={{fontSize: 12}}>
						<div className="ta-props-header">
							<i className="fa fa-minus-square-o" />&nbsp;
							Settings
						</div>
						<div>
							<div className="ta-prop">
								Speed:
								<div style={{float: 'right'}}>
									<input className="transparent-input" style={{width: 40}} type="number" value={35} />%
								</div>
							</div>
						</div>


					</div>

					<div className="ta-props" style={{fontSize: 12}}>
						<div className="ta-props-header">
							<i className="fa fa-minus-square-o" />&nbsp;
							Points
						</div>
						<div>
							{[1,2,3].map(function(pt, i){

								return (
									<div className="ta-prop">
										<span style={{fontWeight: 'bold', paddingRight: 10}}>
											{i+1}:
										</span>
										<span>
											x: <input className="transparent-input" style={{width: 40}} type="number" value={0} />
										</span>
										<span>
											y: <input className="transparent-input" style={{width: 40}} type="number" value={0} />
										</span>
										<span>
											z: <input className="transparent-input" style={{width: 40}} type="number" value={2} />
										</span>
										<span style={{float: 'right'}}>
											<i className="fa fa-times" />
										</span>
										<br />
										<span style={{paddingLeft: 20}}>
											Time: <input className="transparent-input" style={{width: 80}} type="number" value={0.43} />s
										</span>
									</div>
								);

							})}
						</div>


					</div>



				</div>
			</div>
		);
	}

});

module.exports = PropertiesPane;
