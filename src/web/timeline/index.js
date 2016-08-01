var React = require('react'),
	ReactDOM = require('react-dom'),
	Controls = require('./controls');

require('./style.css');


var currentTime = 0;

/*
	Used to visualize a 'concurrent' track type
	- typically made up of tracks which each are a linear sequences of actions
*/
var Timeline = React.createClass({

	getInitialState: function(){
		return {
			activePlan: null
		};
	},


	componentWillMount: function(){

		this.tracks = [
			{
				type: 'Track',
				name: 'First',
				children: [
					{
						type: 'LinearMotion',
						offset: 0,
						points: []
					}
				]
			},
			{
				type: 'Track',
				name: 'Second',
				children: [
					{
						type: 'LinearMotion',
						offset: 30,
						points: []
					}
				]
			}


		]


	},


	renderTickMarkers: function(){
		// 100% is 5min
		var marks = [];


		for(var i = 1; i <= this.span; i++){
			var left = (this.percentPerSecond*i);
			var show = false, height = 0;


			if(i % 60 == 0){
				show = true;
				height = 100;
				marks.push(<div key={i + '-lbl'} style={{right: (100-left) + '%', top: 4, fontSize: 8, color: '#888', position: 'absolute', paddingRight: 4, top: 2}}>{i/60}m</div>)
			}
			else if(i % 30 == 0){
				show = true;
				height = 50;
			}
			else if(i % 15 == 0){
				show = true;
				height = 25;
			}

			if(show){
				marks.push(<div key={i} style={{height: (height + '%'), position: 'absolute', bottom: 0, left: left + '%', borderRight: '1px solid #888'}}></div>)
			}


		}

		return marks;

	},


	onLineClick: function(e){
		// When clicking in a random spot, it should take you to that spot in the timeline

		var $el = $(this.refs.line);

		var x = e.clientX - $el.offset().left; // TODO: Also account for timeline scroll

		currentTime = (this.span / $el.width()) * x;

		this.forceUpdate();
	},

	onPlanClick: function(e, p){
		console.log(p);
		this.setState({activePlan: p});
		e.stopPropagation();
	},


	renderTimebar: function(){
		return (
			<div style={{left: (currentTime*this.percentPerSecond) + '%', borderRight: '1px solid red', top: 0, bottom: 0, position: 'absolute'}}></div>
		);
	},

	render: function(){
		this.span = 5*60;

		this.percentPerSecond = 100 / this.span;


		return (
			<div style={{width: '100%', height: '100%', display: 'table'}}>
				<div style={{width: 300, display: 'table-cell', verticalAlign: 'top', borderRight: '1px solid #888'}}>

					<div className="ta-timeline-row" style={{borderBottom: '1px solid #888', backgroundColor: '#222'}}>
						<Controls />
					</div>

					{this.tracks.map(function(t, i){
						return (
							<div key={i} className="ta-timeline-row">
								<input type="text" value={t.name} className="transparent-input" onChange={(e) => { t.name = e.target.value; this.forceUpdate(); }} />
							</div>
						);
					})}
				</div>
				<div ref="line" style={{display: 'table-cell', verticalAlign: 'top', position: 'relative', cursor: 'text'}} onClick={this.onLineClick}>
					<div className="ta-timeline-row" style={{borderBottom: '1px solid #888', backgroundColor: '#222'}}>
						{this.renderTickMarkers()}
					</div>


					{this.tracks.map((t, i) => {
						return (
							<div key={i} className="ta-timeline-row">
								{t.children.map((st, i) => {

									var left = st.offset * this.percentPerSecond;

									return (
										<div key={i} style={{left: left + '%'}} onClick={(e) => this.onPlanClick(e, st)}
											className={"ta-timeline-block " + (this.state.activePlan == st? 'active' : '')}>
											<div style={{display: 'block', padding: '5px 4px'}}>{st.type}</div>
										</div>
									);
								})}
							</div>
						);
					})}

					{this.renderTimebar()}

				</div>
			</div>
		);


	}

});

module.exports = Timeline;
