import * as React from 'react';
import { Breakpoint } from '../../messaging/dtos';

interface BreakpointProps {
	breakpoints: Array<Breakpoint>,
	handleChange(newValue: number): void
}

interface BreakpointState {
	value: number
}

export class BreakpointSelector extends React.Component<BreakpointProps, BreakpointState> {
	constructor(props: BreakpointProps) {
		super(props);
		this.state = { value: 1 };
		this.breakpointSelected = this.breakpointSelected.bind(this);
	}

	private breakpointSelected(event: React.ChangeEvent<HTMLSelectElement>) {
		const newVal = +event.target.value || 1;
		this.setState({ value: newVal });
		this.props.handleChange(newVal);
	}

	public render(): JSX.Element {
		const options = this.props.breakpoints.map(breakpoint => <option value={breakpoint.number} key={breakpoint.number}>{breakpoint.name}</option>);

		return (
			<div className='col-xs-6 col-md-3'>
				<select className='form-control' value={this.state.value} onChange={this.breakpointSelected}>
					{ options.length > 0 ? options : <option value={this.state.value}>N/A</option> }
				</select>
			</div>
		);
	}
}
