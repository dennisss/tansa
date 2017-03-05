import * as React from 'react';

interface ScaleSelectorProps {
	handleChange(newVal: number): void
}

interface ScaleSelectorState {
	value: string
}

export class ScaleSelector extends React.Component<ScaleSelectorProps, ScaleSelectorState> {
	constructor(props: ScaleSelectorProps) {
		super(props);
		this.state = { value: '1.0' };
		this.handleChange = this.handleChange.bind(this);
	}

	/**
	 * Update the component's state with the value of the input whenever it changes.
	 * If the input is a number, update the parent's state (ie: the scale value of the form).
	 * The distinction is needed because this event will fire for every character being entered (ie: the decimal point).
	 */
	private handleChange(event: React.ChangeEvent<HTMLInputElement>): void {
		const scale = +event.currentTarget.value;
		this.setState({ value: event.currentTarget.value });

		if (!isNaN(scale)) {
			this.props.handleChange(scale);
		}
	}

	public render(): JSX.Element {
		return(
			<div className='col-xs-2 col-md-1'>
				<div className='form-group label-floating text-left'>
					<label htmlFor='scale' className='control-label'>Scale</label>
					<input id='scale' className='form-control' value={this.state.value} onChange={this.handleChange} />
				</div>
			</div>
		);
	}
}
