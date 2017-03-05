import * as React from 'react';

interface ButtonProps {
	cb: () => void,
	containerClass: string,
	disabled: boolean,
	iconClass: string
}

export class Button extends React.Component<ButtonProps, undefined> {
	constructor(props: ButtonProps) {
		super(props);
	}

	public shouldComponentUpdate(nextProps: ButtonProps): boolean {
		return (
			nextProps.containerClass !== this.props.containerClass ||
			nextProps.disabled !== this.props.disabled ||
			nextProps.iconClass !== this.props.iconClass
		);
	}

	public render(): JSX.Element {
		return (
			<div className={this.props.containerClass}>
				<button className='btn btn-fab actionFab' disabled={this.props.disabled} onClick={this.props.cb}>
					<i className={this.props.iconClass} aria-hidden='true'></i>
				</button>
				<br/><br/>
			</div>
		);
	}
}
