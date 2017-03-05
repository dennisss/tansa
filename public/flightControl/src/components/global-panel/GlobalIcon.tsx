import * as React from 'react';

interface GlobalIconProps {
	colClass: string,
	icon: string,
	text: string
}

export class GlobalIcon extends React.Component<GlobalIconProps, undefined> {
	constructor(props: GlobalIconProps) {
		super(props);
	}

	public shouldComponentUpdate(nextProps: GlobalIconProps): boolean {
		return nextProps.text !== this.props.text;
	}

	public render(): JSX.Element {
		return (
			<div className={this.props.colClass}>
				{this.props.text}  <i className={this.props.icon} aria-hidden='true' />
			</div>
		);
	}
}
