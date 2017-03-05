import * as React from 'react';

interface VehicleStatusIconProps {
	name: string,
	iconClass: string
}

export class VehicleStatusIcon extends React.Component<VehicleStatusIconProps, undefined> {
	constructor(props: VehicleStatusIconProps) {
		super(props);
	}

	public shouldComponentUpdate(nextProps: VehicleStatusIconProps): boolean {
		return nextProps.iconClass !== this.props.iconClass;
	}

	public render(): JSX.Element {
		const className = 'col-xs-2 col-md-2 text-center';

		return (
			<div className='col-xs-2 col-md-2 text-center'>
				{this.props.name}:  <i className={this.props.iconClass} aria-hidden='true'></i>
			</div>
		);
	}
}
