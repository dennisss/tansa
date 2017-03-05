import * as React from 'react';

interface VehicleIdIconProps {
	id: number
}

export class VehicleIdIcon extends React.Component<VehicleIdIconProps, undefined> {
	constructor(props: VehicleIdIconProps) {
		super(props);
	}

	public shouldComponentUpdate(nextProps: VehicleIdIconProps): boolean {
		return nextProps.id !== this.props.id;
	}

	public render(): JSX.Element {
		return (
			<div className='col-xs-2 col-md-2 noPad'>
				<i className='fa fa-plane' aria-hidden='true'></i> {this.props.id}
			</div>
		);
	}
}
