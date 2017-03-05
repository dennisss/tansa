import * as React from 'react';

interface BatteryIconProps {
	percent: number
}

export class BatteryIcon extends React.Component<BatteryIconProps, undefined> {
	constructor(props: BatteryIconProps) {
		super(props);
	}

	public shouldComponentUpdate(nextProps: BatteryIconProps) {
		return nextProps.percent !== this.props.percent;
	}

	public render(): JSX.Element {
		const batteryLevel = Math.floor(this.props.percent * 100);
		const batteryClass = 'col-xs-2 col-md-2 visible-md visible-lg text-center ' + (batteryLevel > 30 ? 'green' : 'red');

		return (
			<div className={batteryClass}>
				Battery:  {batteryLevel}%
			</div>
		);
	}
}
