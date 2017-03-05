import * as React from 'react';
import { BatteryIcon } from './BatteryIcon';
import { TrackSelector } from './TrackSelector';
import { VehicleIdIcon } from './VehicleIdIcon';
import { VehicleStatusIcon } from './VehicleStatusIcon';
import { Vehicle } from '../../messaging/dtos';

interface PanelProps {
	vehicles: Array<Vehicle>
}

export class VehiclePanel extends React.Component<PanelProps, undefined> {
	constructor(props: PanelProps) {
		super(props);
	}

	public shouldComponentUpdate(nextProps: PanelProps): boolean {
		return nextProps.vehicles.length > 0 || nextProps.vehicles.length !== this.props.vehicles.length;
	}

	public render(): JSX.Element {
		const drones = this.props.vehicles.map(vehicle => {
			const iconAttrs: Array<[string, boolean]> = [
				['Tracking', vehicle.tracking],
				['Connected', vehicle.connected],
				['Armed', vehicle.armed]
			];

			const statusIcons = iconAttrs.map(([name, status]) =>
				<VehicleStatusIcon key={name} name={name} iconClass={'fa fa-circle ' + (status ? 'green' : 'red')} />
			);

			return (
				<div className='row droneData' key={vehicle.id}>
					<VehicleIdIcon id={vehicle.id} />
					<TrackSelector />
					<BatteryIcon percent={vehicle.battery.percent} />
					{statusIcons}
				</div>
			);
		});

		return (<div>{ drones.length > 0 ? drones : <div>No drones connected.</div> }</div>);
	}
}
