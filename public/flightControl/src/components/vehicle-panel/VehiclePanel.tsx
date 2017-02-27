import * as React from 'react';
import { Vehicle } from '../../messaging/dtos';

interface PanelProps {
	vehicles: Array<Vehicle>
}

export class VehiclePanel extends React.Component<PanelProps, undefined> {    
	constructor(props: PanelProps) {
		super(props);
	}

	public render(): JSX.Element {
		const getStatusColor = (b: boolean) => b ? 'green' : 'red';

		const drones = this.props.vehicles.map(vehicle => {
			const batteryLevel = Math.floor(vehicle.battery.percent * 100);
			const batteryClass = 'col-xs-2 col-md-2 visible-md visible-lg text-center ' + getStatusColor(batteryLevel <= 30);

			return (
				<div className='row droneData' key={vehicle.id}>
					<div className='col-xs-2 col-md-2 noPad'>
						<i className='fa fa-plane' aria-hidden='true'></i> {vehicle.id}
					</div>
					<div className='col-xs-1 col-md-1 text-center'>
						<div className='form-group noMargin'>
							<select className='form-control'>
								<option>N/A</option>
							</select>
						</div>
					</div>
					<div className={batteryClass}>
						Battery:  {vehicle.battery.percent}%
					</div>
					<div className='col-xs-2 col-md-2 text-center'>
						Tracking:  <i className={'fa fa-circle ' + getStatusColor(vehicle.tracking)} aria-hidden='true'></i>
					</div>
					<div className='col-xs-2 col-md-2 text-center'>
						Connected:  <i className={'fa fa-circle ' + getStatusColor(vehicle.connected)} aria-hidden='true'></i>
					</div>
					<div className='col-xs-2 col-md-2 text-center'>
						Armed:  <i className={'fa fa-circle ' + getStatusColor(vehicle.armed)} aria-hidden='true'></i>
					</div>
				</div>
			);
		});

		return (
			<div>
				{ drones.length > 0 ? drones : <div>No drones connected.</div> }
			</div>
		);
	}
}
