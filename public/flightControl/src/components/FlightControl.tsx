import * as React from 'react';
import * as io from 'socket.io-client';
import { GlobalStatus, Vehicle } from '../messaging/dtos';
import { PreFlightPanel } from './pre-flight-panel/PreFlightPanel';
import { VehiclePanel } from './vehicle-panel/VehiclePanel';
import { FlightPanel } from './flight-panel/FlightPanel';
import { GlobalStatusPanel } from './global-panel/GlobalPanel';

interface FlightControlState {
	global: GlobalStatus,
	socket: SocketIOClient.Socket,
	vehicles: Array<Vehicle>
}

export class FlightControl extends React.Component<undefined, FlightControlState> {
	constructor() {
		super();

		this.state = {
			global: {
				initialized: false,
				landed: false,
				paused: false,
				playing: false,
				time: 0,
				ready: false
			},
			socket: io(),
			vehicles: []
		};

		this.state.socket.on('status', this.processStatusMessage.bind(this));
	}

	// Event Handlers
	private processStatusMessage(message: any): void {
		const json = JSON.parse(message);

		if (json && json.global && json.vehicles) {
			const global: GlobalStatus = json.global;
			const vehicles: Array<Vehicle> = json.vehicles;

			this.setState({ global: global, vehicles: vehicles });
		} else {
			alert('Status Message shape is invalid.');
		}
	}

	public render(): JSX.Element {
		const panel = (this.state.global.ready || this.state.global.playing || this.state.global.paused) ?
			<FlightPanel socket={this.state.socket} globalStatus={this.state.global} /> :
			<PreFlightPanel socket={this.state.socket} initialized={this.state.global.initialized} />;

		return (
			<div>
				<h1 className="text-center">Dancing Drones Command Center</h1>
				<div className='container-fluid'>
					{panel}
					<GlobalStatusPanel status={this.state.global} />
					<br />
					<VehiclePanel vehicles={this.state.vehicles} />
				</div>
			</div>
		);
	}
}
