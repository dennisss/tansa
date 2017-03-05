import * as React from 'react';
import { Button } from '../Button';
import { KillButton } from '../KillButton';
import { GlobalStatus } from '../../messaging/dtos';

interface FlightPanelProps {
	globalStatus: GlobalStatus,
	socket: SocketIOClient.Socket
}

export class FlightPanel extends React.Component<FlightPanelProps, undefined> {
	constructor(props: FlightPanelProps) {
		super(props);
	}

	public render(): JSX.Element {
		const containerClass = 'col-md-3 col-xs-6';
		const callback = (type: string) => () => this.props.socket.emit('msg', JSON.stringify({ type: type }));

		const buttonAttributes: Array<[string, () => void, boolean]> = [
			['glyphicon glyphicon-play', callback('play'), !this.props.globalStatus.ready || this.props.globalStatus.playing],
			['fa fa-step-forward', callback('pause'), !this.props.globalStatus.playing],
			['fa fa-download', callback('stop'), !this.props.globalStatus.paused]
		];

		const buttons = buttonAttributes.map(([iconClass, callback, disabled]) =>
			<Button key={iconClass} cb={callback} containerClass={containerClass} disabled={disabled} iconClass={iconClass} />
		);

		return (
			<div className='text-center gap row'>
				{buttons}
				<KillButton class={containerClass} socket={this.props.socket}/>
			</div>
		);
	}
}
