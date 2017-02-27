import * as React from 'react';
import { Button } from './Button';

interface KillButtonProps {
	class: string,
	socket: SocketIOClient.Socket
}

export class KillButton extends React.Component<KillButtonProps, undefined> {
	constructor(props: KillButtonProps) {
		super(props);
		this.sendKillMessage = this.sendKillMessage.bind(this);
	}

	private sendKillMessage(): void {
		this.props.socket.emit('msg', JSON.stringify({ type: 'kill', enabled: true }));
	}

	public render(): JSX.Element {
		return <Button containerClass={this.props.class} iconClass='fa fa-bomb' cb={this.sendKillMessage} disabled={false} />;
	}
}
