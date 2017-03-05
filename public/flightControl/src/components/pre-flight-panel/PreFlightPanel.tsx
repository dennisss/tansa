import * as React from 'react';
import { File } from '../../messaging/dtos';
import { FileSelector } from './FileSelector';
import { ScaleSelector } from './ScaleSelector';
import { BreakpointSelector } from './BreakPointSelector';
import { Button } from '../Button';
import { KillButton } from '../KillButton';

interface PreFlightProps {
	initialized: boolean,
	socket: SocketIOClient.Socket
}

interface PreFlightState {
	breakpoint: number,
	filename: string,
	files: Array<File>,
	scale: number
}

export class PreFlightPanel extends React.Component<PreFlightProps, PreFlightState> {
	constructor(props: PreFlightProps) {
		super(props);

		this.state = {
			breakpoint: 1,
			filename: '',
			files: [],
			scale: 1.0
		};

		this.handleSubmit = this.handleSubmit.bind(this);
		this.handleNewJocsFile = this.handleNewJocsFile.bind(this);
		this.props.socket.on('list_reply', this.processListReply.bind(this));

		// TODO: Trigger this on a button
		this.props.socket.emit('msg', JSON.stringify({ type: 'list' }));
	}

	public componentWillUnmount() {
		this.props.socket.removeListener('list_reply');
	}

	public shouldComponentUpdate(nextProps: PreFlightProps, nextState: PreFlightState): boolean {
		return (
			nextProps.initialized !== this.props.initialized ||
			nextState.files.length !== this.state.files.length ||
			nextState.filename !== this.state.filename ||
			nextState.breakpoint !== this.state.breakpoint ||
			nextState.scale !== this.state.scale
		);
	}

	/**
	 * TODO: Whatever behavior is supposed to be triggered by this button.
	 */
	private handleNewJocsFile(event: React.MouseEvent<HTMLButtonElement>): void {
		alert('JOCS Upload has not been implemented yet.');
	}

	/**
	 * Process an request to load a JOCS file. If any of the form paramaters are invalid, alert the user.
	 */
	private handleSubmit(event: React.MouseEvent<HTMLButtonElement>): void {
		// TODO: More sophisticated form validation
		const conditions: Array<[boolean, string]> = [
			[this.state.breakpoint < 0, 'Invalid breakpoint selected'],
			[!this.state.files.some(file => file.name === this.state.filename), 'Invalid File Selected'],
			[this.state.scale <= 0 || this.state.scale > 1, 'Invalid Scale entered']
		];

		const errors = conditions.filter(([cond,]) => cond);

		if (errors.length) {
			errors.forEach(([, msg]) => alert(msg));
		} else {
			const request = {
				jocsPath: 'data/' + this.state.filename,
				startPoint: this.state.breakpoint,
				theaterScale: this.state.scale,
				type: 'load'
			};
			this.props.socket.emit('msg', JSON.stringify(request));
		}
		event.preventDefault();
	}

	/**
	 * Process the file response sent from the backend.
	 * If the filename hasn't been initialized yet, set it to the first file in the list (or N/A).
	 */
	private processListReply(message: any): void {
		const json = JSON.parse(message);
		if (json && json.files) {
			const files: Array<File> = json.files;
			const filename = this.state.filename || (files.length > 0 ? files[0].name : 'N/A');
			this.setState({ filename: filename, files: json.files });
		} else {
			alert('File list from server is invalid.');
		}
	}

	public render(): JSX.Element {
		const containerClass = 'col-xs-4 text-center';
		const selectedFile = this.state.files.find(file => file.name === this.state.filename);
		const breakpoints = (selectedFile && selectedFile.breakpoints) || [];
		const filenames = this.state.files.map(file => file.name);

		return (
			<div className='text-center gap row'>
				<div className='col-sm-12 text-center'>
					<div className='container-fluid noMargin'>
						<div className='col-xs-2 col-md-1'>
							<button className='btn btn-fab white' onClick={this.handleNewJocsFile}>
								<span className="fa fa-upload" aria-hidden="true"></span>
							</button>
						</div>
						<FileSelector filenames={filenames} handleChange={ (value: string) => this.setState({ filename: value }) } />
						<div className='col-md-2'>&nbsp;</div>
						<ScaleSelector handleChange={ (value: number) => this.setState({ scale: value }) } />
						<BreakpointSelector breakpoints={breakpoints} handleChange={ (value: number) => this.setState({ breakpoint: value }) } />
						<div className='col-xs-2 col-md-1'>
							<button className='btn btn-fab white' disabled={this.props.initialized} onClick={this.handleSubmit}>
								<span className='fa fa-thumbs-up' aria-hidden='true'></span>
							</button>
						</div>
					</div>
				</div>
				<Button cb={ () => alert('Mapping currently not enabled') } containerClass={containerClass} disabled={false} iconClass='fa fa-th' />
				<Button cb={ () => this.props.socket.emit('msg', JSON.stringify({ type: 'prepare' })) }
						containerClass={containerClass}
						disabled={!this.props.initialized}
						iconClass='fa fa-rocket' />
				<KillButton class={containerClass} socket={this.props.socket} />
			</div>
		);
	}
}
