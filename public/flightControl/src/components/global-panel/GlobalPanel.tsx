import * as React from 'react';
import { GlobalStatus } from '../../messaging/dtos';

interface GlobalStatusPanelProps {
	status: GlobalStatus
}

export class GlobalStatusPanel extends React.Component<GlobalStatusPanelProps, undefined> {
	constructor(props: GlobalStatusPanelProps) {
		super(props);
		this.timeConvert = this.timeConvert.bind(this);
	}

	private timeConvert(seconds: number): string {
		if (seconds > 0) {
			const realSeconds = ~~((seconds * 100)/100);
			// Minutes and seconds
			const mins = ~~(realSeconds / 60);
			const secs = realSeconds % 60;
			const digit = secs < 10 ? '0' : '';

			// Output like '1:01' or '4:03:59' or '123:03:59'
			return digit + mins + ':' + digit + secs;
		}

		return ' -:-- ';
	}

	public render(): JSX.Element {
		const statusClass = (b: boolean) => 'fa fa-circle ' + (b ? 'green' : 'red');
		const colClass = 'col-xs-3 text-center';

		const cols = [
			['MoCap:', 'fa fa-binoculars green'],
			['Ready:', statusClass(this.props.status.ready)],
			['Playing:', statusClass(this.props.status.playing)],
			['Run Time:' + this.timeConvert(this.props.status.time), '']
		].map(([text, icon]) =>
			<div className={colClass} key={text}>
				{text}  <i className={icon} aria-hidden='true' />
			</div>
		);

		return (
			<div className='row'>
			   {cols}
			</div>
		);
	}
}
