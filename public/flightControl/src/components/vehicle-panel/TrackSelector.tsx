import * as React from 'react';

export class TrackSelector extends React.Component<undefined, undefined> {
	constructor() {
		super();
	}

	public shouldComponentUpdate():boolean {
		return false;
	}

	public render(): JSX.Element {
		return(
			<div className='col-xs-1 col-md-1 text-center'>
				<div className='form-group noMargin'>
					<select className='form-control'>
						<option>N/A</option>
					</select>
				</div>
			</div>
		);
	}
}
