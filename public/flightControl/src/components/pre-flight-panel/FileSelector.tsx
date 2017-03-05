import * as React from 'react';

interface FileSelectProps {
	filenames: Array<string>,
	handleChange(newValue: string): void
}

interface FileSelectState {
	value: string
}

export class FileSelector extends React.Component<FileSelectProps, FileSelectState> {
	constructor(props: FileSelectProps) {
		super(props);
		this.state = { value: 'Select a JOCS file' };
		this.fileSelected = this.fileSelected.bind(this);
	}

	private fileSelected(event: React.ChangeEvent<HTMLSelectElement>): void {
		this.setState({ value: event.target.value });
		this.props.handleChange(event.target.value);
	}

	public render(): JSX.Element {
		const options = this.props.filenames.map(filename => <option value={filename} key={filename}>{filename}</option>);

		return (
			<div className='col-xs-8 col-md-4'>
				<select className='form-control' value={this.state.value} onChange={this.fileSelected}>
					{options}
				</select>
			</div>
		);
	}
}
