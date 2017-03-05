export interface Breakpoint {
	name: string,
	number: number,
	start: number
}

export interface File {
	breakpoints: Array<Breakpoint>,
	name: string
}

export interface GlobalStatus {
	initialized: boolean,
	landed: boolean
	paused: boolean,
	playing: boolean,
	ready: boolean,
	time: number
}

interface Point {
	x: number,
	y: number,
	z: number
}

export interface Vehicle {
	armed: boolean,
	battery: {
		percent: number,
		voltage: number
	},
	connected: boolean,
	id: number,
	position: [number, number, number],
	role: number,
	tracking: boolean
}
