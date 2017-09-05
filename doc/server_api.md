Server JSON API
===============

- Used by web/javascript interface to view/control the server

- The socket will emit 'msg' events for sending data


The stats message is sent every 1/10th of a second with the current state of everything:

```
{
	type: 'status',
	time: 3.25, // in seconds
	activeFile: '...',
	vehicles: [
		{
			role: 0, // The number in order of the choroegraphy 0-5
			id: 7, // The network id written on the drone, used to distinguish physical drones
			tracking: true,
			connected: true,
			armed: true,
			// takeoff if getting up to the target start position
			// holding if ready to play or is paused and holding in one location
			// flying will coincide with global.playing
			state: 'init|takeoff|holding|flying|landing',

			battery: {
				voltage: 7.4,
				percent: 0.95 // From 0-1
			},

			log: ["", ""] // A list of strings (lines) to add to the log since last time
			position: [x, y, z]
			orientation: [w, x, y, z]
			...

		}

	],
	global: {
		playing: false,
		mocap_connected: true
		...					
	}

}
```


Reply messages
--------------

All messages sent to the server will be replied to using a "\*\*\_reply" message. It tooks the form:

```
{
	type: "something_reply",
	error: "oops!", // Present if something failed
	// Some extra data probably
}
```

List
----

To list all files available to use:
```
{
	type: "list"
}
```

Reply is:
```
{
	type: "list_reply",
	files: ["singleDrone.jocs", ....]
}
```

Load
----

Load a JOCS file at some cue using this: (if no cue is specified, then it will look at the whole file)
```
{
	type: "load",
	jocsPath: "....", // Something from list
	startPoint: 1643,
	theaterScale: 1
}
```

Reply looks like
```
{
	type: "load_reply",
	cues: [....] // A list of all cues in the file
	paths: [ [ [x,y,z],[x,y,z], .. ], ... ], // List of a list of points specifying line segment approximations of the entire motion paths of the drones
	target_positions: [ [x,y], ... ] // A list of 2d positions at which the drones should be at
}
```


Prepare/Takeoff
---------------
```
{
	type: 'prepare'
}
```


Play
----
```
{
	type: 'play'
}
```
