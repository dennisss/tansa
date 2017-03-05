'use strict';

/*
	All commands and data with the browser as sent via this socket

	General flow of events

	- Browser calls 'open' to start a new session (or resume an existing one)

	- Browser calls 'save' with { name: 'myfirstplan', plan: {...}  } to save a plan by a given name

	- Browser calls 'run' with { name: 'myfirstplan' } to run an already saved plan
		- Server responds with a { id: 'randomSimulationId' } // This will initialize a control session and start playing it

	- Server starts a container simulation instance
		- Server incrementally sends back
			'update': {session_id: 'same as acked before', type: 'update' time: simtimeInSeconds, state: { iris: {position: [...], orientation: [...]}  }}


		- Eventually server sends back a message with 'done': {session_id: '...'}


*/


module.exports = function(server){

	var io = require('socket.io')(server /* , {
		path: '/api/socket',
		serveClient: false
	} */);

	io.on('connection', function(socket) {
		console.log('Client Connected!');

		// Just broadcast all received messages to all other clients
		socket.on('msg', function(data) {
			socket.broadcast.emit('msg', data);
		});

		socket.on('status', function(data) {
			socket.broadcast.emit('status', data);
		});

		socket.on('list_reply', function(data) {
			socket.broadcast.emit('list_reply', data);
		});

		socket.on('load_reply', function(data) {
			socket.broadcast.emit('load_reply', data);
		});

	});


}
