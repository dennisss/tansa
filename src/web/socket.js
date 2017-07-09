var io = require('socket.io-client'),
	EventEmitter = require('events').EventEmitter;

var socket = io();

socket.on('msg', function(data) {
	data = JSON.parse(data);

	if(data.type == 'msg') {
		console.log('Bad message')
		console.log(data);
		return;
	}

	socket.msgs.emit(data.type, data);

})

socket.msgs = new EventEmitter();


module.exports = socket;
