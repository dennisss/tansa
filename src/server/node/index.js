var http = require('http'),
	express = require('express');

var app = express();
var server = http.Server(app);


//app.get('/', function (req, res) {
//	res.send('Hello World!');
//});

require('./socket')(server);

app.use('/node_modules', express.static(__dirname + '/../../../node_modules'));

app.use(express.static(__dirname + '/../../../public'));

server.listen(3000, function () {
	console.log('Example app listening on port 3000!');
});
