var http = require('http'),
	express = require('express'),
	request = require('request');

var app = express();
var server = http.Server(app);


//app.get('/', function (req, res) {
//	res.send('Hello World!');
//});

require('./socket')(server);

app.use('/node_modules', express.static(__dirname + '/../../../node_modules'));

app.use(express.static(__dirname + '/../../../public'));

app.use('/assets', function(req, res) {
	var url = 'http://127.0.0.1:4200/assets'+ req.url;
	req.pipe(request(url)).pipe(res);
});

server.listen(4000, function () {
	console.log('Communications server started on port 4000');
});
