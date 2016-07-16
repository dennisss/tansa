var express = require('express');
var app = express();

//app.get('/', function (req, res) {
//	res.send('Hello World!');
//});

app.use(express.static(__dirname + '/../../public'));

app.listen(4000, function () {
	console.log('Example app listening on port 3000!');
});
