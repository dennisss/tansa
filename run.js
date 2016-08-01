#!/usr/bin/env node

var program = require('commander'),
	child_process = require('child_process'),
	path = require('path'),
	chokidar = require('chokidar'),
	webpack = require('webpack'),
	WebpackDevServer = require('webpack-dev-server')
	util = require('util'),
	_ = require('underscore');

function run(){

	var server = null;

	function start(){

		server = child_process.spawn('/usr/bin/env', ['node', 'src/server'], {
			cwd: __dirname,
			stdio: ['ignore', 1, 2]
		});

		// Restart server on exit
		server.on('exit', function(code, signal){
			console.log('Server exited.');
			start();
		});
	}

	start();

	buildServer();


	var watcher = chokidar.watch(path.resolve(__dirname, '../src/server/**/*.js'), {
		//ignored: /[\/\\]\./,
		persistent: true
	});

	watcher.on('change', (path, stats) => {
		console.log('CHANGE')
		if (stats) console.log(`File ${path} changed size to ${stats.size}`);
		if(server){
			console.log('Stopping server')
			server.kill('SIGINT')
			return;
		}

	});


	process.on('exit', function(){
		if(server != null){
			console.log('Killing server')
			server.kill('SIGINT');
		}
	})


}


function buildServer(){
	var config = require("./webpack.config.js");
	var compiler = webpack(config);
	var server = new WebpackDevServer(compiler, {
		// webpack-dev-middleware settings
		publicPath: '/assets',
		stats: {
			chunks: false,
			children: false,
			colors: true
		}

	});
	server.listen(4200);
}



run();
