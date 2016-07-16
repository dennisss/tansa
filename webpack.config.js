var webpack = require('webpack'),
	autoprefixer = require('autoprefixer'),
	ExtractTextPlugin = require("extract-text-webpack-plugin"),
	path = require('path'),
	ProgressBarPlugin = require('progress-bar-webpack-plugin');


module.exports = {

	context: __dirname,

	entry: {
		web: __dirname + "/src/web/index.js",
	},

	resolve: {
		root: path.resolve('./src'),
	    extensions: ['', '.js']
	},

	devtool: 'source-map',

	cache: true,

	module: {
		loaders: [
			{ test: /\.css$/, loader: ExtractTextPlugin.extract("style-loader", "css-loader!postcss") },
			{ test: /\.less$/, loader: ExtractTextPlugin.extract("style-loader", "css-loader!less-loader!postcss") },
			{
				test: /\.jsx?$/,
				exclude: /(node_modules|bower_components)/,
				include:  (__dirname + '/src'),
				loader: 'babel', // 'babel-loader' is also a legal name to reference
				query: {
					presets: ['stage-1', 'es2015', 'react']
				}
			},
			{
				test: /\.(eot|svg|ttf|woff|woff2)$/,
				loader: 'file?name=fonts/[name].[ext]'
			}
		]
	},
	output: {
		path: __dirname + '/public/assets/',
		filename: 'web.js'
	},

	postcss: [ autoprefixer({ browsers: ['last 2 versions'] }) ],
	plugins: [
		new ExtractTextPlugin("web.css"),
		new ProgressBarPlugin()
	],

	stats: {
		children: false
	}


};
