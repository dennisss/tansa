#!/usr/bin/env node

'use strict';

var fs = require('fs'),
	path = require('path');

var args = process.argv.slice(2);

var fname = args[0];

var data;

try {
	data = require(path.resolve(process.cwd(), fname));

	if(data == null) {
		data = { error: "Don't forget to module.export the data" };
	}
}
catch(e) {
	data = { error: e.stack || e };
}

fs.writeFileSync(fname + ".o", JSON.stringify(data));
