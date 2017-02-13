#!/usr/bin/env node

'use strict';

var fs = require('fs'),
	path = require('path');

var args = process.argv.slice(2);

var fname = args[0];

var data = require(path.resolve(process.cwd(), fname));

fs.writeFileSync(fname + ".o", JSON.stringify(data));
