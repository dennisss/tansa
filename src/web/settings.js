var EventEmitter = require('events').EventEmitter,
	_ = require('underscore');

// Class for managing the model
class Settings extends EventEmitter {

	constructor() {
		super();

		var data;
		try {
			data = JSON.parse(localStorage.settings);
		}
		catch(e) {
			data = {};
		}

		this._changes = {};
		this._timeout = null;
		this._data = data;
	}

	setup(options, data) {
		for(var k in data) {
			if(!this._data.hasOwnProperty(k)) {
				this._data[k] = data[k];
			}
		}

		return this._data;
	}


	_parsePath(p) {

		p = (p || '').split(/[\.\[\]]/);
		for(var i = 0; i < p.length; i++) {
			if(p[i].length == 0) {
				p.splice(i, 1);
				i--;
				continue;
			}

			if(p[i]*1 === p[i])
				p[i] *= 1;
		}

		return p;
	}

	get(path) {
		path = this._parsePath(path);
		var obj = this._data;
		for(var i = 0; i < path.length; i++)
			obj = obj[path[i]];

		return obj;
	}

	set(path, value) {
		path = this._parsePath(path);
		var obj = this._data;
		for(var i = 0; i < path.length - 1; i++)
			obj = obj[path[i]];

		obj[path[path.length - 1]] = value;


		// Set that it was changed
		obj = this._changes;
		for(var i = 0; i < path.length - 1; i++) {
			if(!obj.hasOwnProperty(path[i])) {
				obj[path[i]] = {};
			}
			obj = obj[path[i]];
		}

		if(!this._timeout) {
			this._timeout = setTimeout(() => {
				this._timeout = null;

				try {
					this.emit('change', this);
				} catch(e) { console.error(e); }
				this._changes = {};

				localStorage.settings = JSON.stringify(this._data);
			}, 30)
		}
	}

	changed(path) {
		var path = this._parsePath(path);
		var obj = this._changes;
		for(var i = 0; i < path.length; i++) {
			if(!obj.hasOwnProperty(path[i]))
				return false;
			obj = obj[path[i]];
		}

		return true;
	}




	// last file


};


module.exports = new Settings();
