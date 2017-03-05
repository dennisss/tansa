

function timeConvert(seconds) {
	if(seconds > 0) {
		seconds = ~~((seconds * 100)/100);
		// Minutes and seconds
		var mins = ~~(seconds / 60);
		var secs = seconds % 60;

		// Output like "1:01" or "4:03:59" or "123:03:59"
		var ret = "";

		ret += "" + (mins < 10 ? "0" : "")
		ret += mins + ":" + (secs < 10 ? "0" : "");
		ret += "" + secs;
	} else {
		ret = " -:-- "
	}
	return ret;
}


module.exports = {
	timeConvert: timeConvert
}
