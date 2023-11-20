// Copied from <user>/python_lib
// MISC JS functions / stuff that can be used by any page

// DO NOT USE, CONTAINS INFINITE LOOP
/*class MultiLoader {
	jobs;
	callback;

	constructor(callback, args) {
		this.jobs = args.length;
		this.callback = callback;
        //this.jobDone.bind(this);// https://stackoverflow.com/questions/4011793/this-is-undefined-in-javascript-class-methods
		for(let i = 0; i < args.length; i++) {
			alert(`break (MultiLoader.constructor()), beginning of loop`)
			//args[i][4] = function(status, response){this.jobDone();args[i][4](status, response);};// Very bad, fix later
            args[i][4] = (status, response) => {// https://chat.openai.com/share/a60b6eeb-112a-4f63-90b3-a7773b6149ce
                this.jobDone();
                args[i][4](status, response);
            };
			asyncRequest(...args[i]);
		}
	}

	jobDone() {
		alert(`break (MultiLoader.jobDone()), jobs: ${this.jobs}`)
		this.jobs -= 1;
		if(this.jobs == 0) {
			alert("break (MultiLoader.jobDone()), about to call this.callback()")
			this.callback();
		}
	}
}*/

function asyncRequest(action, dir, sendString, contentType, doneCallback, statusElement, userMessages)
{
	if(typeof(userMessages) === 'undefined') userMessages = ["Updated", "Failed"];
	asyncRequestRetryCallback = function(){asyncRequest(action, dir, sendString, contentType, doneCallback, statusElement, userMessages)};// TODO: don't use global variable
	var xhr = new XMLHttpRequest();
	xhr.open(action, dir);
	xhr.setRequestHeader("Content-Type", contentType);
	xhr.onreadystatechange = function(){
		if(xhr.readyState === 4){
			doneCallback(xhr.status, xhr.responseText);
			if(xhr.status == 200)
			{
				statusElement.innerHTML = userMessages[0];
				statusElement.style.color = "#089500";
			}
			else
			{
				statusElement.innerHTML = userMessages[1] + ", HTTP code=" + xhr.status + "<button onClick='asyncRequestRetryCallback()'>Retry</button>";
				statusElement.style.color = "#FF0000";
			}
		}};
	// waiting
	statusElement.innerHTML = "Waiting...";
	statusElement.style.color = "#000000";
	// send request
	xhr.send(sendString);
}

function E(id)// Element
{
	return document.getElementById(id);
}

function centerBody()// puts the <body> tag in the center of the screen
{
	var screenWidth = window.screen.width;
	var body = document.querySelector("body");
	var bodyWidth = body.offsetWidth;// no pun intended
	body.style += "margin-left: " + Math.max((screenWidth - bodyWidth) / 2, 5) + "px;";
}

function sizeBody()// sets the max size of the body to fit the screen
{
	var screenWidth = window.screen.width;
	document.querySelector("body").style.maxWidth = screenWidth - 10;
}

function parseDateString(dateStr)// https://stackoverflow.com/questions/5619202
{
	var parts = dateStr.split('-');
	// Please pay attention to the month (parts[1]); JavaScript counts months from 0:
	// January - 0, February - 1, etc.
	var date = new Date(parts[0], parts[1] - 1, parts[2]);
	return date.getTime() / 1000;
}

function parseTimeString(timeStr)
{
	var arr = timeStr.split(":");
	var validLengths = [1, 2, 3];
	var multipliers = {1: [1], 2: [60, 1], 3: [3600, 60, 1]};
	if(validLengths.includes(arr.length))
	{
		var result = 0;
		for(i = 0; i < arr.length; i ++)
		{
			result += multipliers[arr.length][i] * arr[i];
		}
		return result;
	}
	else
	{
		return null;
	}
}

function formatTimeString(secs)// opposite of parseTimeString(), copied from https://stackoverflow.com/questions/1322732 (w/ some modification), answer by Harish Ambady
{
	var secsPerDay = 3600 * 24;
	var date = new Date(null);
	date.setSeconds(secs); // specify value for SECONDS here
	var days = Math.floor(secs / secsPerDay);
	var secs = secs % secsPerDay;
	// days
	if(days > 0)
	{
		var currStr = days.toString() + "d ";
	}
	else
	{
		var currStr = "";
	}
	// < 1 day
	var decimalStr = "";
	if(secs != Math.round(secs))
	{
		decimalStr = "." + (secs + "").split(".")[1];
	}
	if(secs < 60)// < 1 minute
	{
		return currStr + date.toISOString().substr(17, 2) + decimalStr;
	}
	if(secs < 3600)// < 1 hour
	{
		return currStr + date.toISOString().substr(14, 5) + decimalStr;
	}
	return currStr + date.toISOString().substr(11, 8) + decimalStr;// >= 1 hour
}

// from https://dirask.com/posts/JavaScript-convert-RGB-color-to-CSS-HEX-color-pqeZgp
const nToStringHex = (component) => {
    if (component < 0 || component > 255) {
        throw new Error('Incorrect color component value.');
    }
    const text = component.toString(16);
    if (component < 16) {
        return '0' + text;
    }
    return text;
};

const RGBToHex = (rgb) => '#' + nToStringHex(rgb[0]) + nToStringHex(rgb[1]) + nToStringHex(rgb[2]);

function formatStringMatrix(matrix, sep)// matrix: array (top down) of rows (left to right)
{
	// default seperator: " - "
	if(typeof(sep) === 'undefined') sep = " - ";
	// get column widths
	var columnWidths = [];
	for(x = 0; x < matrix[0].length; x++)
	{
		var currLengths = [];
		for(y = 0; y < matrix.length; y++)
		{
			currLengths.push(matrix[y][x].toString().length);
		}
		columnWidths.push(Math.max(...currLengths));
	}
	// create string
	var returnStr = "";
	for(y = 0; y < matrix.length; y++)
	{
		var x = 0;
		matrix[y].map(
			function(str)
			{
				if(x > 0) returnStr += sep;
				returnStr += str.toString().padStart(columnWidths[x], " ")
				x++;
			}
		)
		returnStr += "\n";// end of line
	}
	return returnStr;// Done
}

function fillTableHTML(header, data)
{
	var returnStr = "";
	if(header.length > 0)
	{
		returnStr += "<tr>" + fillTableRowHTML("th", header) + "</tr>";
	}
	for(var i = 0; i < data.length; i++)
	{
		returnStr += "<tr>" + fillTableRowHTML("td", data[i]) + "</tr>";
	}
	return returnStr;
}

function fillTableRowHTML(tag, rowData)
{
	var returnStr = "";
	for(var i = 0; i < rowData.length; i++)
	{
		returnStr += `<${tag}>${rowData[i]}</${tag}>`;
	}
	return returnStr;
}

function timeStampToDateTime(ts)
{
	var date = new Date(ts * 1000);
	return date.getFullYear() + "-" + (date.getMonth() + 1) +
    	"-" + date.getDate() + " " + date.getHours().toString().padStart(2, '0') +
    	":" + date.getMinutes().toString().padStart(2, '0') + ":" + date.getSeconds().toString().padStart(2, '0');
}

function isUsableNumber(n)
{
	var n = Number(n);
	return typeof(n) === 'number' && !isNaN(n);
}
