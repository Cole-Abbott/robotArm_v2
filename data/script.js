var websocket = null;
var localhost = "";
var b = document.getElementById('btnWS');
var buttonClicked = false;

// to keep track of whether the websocket is open or closed
var webSocketState = false;

//global state variable
var state;

// Initialize the websocket
function init() {
	if (window.location.hostname != "") {
		localhost = window.location.hostname;
	}

	doConnect();
}

function doConnect() { // makes a connection and defines callbacks
	if (webSocketState == false) {
		webSocketState = true;
		localhost = window.location.hostname;
		writeToScreen("Connecting to ws://" + localhost + "/ws ...");
		websocket = new WebSocket("ws://" + localhost + "/ws");
		websocket.onopen = function (evt) {
			onOpen(evt)
		};
		websocket.onclose = function (evt) {
			onClose(evt)
		};
		websocket.onmessage = function (evt) {
			onMessage(evt)
		};
		websocket.onerror = function (evt) {
			onError(evt)
		};
	} else {
		writeToScreen("Disconnecting ...");
		websocket.close();
		webSocketState = false;
	}
}

function onOpen(evt) { // when handshake is complete:
	writeToScreen("Connected.");
}

function onClose(evt) { // when socket is closed:
	webSocketState = false;
	writeToScreen("Disconnected. Error: " + evt);
}

function onMessage(msg) {
	//turn the message into a json object
	const message = JSON.parse(msg.data);
	console.log(message);

	//update the buttons
	// updateButtons();
}

function onError(evt) { // when an error occurs
	websocket.close();
	webSocketState = false;
	writeToScreen("Websocket error");

}


// add event listeners for all the buttons in a div
function addEventListeners() {
	var div = document.getElementById("Modes");
	var buttons = div.getElementsByTagName('button');
	for (var i = 0; i < buttons.length; i++) {
		buttons[i].addEventListener('click', function () {
			if (webSocketState == false) {
				doConnect();
			}
			message = this.title;
			websocket.send(message);
		});
	}

	// event listener for the brightness slider
	div.getElementsByClassName('speedSlider')[0].addEventListener('input', function () {
		if (webSocketState == false) {
			doConnect();
		}
		var speed = this.value;

		var message = `speed: ${speed}`;

		websocket.send(message);
		// console.log(message);
	});


}


// Function to update the buttons based on the state
function updateButtons() {
	// get the buttons
	const modeDiv = document.getElementById('Modes');
	const modeButtons = modeDiv.getElementsByTagName('button');

	// get the brightness sliders
	const speedSlider = modeDiv.getElementsByClassName('speedSlider')[0];

	// update the buttons
	for (var i = 0; i < modeButtons.length; i++) {
		if (state.ceiling.mode == modeButtons[i].title) {
			modeButtons[i].style.backgroundColor = "#2EE59D";
		} else {
			modeButtons[i].style.backgroundColor = "#f2f2f2";
		}
	}

	
	// update the brightness sliders
	speedSlider.value = state.ceiling.brightness;
}

// add event listeners for all the buttons
addEventListeners();


// Function to display to the message box
function writeToScreen(message) {
	console.log(message);
}

// Open Websocket as soon as page loads
window.addEventListener("load", init, false);


// keep track of the keys that are pressed
var pressedKeys = {};
window.onkeyup = function(e) { pressedKeys[e.keyCode] = false; }
window.onkeydown = function(e) { pressedKeys[e.keyCode] = true;}

pos = {x: 0, y: 30, z: 555.9, type: 0};

function updatePosition() {

	var updatePos = false;

	if (pressedKeys[87]) { // W
		pos.x += 1;
		updatePos = true;
	}
	if (pressedKeys[83]) { // S
		pos.x -= 1;
		updatePos = true;
	}
	if (pressedKeys[65]) { // A
		pos.y -= 1;
		updatePos = true;
	}
	if (pressedKeys[68]) { // D
		pos.y += 1;
		updatePos = true;
	}
	if (pressedKeys[81]) { // Q
		pos.z += 1;
		updatePos = true;
	}
	if (pressedKeys[69]) { // E
		pos.z -= 1;
		updatePos = true;
	}

	if (webSocketState == false) {
		doConnect();
	} else if (updatePos) {
		websocket.send(JSON.stringify(pos));
		console.log(pos);
	}
}

setInterval(updatePosition, 100); // update the position every 100 ms


const jointFunction = () => {
	if (webSocketState == false) {
		doConnect();
	}
	var t1 = parseFloat(document.getElementsByClassName('j1Slider')[0].value);
	var t2 = parseFloat(document.getElementsByClassName('j2Slider')[0].value);
	var t3 = parseFloat(document.getElementsByClassName('j3Slider')[0].value);
	var t4 = parseFloat(document.getElementsByClassName('j4Slider')[0].value);

	updateJointReadouts();

	var jointMessage = {
		t1: t1,
		t2: t2,
		t3: t3,
		t4: t4,
		type: 1
	};

	websocket.send(JSON.stringify(jointMessage));
	console.log(jointMessage);
}

document.getElementsByClassName('j1Slider')[0].addEventListener('input', jointFunction);
document.getElementsByClassName('j2Slider')[0].addEventListener('input', jointFunction);
document.getElementsByClassName('j3Slider')[0].addEventListener('input', jointFunction);
document.getElementsByClassName('j4Slider')[0].addEventListener('input', jointFunction);

document.getElementById('zeroJoints').addEventListener('click', () => {
	if (webSocketState == false) {
		doConnect();
	}

	var zeroMessage = {
		type: 2
	};

	websocket.send(JSON.stringify(zeroMessage));
	console.log("Zeroing joints");
	document.getElementsByClassName('j1Slider')[0].value = 0;
	document.getElementsByClassName('j2Slider')[0].value = 0;
	document.getElementsByClassName('j3Slider')[0].value = 0;
	document.getElementsByClassName('j4Slider')[0].value = 0;
	updateJointReadouts();
});

function updateJointReadouts() {
	const toDeg = (rad) => rad * 180 / Math.PI;

	const t1 = parseFloat(document.getElementsByClassName('j1Slider')[0].value);
	const t2 = parseFloat(document.getElementsByClassName('j2Slider')[0].value);
	const t3 = parseFloat(document.getElementsByClassName('j3Slider')[0].value);
	const t4 = parseFloat(document.getElementsByClassName('j4Slider')[0].value);

	const setText = (id, val) => {
		const el = document.getElementById(id);
		if (!el) return;
		el.textContent = `${val.toFixed(2)} rad (${Math.round(toDeg(val))}°)`;
	};

	setText('j1Value', t1);
	setText('j2Value', t2);
	setText('j3Value', t3);
	setText('j4Value', t4);
}

// initialize readouts on page load
updateJointReadouts();