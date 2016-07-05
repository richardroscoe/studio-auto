// JavaScript Document

// This is our MQTT Broker
host = "192.168.1.127";
port = 9001;

// Create a client instance
client = new Paho.MQTT.Client(host, port, "web_" + parseInt(Math.random() * 100, 10));

// set callback handlers
client.onConnectionLost = onConnectionLost;
client.onMessageArrived = onMessageArrived;

// Attempt reconnection every 6seconds
function retryConnect() {
	setInterval(tryConnect(), 6000);
}

// connect the client
function tryConnect() {
	document.getElementById("connectionStatus").innerHTML = "Trying to connect";
	client.connect({onSuccess:onConnect, onFailure:retryConnect});
}
  
tryConnect();


// called when the client connects
function onConnect(context) {
  // Once a connection has been made, make a subscription and send a message.
  console.log("onConnect");
  
  // Make our topic subscriptions
  client.subscribe("studio/#");
  client.subscribe("store/#");
  
  // Update the displayed connection status
  $("#connectionStatus").html("Connected to: " + host + ':' + port);
}

// called when the client loses its connection
function onConnectionLost(responseObject) {
  if (responseObject.errorCode !== 0) {
    console.log("onConnectionLost:"+responseObject.errorMessage);
  }
  // Update the displayed connection status
  $("#connectionStatus").html("Not connected");
  retryConnect();
}

// called when a message arrives from the MQTT broker
// See which topic arrived and do the appropriate action
//
function onMessageArrived(message) {
  console.log('Message Recieved: Topic: "', message.destinationName, '". Payload: ', message.payloadString, '. QoS: ', message.qos);
  
  switch(message.destinationName.trim()) {
	case "studio/sensor/1/temperature":
		$("#studioTemp").html(message.payloadString + " &deg;C");
		break;
	case 'studio/sensor/1/humidity':
		$("#studioHumidity").html(message.payloadString + " %");
	  	break;
	case 'studio/sensor/1/changeRate':
		$("#studioRoC").html(message.payloadString + " &deg;C/min");
	  	break;
	case "store/sensor/1/temperature":
		$("#storeTemp").html(message.payloadString + " &deg;C");
		break;
	case 'store/sensor/1/humidity':
		$("#storeHumidity").html(message.payloadString + " %");
	  	break;
	case 'store/sensor/1/changeRate':
		$("#storeRoC").html(message.payloadString + " &deg;C/min");
	  	break;		
	case 'studio/state/thermostat/control':
		$("#thermoControl").val((parseInt(message.payloadString) == 0) ? "Off" : "On").flipswitch("refresh");
		break;
	case 'studio/state/thermostat/sp/0':
		$("#sp0input").val(message.payloadString).slider("refresh");
		break;
	case 'studio/state/thermostat/sp/1':
		$("#sp1input").val(message.payloadString).slider("refresh");
		break;
	case 'studio/state/light/sw/0':
		$("#switch0").val((parseInt(message.payloadString) == 0) ? "Off" : "On").flipswitch("refresh");
		break;
	case 'studio/state/light/sw/1':
		$("#switch1").val((parseInt(message.payloadString) == 0) ? "Off" : "On").flipswitch("refresh");
		break;
	case 'studio/state/light/sw/2':
		$("#switch2").val((parseInt(message.payloadString) == 0) ? "Off" : "On").flipswitch("refresh");
		break;
	case 'studio/state/light/sw/3':
		$("#switch3").val((parseInt(message.payloadString) == 0) ? "Off" : "On").flipswitch("refresh");
		break;
	case 'studio/state/light/dim/0':
		$("#dim0input").val(message.payloadString).slider("refresh");
		break;
	case 'studio/state/light/dim/1':
		$("#dim1input").val(message.payloadString).slider("refresh");
		break;				
	case 'studio/state/time/0': // Time in the sensors device
		var sensorDate = new Date(parseInt(message.payloadString) * 1000);
		$("#studioSensorTime").html(sensorDate.toLocaleTimeString());
		break;
	case 'studio/state/time/1': // Time in the lighting device
		var sensorDate = new Date(parseInt(message.payloadString) * 1000);
		$("#studioLightTime").html(sensorDate.toLocaleTimeString());
		break;
	case 'store/state/time/0': // Time in the storeroom device
		var sensorDate = new Date(parseInt(message.payloadString) * 1000);
		$("#storeTime").html(sensorDate.toLocaleTimeString());
		break;	
	default:
		console.log("No match");
		break;
  }
}

// Just before JQuery Mobile goes to play with our page, changing it
// a little, put in the event handlers for our controls
//
$( document ).on( "pagecontainerbeforeshow", function ( event, ui ) { 
	$("#thermoControl").on("change", function (e) {
		thermoControl(this.value == "On" ? 1 : 0);
	});

	$("input#sp0input").on("slidestop", function (e) {
		thermoSp(0);
	});
	
	$("input#sp1input").on("slidestop", function (e) {
		thermoSp(1);
	});
	
	$("#switch0").on("change", function (e) {
		lightSw(0, this.value == "On" ? 1 : 0);
	});
	
	$("#switch1").on("change", function (e) {
		lightSw(1, this.value == "On" ? 1 : 0);
	});

	$("#switch2").on("change", function (e) {
		lightSw(2, this.value == "On" ? 1 : 0);
	});
		
	$("#switch3").on("change", function (e) {
		lightSw(3, this.value == "On" ? 1 : 0);
	});	
	
	$("input#dim0input").on("slidestop", function (e) {
		dimmer(0);
	});
	
	$("input#dim1input").on("slidestop", function (e) {
		dimmer(1);
	});		
});


function thermoControl(OnOff) {
	// Send a message
	message = new Paho.MQTT.Message(String(OnOff));
	message.destinationName = "studio/thermostat/control";
	message.retained = true;
	client.send(message);
	return 0;
}
function thermoHeater(heaterNum, OnOff) {
	message = new Paho.MQTT.Message(String(OnOff));
	message.destinationName = "studio/heater/" + heaterNum;
	message.retained = true;
	client.send(message);
	return 0;
}

function thermoSp(spNum) {
	// Get the value from the input field
	var sp = document.getElementById("sp" + spNum + "input").value;
	console.log("Field value :" + sp);

	message = new Paho.MQTT.Message(String(sp));
	message.destinationName = "studio/thermostat/sp/" + spNum;
	message.retained = true;
	client.send(message);
	return 0;
}

// dimmer - tells the MQTT broker that we want a dimmer to change level
function dimmer(dimNum) {
	// Get the value from the input field
	var v = document.getElementById("dim" + dimNum + "input").value;
	console.log("Dimmer Field value :" + v);

	message = new Paho.MQTT.Message(String(v));
	message.destinationName = "studio/light/dim/" + dimNum;
	message.retained = true;
	client.send(message);
	return 0;
}

// lightSw - tells the MQTT broker that we want a light switches state to change
function lightSw(switchNum, OnOff) {
	// Send a message
	message = new Paho.MQTT.Message(String(OnOff));
	message.destinationName = "studio/light/sw/" + switchNum;
	message.retained = true;
	client.send(message);
	return 0;
}