// MAPIR HRI

// =============================== CONFIGURATION PARAMETERS ============================================

	// MQTT params
	var wsbroker = "150.214.109.137";
	var wsport = 8000;
	var prefix = 'rhodon';
	var topic = prefix + '/TTS';
	var topicSTT = prefix + '/STT';
	var robotID = "ttsListener";

	// SpeechSynthesis params
	var voice_names = ["Google español", "Google US English", "Google UK English Female", "Google UK English Male"];
	var my_voice = voice_names[0];

// ======================================================================================================
	
// Create a new utterance for the specified text
function speak_msg(text) {
	console.log("[speak] request to speechSynthesis API with msg: " + text);
	// Create a new instance of SpeechSynthesisUtterance.
	var msg = new SpeechSynthesisUtterance();

	// Set the text.
	msg.text = text;

	// Set the attributes.
	msg.volume = 1;		    // range [0,1]
	msg.rate = 1;			// range [0.1,10]
	msg.pitch = 1; 			// range [0,2]
	msg.voice = speechSynthesis.getVoices().filter(function (voice) {return voice.name == my_voice;})[0];
    
	// Events
	msg.onstart = (event) => {
		console.log(`[onstart] We have started uttering this speech: ${event.utterance.text}`);
		
		// Add subtitles
		writeInner(msg.text);

		// Update image
		changeImage("Robot/RobotGif1.gif");
	};

	msg.onend = (event) => {
		console.log("[onend] utterance has finished being spoken after ${event.elapsedTime} seconds.");
		// Remove subtitles
		writeInner("");

		// Update image
		changeImage("Robot/RobotGif0.gif");
	  };

	// Queue this utterance.
	window.speechSynthesis.speak(msg);

	
}


//Funcion que se encarga de cambiar el texto a mostrar en el html
function writeInner(str){
	var displayMsg = document.getElementById('text');
	displayMsg.innerHTML = str;
}

//Funcion que se encarga la imagen a mostrar en el html
function changeImage(image){
	document.getElementById("img").src=image;
}

//Crea una instancia de un cliente. Paho.MQTT.Client(host, port, path, clientId) 
var client = new Paho.MQTT.Client(wsbroker, wsport,"/mqtt",
        "myclientid_" + parseInt(Math.random() * 100, 10));
				
//Manejador llamado cuando hay perdida de conexi�n		
client.onConnectionLost = function (responseObject) {
		console.log("[MQTT] Connection lost: " + responseObject.errorMessage);
		changeImage("Robot/RobotDesconectado.png"); //�ltimo a�adido para mostrar cara al desconectarse
		writeInner("[ERROR] Connection lost: " + responseObject.errorMessage + "<br> Reconnecting...");
		setTimeout(function() {init();},5000);
	};
 
//Manejador de la llegada de un mensaje por MQTT
client.onMessageArrived = function (message) {
	console.log("[onMessageArrived] from: "+ message.destinationName, ' content: ', message.payloadString);
	speak_msg(message.payloadString)	
};
 

function init() {	
	// Check for speechSynthesis browser support
	var supportMsg = document.getElementById('text');
	if ('speechSynthesis' in window) {
		supportMsg.innerHTML = 'Great! Your browser <strong>supports</strong> speech synthesis.';
	} else {
		supportMsg.innerHTML = 'Sorry, your browser <strong>does not support</strong> speech synthesis.';
		supportMsg.classList.add('not-supported');
	}

	//Configure MQTT connection
	var options = {
      timeout: 3,
      onSuccess: function () {			
        console.log("[init] MQTT connected to broker: "+ wsbroker);
        //Conexi�n hecha, se subscribe a un t�pico
        client.subscribe(topic, {qos: 1});
        console.log("[init] Subscribed to topic: "+topic);
		writeInner("MQTT connected and subscribed to topic: "+topic);
		changeImage("Robot/RobotGif0.gif");
      },
      onFailure: function (message) {
        console.log("[init] Connection failed: " + message.errorMessage); 
		changeImage("Robot/RobotDesconectado.png"); //�ltimo a�adido para mostrar cara al desconectarse
		writeInner("Error connecting to MQTT broker: " + message.errorMessage + "<br> Reconnecting...");
		setTimeout(function() {init();},5000);
      }
    };
 
	console.log("[init] Initializing MQTT client..."); 
	writeInner("Inicializando MQTT");
    client.connect(options); //Conecta el cliente al servidor
}
 
 // sends msg over MQTT
 function sendMessage(topic,message_cmd) {
	mqttmessage = new Paho.MQTT.Message(message_cmd);
	mqttmessage.destinationName = topic;
	client.send(mqttmessage);
	console.log("SENDMESSAGE_Sent:" + message_cmd + " to: " + topic);
}

// call the initialization!
init();