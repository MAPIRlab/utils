// MAPIR HRI

/*
* CONFIGURATION PARAMETERS
*/
	// MQTT params
	var wsbroker = "150.214.109.137";
	var wsport = 8000;
	var prefix = 'rhodon';
	var topic = prefix + '/TTS';
	var topicSTT = prefix + '/STT';
	var robotID = "ttsListener";

	// SpeechSynthesis params
	var my_voice = "Google español";	// Google US English, Google UK English Female, Google UK English Male, Google español

	
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
            
	// Queue this utterance.
	window.speechSynthesis.speak(msg);
	var supportMsg = document.getElementById("text");
	supportMsg.innerHTML = msg.text;
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

	/*
	//Comprobamos que el mensaje no tenga m�s de 200 caracteres, pues entonces no funciona el speak de chorme
	if(message.length>200){ //El tama�o no es soportado por chrome tts //Posible 210 tambi�n
		console.log("[onMessageArrived]: El tama�o del mensaje ("+message.length+") es mayor del l�mite soportado por Chrome.tts (sobre 200), vamos a dividirlo en alg�n . : ? � !");
		dividirMensaje(message); //Dividimos el mensaje en dos o tres submensajes si es necesario (hasta 600 caracteres)		
	}else{ //El tama�o es soportado por chrome tts
		//El mensaje recibido se transfiere a la funci�n de habla
		setTimeout(function() {speak(message);},100); //Retardo de tiempo de muestra del mensaje (Para la paralelizaci�n que dice Curro)
	}
	*/	
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
 
 //Funci�n auxiliar para mandar un nuevo mensaje
 function sendMessage(topic,message_cmd) {
	mqttmessage = new Paho.MQTT.Message(message_cmd);
	mqttmessage.destinationName = topic;
	client.send(mqttmessage);
	console.log("SENDMESSAGE_Sent:" + message_cmd + " to: " + topic);
}

//Funci�n que recursivamente va partiendo (siempre que sea posible) una cadena a hablar en cadenas peque�as de 200 caracteres para que sean reproducibles por Chrome.tts
function dividirMensaje(mensaje){

	if(mensaje.length<=200){ //Lo hablo y termina la recursividad
		speak(mensaje);
		//console.log(mensaje);
	}else{	//Lo divido en submensajes, hablo el primero y el segundo lo vuelvo a pasar a la funci�n (recursivamente)
		var punto = (mensaje.substring(0,201)).lastIndexOf(".");
		var puntoComa = (mensaje.substring(0,201)).lastIndexOf(";");
		var inte = (mensaje.substring(0,201)).lastIndexOf("?");
		var excla = (mensaje.substring(0,201)).lastIndexOf("!");
	
		//Vamos a quedarnos con el punto de corte m�s cercano a 200 	
		var puntoParticion = Math.max(punto,puntoComa,inte,excla);
		//console.log("Punto de partici�n: "+puntoParticion);
		
		if(puntoParticion!=-1){ //Si hay partici�n posible, partimos por ah�
			var m1 = mensaje.substring(0,puntoParticion+1);
			var m2 = mensaje.substring(puntoParticion+1,mensaje.length);
			//console.log(m1);
			//console.log(m2);
			
			//La primera partici�n es menor que 200, luego la hablamos. No sabemos si la segunda lo es
			speak(m1); //Hablamos la primera partici�n
			dividirMensaje(m2);
		}else{ //Si no, no hacemos nada
			console.log("DIVIDIRMENSAJE_Error: no hay ning�n punto de partici�n �til en el mensaje que no altere el significado del mismo, use en el mensaje m�s . ; ? � !");
			speak("Error, no se ha podido seguir hablando ya que el mensaje era demasiado largo, por favor vuelva a introducirlo pero ahora usando puntos como pausas");
		}
	}
}


// call the initialization!
init();