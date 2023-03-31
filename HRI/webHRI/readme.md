# Web Based Human Robot Interface (webHRI)
This pkg implementes a web-based interface for human-robot interaction. 
Its main component is a web page (defults chrome) that integrates a MQTT client, listening to a parametrized topic (see web/ttsMQTT.js), and a speechSynthesis API callback to reproduce the text. Multiple parameters of the voice can be modified (see web/ttsMQTT.js).

For further documentation refer to: https://developer.mozilla.org/en-US/docs/Web/API/SpeechSynthesis


## Hints
The ros2 node in this pkg is used for the simple task to automatically open and load the web-interface (see webHRI.launch). As the web integrates its own MQTT client (paho javascript client), there is no "direct" connection with ros, but through MQTT msgs. The topic for this communication can be easily configured in (web/ttsMQTT.js).

See the pkg "management/task_manager" which offers a "say" srv call, wrapping the MQTT communication.