var ROSLIB = require('roslib');

var ros = new ROSLIB.Ros();

// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
  console.log(error);
});

// Find out exactly when we made a connection.
ros.on('connection', function() {
  console.log('Connection made!');
});

ros.on('close', function() {
  console.log('Connection closed.');
});

ros.connect('ws://192.168.0.26:8080');

var chatterSub = new ROSLIB.Topic({
  ros: ros,
  name: '/chatter',
  messageType: 'std_msgs/String'
});

chatterSub.subscribe((message) => {console.log(message.data)});