var Kinect2 = require('kinect2');
var request = require('request');
var http = require('http');
var zlib = require('zlib');
var jpeg = require('jpeg-js');

var currFrame = null;

var kinect = new Kinect2();

if(kinect.open()) {
    console.log("Kinect Opened");
    kinect.on('bodyFrame', function(bodyFrame){
        currFrame = bodyFrame;
        for(var i=0; i < bodyFrame.bodies.length; i++){
            var body = bodyFrame.bodies[i];
            if (body.tracked){
                currFrame = body;
                break;
            }
        }
    });

    kinect.openBodyReader();
}

var host = '0.0.0.0';
var port = 1337;
http.createServer(function (req, res) {
    console.log("Receiving request...");
    res.write(JSON.stringify({"body":currFrame}));
    res.end();
    console.log("Sent done");
}).listen(port, host);
console.log("Waiting...");
