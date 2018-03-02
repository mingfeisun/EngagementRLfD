var Kinect2 = require('kinect2');
var request = require('request');
var http = require('http');
var zlib = require('zlib');
var jpeg = require('jpeg-js');

var currFrame = null;
var currFace = null;

var kinect = new Kinect2();

var params = {};
var url = "https://westus.api.cognitive.microsoft.com/emotion/v1.0/recognize?"+params;

var compression = 1;
var origWidth = 1920;
var origHeight = 1080;
var compressedWidth = origWidth / compression;
var compressedHeight = origHeight / compression;
var resizedLength = 4 * compressedWidth * compressedHeight;

var resizedBuffer = new Buffer(resizedLength);
var compressing = false;

var headers = [
    {
        'Content-Type': 'application/octet-stream',
        'Ocp-Apim-Subscription-Key': 'dc5d2900fca941a2b0380af78b231521',
    },
    {
        'Content-Type': 'application/octet-stream',
        'Ocp-Apim-Subscription-Key': '497902a70e3c4dc9ade3dd964ebbaed5',
    },
    {
        'Content-Type': 'application/octet-stream',
        'Ocp-Apim-Subscription-Key': 'da7bb7645b634e24bf5cde607b2cb652',
    },
    {
        'Content-Type': 'application/octet-stream',
        'Ocp-Apim-Subscription-Key': 'a8c09735cc8b49bb88c5ded3f11d1274',
    },
    {
        'Content-Type': 'application/octet-stream',
        'Ocp-Apim-Subscription-Key': 'faf5f407ee3a49e383c731b6072a6acd',
    },
    {
        'Content-Type': 'application/octet-stream',
        'Ocp-Apim-Subscription-Key': '3a08cdb4724e47b78e3d6feb574f649e',
    },
    {
        'Content-Type': 'application/octet-stream',
        'Ocp-Apim-Subscription-Key': '664df8bdc1cb4b2098b6766023c2fa5f',
    },
    {
        'Content-Type': 'application/octet-stream',
        'Ocp-Apim-Subscription-Key': 'ec3dac63a6904b409136e3c5405a42cb',
    } ];
var index = 0;

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

    kinect.on('colorFrame', function(data){
        if(!compressing) {
            compressing = true;
            var y2 = 0;
            for(var y = 0; y < origHeight; y+=compression) {
                y2++;
                var x2 = 0;
                for(var x = 0; x < origWidth; x+=compression) {
                    var i = 4 * (y * origWidth + x);
                    var j = 4 * (y2 * compressedWidth + x2);
                    resizedBuffer[j] = data[i];
                    resizedBuffer[j+1] = data[i+1];
                    resizedBuffer[j+2] = data[i+2];
                    resizedBuffer[j+3] = data[i+3];
                    x2++;
                }
            }
            compressing = false;
        }
    });

    kinect.openColorReader();
    kinect.openBodyReader();
}

var host = '0.0.0.0';
var port = 1337;
http.createServer(function (req, res) {
    console.log("Receiving request...");
    res.write(JSON.stringify({"body":currFrame, "face":currFace}));
    res.end();
    console.log("Sent done");
}).listen(port, host);
console.log("Waiting...");


setInterval(function () {
    var rawData = {
        data: resizedBuffer,
        width: compressedWidth,
        height: compressedHeight
    };

    var jpegData = jpeg.encode(rawData, 50 );

    if(index >=6){
        index = 0;
    }

    var options = {
        url: url,
        method: 'POST',
        body: jpegData['data'],
        headers:headers[index]
    };
    request(options, function (err, res, body) {
        if(!err && res.statusCode == 200 && body.length != 0){
            currFace = body;
        }
        index = index + 1;
    });
}, 700);
