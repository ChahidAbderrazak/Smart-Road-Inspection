var http = require('http');
var fs = require('fs');
// var index = fs.readFileSync('index.html');
var home = fs.readFileSync('home.html');
var selectmission = fs.readFileSync('selectmission.html');
var createmission = fs.readFileSync('createmission.html');
var startmission = fs.readFileSync('startmission.html');
var viewhistory = fs.readFileSync('viewhistory.html');
var css = fs.readFileSync('style.css');

// Global Variables
var hallValue = 0;

const admin = require("firebase-admin");
const serviceAccount = require("./recipeappdjango-firebase-adminsdk-63i16-bb9afe57c7.json");
admin.initializeApp({
	credential: admin.credential.cert(serviceAccount),
	databaseURL: "https://recipeappdjango-default-rtdb.firebaseio.com"
});
// admin.database.enableLogging(true);
var db = admin.database();

var app = http.createServer(function(req, res){
    switch (req.url) {
        case ("/style.css") :
            res.writeHead(200, {"Content-Type": "text/css"});
            res.write(css);
            break;
        case ("/select-mission") :
            res.writeHead(200, {"Content-Type": "text/html"});
            res.write(selectmission);
            break;
        case ("/create-mission") :
            res.writeHead(200, {"Content-Type": "text/html"});
            res.write(createmission);
            break;
        case ("/start-mission") :
            res.writeHead(200, {"Content-Type": "text/html"});
            res.write(startmission);
            break;
        case ("/view-history") :
            res.writeHead(200, {"Content-Type": "text/html"});
            res.write(viewhistory);
            break;
        // case ("/view-history") :
        //     res.writeHead(200, {"Content-Type": "text/html"});
        //     res.write(viewhistory);
        //     break;
        
        default :    
            res.writeHead(200, {"Content-Type": "text/html"});
            res.write(home);
	}
    res.end();
});

var io = require('socket.io')(app);

io.on('connection', (socket) => {
    console.log('Nodejs is listening');
	var ref = db.ref("drones/drone_01"); 

	ref.on("value", function(snap) {
		var ppm = snap.val()['RC']['ppm'];

		var signalstrength = snap.val()['angles']['signalstrength'];
		var missionstatus = snap.val()['currentmission']['000']['missionstatus'];

		var status = snap.val()['status']['battery'];
		var pitch = snap.val()['angles']['pitch'];
		var roll = snap.val()['angles']['roll'];
		var yaw = snap.val()['angles']['yaw'];
		var alt = snap.val()['position']['y'];
		var positionx = snap.val()['position']['x'];
		var positiony = alt;
		var positionz = snap.val()['position']['z'];

		var context = {
			ppm: ppm,
			// dronestatus: dronestatus,
			signalstrength: signalstrength,
			missionstatus: missionstatus,

			status: status, 
			pitch: pitch,
			roll: roll,
			yaw: yaw,
			altitude: alt,
			x: positionx,
			y: positiony,
			z: positionz
		};

		// console.log("Hall-value FB: ", hallValue);
		socket.emit('data', { context: context });
	},  function (errorObject) {
		console.log("The read failed: " + errorObject.code);
	});

});

app.listen(3000);
