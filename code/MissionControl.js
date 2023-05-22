//U97871602 - Hardy Nicholas
const express = require("express");
const app = express();
const dgram = require("dgram");
const { Db } = require("tingodb")();

const PORT = 8081;
const HOST = "192.168.1.24";

// Create a new UDP server
const server = dgram.createSocket("udp4");

// Create server that listens on a port
server.on("listening", function () {
  var address = server.address();
  console.log(
    "UDP Server listening on " + address.address + ":" + address.port
  );
});

// Define the checkpoint times database
const db = new Db("./", {});
const mycollection = db.collection("mycollection");

// Initialize the last recorded times for each checkpoint to 0

// On connection, update checkpoint times if the new time is different from the last recorded time
server.on("message", function (message, remote) {
  const checkpointTime = parseFloat(message);
  const time = Date.now() / 1000;

  // if (lastTimes[checkpointTime] !== time) {
  //   lastTimes[checkpointTime] = time;

    // Update the checkpoint time in the database
    mycollection.update(
      { CheckpointTime: checkpointTime },
      { $set: { Time: time } },
      { upsert: true },
      (err, result) => {
        if (err) {
          console.error(err);
        } else {
          //console.log(`Checkpoint Time: ${checkpointTime} updated with time ${time}`);
        }
      }
    );
  //}
});



server.bind(PORT, HOST);

// Create the HTML for the control panel
const controlPanelHTML = `
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Car Controller</title>
  <style>
    body {
      background-color: #FFE4E1;
      font-family: Arial, sans-serif;
    }
    img {
      display: block;
      margin: 50px auto;
      box-shadow: 0px 0px 10px rgba(0,0,0,0.5);
    }
    table {
      border-collapse: collapse;
      margin: 20px auto;
      box-shadow: 0px 0px 10px rgba(0,0,0,0.5);
    }
    th, td {
      border: 1px solid black;
      padding: 10px;
    }
  </style>
</head>
<body>
  <h1>EC444 Final Rally Comp</h1>
  <p>Car #01 - Driven by: XXXX</p>
  <img src="http://192.168.1.36:8082" width="640" height="480" alt="Buggy Camera Stream"/>
  <table>
    <caption>Checkpoint Times</caption>
    <thead>
      <tr>
        <th>Checkpoint Time</th>
        <th>Time</th>
      </tr>
    </thead>
    <tbody id="checkpointTime">
    </tbody>
  </table>
  <script>
    // Listen for key press events
    document.addEventListener('keydown', function(event) {
      if (event.key === 'w') {
        // Send an HTTP GET request to the server to forward
        fetch('/send-message?key=w')
          .then(response => {
            console.log('Forward message sent');
          })
          .catch(error => {
            console.error(error);
          });
      } else if (event.key === 's') {
        // Send an HTTP GET request to the server to stop
        fetch('/send-message?key=s')
          .then(response => {
            console.log('Stop message sent');
          })
          .catch(error => {
            console.error(error);
          });
      } else if (event.key === 'a') {
          // Send an HTTP GET request to the server to stop
          fetch('/send-message?key=a')
            .then(response => {
              console.log('Left message sent');
            })
            .catch(error => {
              console.error(error);
          });
      } else if (event.key === 'd') {
          // Send an HTTP GET request to the server to stop
          fetch('/send-message?key=d')
            .then(response => {
              console.log('Right message sent');
            })
            .catch(error => {
              console.error(error);
          });
      }
    });
  </script>
</body>
</html>
`;

// Send the HTML for the control panel
app.get("/", (req, res) => {
  res.send(controlPanelHTML);
});

// Handle the message sending HTTP request
app.get("/send-message", (req, res) => {
  const key = req.query.key;
  if (key === "w") {
    server.send("w", PORT, "192.168.1.20");
    console.log("Forward message sent!");
    res.send("Forward message sent");
  } else if (key === "s") {
    server.send("s", PORT, "192.168.1.20");
    console.log("Stop message sent!");
    res.send("Stop message sent");
  }
  else if (key === "a") {
    server.send("a", PORT, "192.168.1.20");
    console.log("Left message sent!");
    res.send("Left message sent");
  }
  else if (key === "d") {
    server.send("d", PORT, "192.168.1.20");
    console.log("Right message sent!");
    res.send("Right message sent");
  }
  else {
    res.status(400).send("Invalid key");
  }
});

// Listen for requests on port 3000
app.listen(3000, () => {
  console.log("App listening on port 8081");
});