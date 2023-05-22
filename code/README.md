# Code Readme

To run the Rally Buggy code, download all files and flash the ESP using idf.py flash monitor (or -p PORT) from inside the main folder. 

The MissionControl.js file is the Node server which hosts the web page with the live stream. Run this in the code folder using node MissionControl.js

# MissionControl.js:  
This file is responsible for controlling the car through a web interface. The code uses the Express framework to create a web server that serves a control panel HTML page to the client. The page includes an image stream from the car's camera, a table displaying the checkpoint times, and key press event listeners that send HTTP GET requests to the server to control the car.

The server uses UDP to receive checkpoint time updates from the car, which are then stored in a database. The database is created using the TingoDB library and is defined as "mycollection." The code initializes the last recorded times for each checkpoint to 0.

The server listens for incoming UDP messages from the car and updates the checkpoint times in the database if the new time is different from the last recorded time. The checkpoint time and the current time are stored in the database using the "update" method of the TingoDB library. The code includes error handling for the database update process.

The code defines an HTML string for the control panel, which is served to the client when they request the root URL. The code also defines an endpoint for handling HTTP GET requests to control the car. The endpoint listens for key press events in the client's browser and sends UDP messages to the car based on the key pressed. The code includes error handling for invalid keys.

Finally, the server listens for incoming requests on port 3000 and logs a message when it is ready to receive connections. When running, the server can receive key presses from the client, send UDP messages to the car, and receive UDP messages from the car to update the checkpoint times in the database.

# qr_reader.py: 
The "qr_code_scanner.py" Python script is designed to run on a Raspberry Pi with a camera module. The script uses the PiCamera library to capture video frames from the camera and the zbarlight library to scan for QR codes in the captured frames. When a QR code is detected, the script sends the decoded data over UDP to a specified IP address and port number.

The script starts by initializing the camera and setting the resolution and framerate. It then creates a stream to capture video frames and starts the camera preview.

The script creates a UDP socket for sending QR code data and binds it to a specific IP address and port number. It then enters a loop to continuously capture and process video frames from the camera.

Within the loop, the script captures an image from the camera and stores it in the stream. It then converts the image to grayscale and scans for QR codes in the image using the zbarlight library.

If a QR code is detected, the script decodes the data and encodes it as bytes. It then sends the data over UDP to the specified IP address and port number using the socket created earlier.

If no QR code is detected, the script prints a message indicating that no QR code was found.

After processing each frame, the script resets the stream and sleeps for a short duration to control the frame rate.

Finally, the script stops the camera preview, closes the camera and socket objects, and exits the script.

# data_display.js: 

This is a JavaScript code snippet for creating a web application using the Express.js framework to display checkpoint times in a table format.

It starts by importing the Express.js library and the Tingodb engine for the database. Then, it creates an instance of the Express application and defines a route for handling requests to the root URL ('/').

In the route handler, it connects to the 'mycollection' collection in the database and retrieves all the documents using the 'find' method. It then formats the data into an HTML table and sends it as a response to the client.

The resulting HTML page displays the checkpoint times in a table with headers for 'CheckpointTime'. The table is styled using CSS to define the border, padding, and background color of each cell.

The server listens on port 8081, and a message is printed to the console to indicate that the server is running.
