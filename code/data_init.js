var fs = require('fs');
var Engine = require('tingodb')();
var db = new Engine.Db('./mydb', {});

// Read the contents of the file as a string
var contents = fs.readFileSync('RallyTimes.txt', 'utf8');

// Split the contents into an array of lines
var lines = contents.split('\n');

// Extract the headers from the first line
var headers = lines[0].split('\t');

// Create an empty array to hold the data
var data = [];

// Loop through the remaining lines and add the data to the array
for (var i = 1; i < lines.length; i++) {
  var row = {};
  var values = lines[i].split('\t');
  for (var j = 0; j < headers.length; j++) {
    row[headers[j]] = values[j];
  }
  data.push(row);
}

// Insert the data into the database
db.collection('mycollection', function(err, collection) {
  if (err) throw err;
  collection.insert(data, function(err, res) {
    if (err) throw err;
    console.log(res.insertedCount + " documents inserted");
    db.close();
  });
});