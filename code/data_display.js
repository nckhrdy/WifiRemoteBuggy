var express = require('express');
var Engine = require('tingodb')();
var db = new Engine.Db('./', {});
var app = express();

app.get('/', function(req, res) {
  db.collection('mycollection', function(err, collection) {
    if (err) throw err;
    collection.find().toArray(function(err, data) {
      if (err) throw err;

      // Format data into a table
      var table = '<table>';
      table += '<thead><tr><th>CheckpointTime</th>';
      table += '<tbody>';

      for (var i = 0; i < data.length; i++) {
        var row = '<tr>';
        if (data[i].CheckpointTime !== undefined) {
          // Handle documents with "CheckpointTime" property
          row += '<td>' + data[i].CheckpointTime + '</td>';
        } else {
          // Handle objects with "k", "o", and "v" properties
          row += '<td>' + data[i].k + '</td>' +
                 '<td>' + data[i].o + '</td>' +
                 '<td>' + data[i].v + '</td>';
        }
        row += '</tr>';
        table += row;
      }
      

      table += '</tbody></table>';

      // Send the table as a response
      var html = '<!DOCTYPE html>' +
                 '<html>' +
                 '<head>' +
                 '<meta charset="utf-8">' +
                 '<title>Rally Splits</title>' +
                 '<style>' +
                 'table { border-collapse: collapse; }' +
                 'th, td { padding: 8px; text-align: left; border-bottom: 1px solid #ddd; }' +
                 'tr:nth-child(even) { background-color: #f2f2f2; }' +
                 '</style>' +
                 '</head>' +
                 '<body>' +
                 table +
                 '</body>' +
                 '</html>';
      res.send(html);
    });
  });
});

app.listen(8081, function() {
  console.log('Server listening on port 8081');
});
