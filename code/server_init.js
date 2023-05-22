const Engine = require('tingodb')();
const db = new Engine.Db('/Team1-Boujaoude-Deokar-Hijazi-Hardy/quest-6/code/mydb', {});

db.collection('mycollection', function(err, collection) {
  if (err) throw err;
  console.log("Collection created!");
});