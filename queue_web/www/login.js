var host = "localhost";
var port = 9090;
var connection;

var token;

function output(m) {
   var output = document.getElementById("output");
   output.innerHTML = m;
}

function main() {
   output("Opening RosJs connection");
   connection = new ros.Connection("ws://" + host + ":" + port);
   connection.setOnClose(function(e) {
      output("RosJs Connection Closed: " + e);
   });

   connection.setOnError(function(e) {
      output("RosJs Error: " + e);
   });

   connection.setOnOpen(function(e) {
      document.getElementById("login").style.display = "block";
      output("");
   });
}

function login(f) {
   var username = f.username.value;
   var password = f.password.value;

   connection.callServiceRaw("/login", "[\"" + username + "\", \"" + 
         password + "\"]", function(resp) {
            str = resp.replace(/ (\d)/g, ' "\$1').replace(/(\d),/g, '\$1",');
            var msg = JSON.parse(str);
            token = msg.msg.token;

            if( token != "0" ) {
               /* show editor section */ 
               output("");
               document.getElementById("login").style.display = "none";
               document.getElementById("edit").style.display = "block";
            } else {
               output("Login Failed");
            }
         });
}

function createAccount(f) {
}
