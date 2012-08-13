/* global veriables. ick, but useful */
var host = "localhost";
var port = 9090;
var connection;
var token;

/* general functions */
function output(m) {
   var output = document.getElementById("output");
   output.innerHTML = m;
}

function main() {
   token = jaaulde.utils.cookies.get('token');

   output("Opening RosJs connection");
   connection = new ros.Connection("ws://" + host + ":" + port);
   connection.setOnClose(function(e) {
      output("RosJs Connection Closed: " + e);
   });

   connection.setOnError(function(e) {
      output("RosJs Error: " + e);
   });

   if( token ) {
      connection.setOnOpen(function(e) {
         document.getElementById("edit").style.display = "block";
         output("");
      });
   } else {
      connection.setOnOpen(function(e) {
         document.getElementById("login").style.display = "block";
         output("");
      });
   }

   start_editor();
}

/* session management functions */
function login(f) {
   var username = f.username.value;
   var password = f.password.value;

   connection.callServiceRaw("/login", "[\"" + username + "\", \"" + 
         password + "\"]", function(resp) {
            str = resp.replace(/ (\d)/g, ' "\$1').replace(/(\d),/g, '\$1",');
            var msg = JSON.parse(str);
            token = msg.msg.token;

            if( token != "0" ) {
               output("");
               /* save token to cookie */
               jaaulde.utils.cookies.set('token', token);
               /* show editor section */ 
               document.getElementById("login").style.display = "none";
               document.getElementById("edit").style.display = "block";
            } else {
               output("Login Failed");
            }
         });
}

function createAccount(f) {
}

function logout(f) {
   if( token ) {
      connection.callService('/logout', '[' + token + ']');
      jaaulde.utils.cookies.del('token');
      /* show login screen */ 
      document.getElementById("edit").style.display = "none";
      document.getElementById("login").style.display = "block";
   }
}
