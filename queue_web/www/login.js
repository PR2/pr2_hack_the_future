/* global veriables. ick, but useful */
var host = "localhost";
var port = 9090;
var connection;
var token;
var is_admin;

/* general functions */
function output(m) {
   var output = document.getElementById("output");
   output.innerHTML = m;
}

function main() {
   token = jaaulde.utils.cookies.get('token');
   is_admin = jaaulde.utils.cookies.get('is_admin');

   //output("Opening RosJs connection");
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
         get_programs();
      });
      if( is_admin ) {
         // if admin, show admin select button
         document.getElementById('admin_button').style.display = "block";
      }
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
            is_admin = msg.msg.is_admin;

            if( token != "0" ) {
               output("");
               /* save token to cookie */
               jaaulde.utils.cookies.set('token', token);
               jaaulde.utils.cookies.set('is_admin', is_admin);
               /* show editor section */ 
               document.getElementById("login").style.display = "none";
               document.getElementById("edit").style.display = "block";

               if( is_admin ) {
                  // if admin, show admin select button
                  document.getElementById('admin_button').style.display = "block";
               }

               get_programs();
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
      jaaulde.utils.cookies.del('is_admin');
      /* show login screen */ 
      document.getElementById("edit").style.display = "none";
      document.getElementById("login").style.display = "block";
   }
}
