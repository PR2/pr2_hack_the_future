/* global veriables. ick, but useful */
var host = "prp1.willowgarage.com";
var port = 9091;
var connection;
var token;
var is_admin;

/* general functions */
function output(m) {
   var output = document.getElementById("output");
   output.innerHTML = m;
}

var timeout = 10;

function setup_ros() {
   output("Opening RosJs connection");
   connection = new ros.Connection("ws://" + host + ":" + port);
   connection.setOnClose(function(e) {
      document.getElementById("edit").style.display = "none";
      document.getElementById("login").style.display = "none";
      timeout = timeout * 2;
      if( timeout > 5000 ) timeout = 5000;
      output("Connection Closed, reconnecting in " + timeout);
      console.log("Connection Closed, reconnecting in " + timeout);
      setTimeout(setup_ros, timeout);
   });

   connection.setOnError(function(e) {
      output("RosJs Error: " + e);
      console.log(e);
   });

   if( token ) {
      connection.setOnOpen(function(e) {
         document.getElementById("edit").style.display = "block";
         output("");
         get_programs();
         timeout = 10;
      });
      if( is_admin ) {
         // if admin, show admin select button
         document.getElementById('admin_button').style.display = "block";
      }
   } else {
      connection.setOnOpen(function(e) {
         document.getElementById("login").style.display = "block";
         output("");
         timeout = 10;
      });
   }
}

function main() {
   token = jaaulde.utils.cookies.get('token');
   is_admin = jaaulde.utils.cookies.get('is_admin');

   setup_ros();

   start_editor();
}

/* session management functions */
function login(f) {
   var username = f.username.value;
   var password = f.password.value;

   connection.callServiceRaw("/login", "[\"" + username + "\", \"" + 
         password + "\"]", function(resp) {
            var str = resp.replace(/ (\d)/g, ' "\$1').replace(/(\d),/g, '\$1",');
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
   var username = f.username.value;
   var password = f.password.value;
   var password2 = f.password2.value;

   if( username.length === 0 ) {
      output("Username should not be blank");
   } else if( password != password2 ) {
      output("Passwords do not match");
   } else if( password.length === 0 ) {
      output("Password may not be blank");
   } else {
      console.log("Creating account for " + username);
      connection.callServiceRaw('/create_user', 
         JSON.stringify([username, password]),
         function(resp) {
            console.log("create_account response: " + resp);
            var str = resp.replace(/ (\d)/g,' "\$1').replace(/(\d)([,}])/g,'\$1"\$2');
            console.log("create_account str: " + str);
            var msg = JSON.parse(str);

            token = msg.msg.token;
            is_admin = 0;

            if( token != "0" ) {
               output("");
               /* save token to cookie */
               jaaulde.utils.cookies.set('token', token);
               jaaulde.utils.cookies.set('is_admin', is_admin);
               /* show editor section */ 
               document.getElementById("login").style.display = "none";
               document.getElementById("edit").style.display = "block";

               get_programs();
            } else {
               output("That name is already in use; please pick another");
            }
         }
      );
   }
}

function logout(f) {
   if( token ) {
      connection.callService('/logout', '[' + token + ']');
      jaaulde.utils.cookies.del('token');
      jaaulde.utils.cookies.del('is_admin');
      jaaulde.utils.cookies.del('program_id');
      editor.setValue("");
      /* show login screen */ 
      document.getElementById("edit").style.display = "none";
      document.getElementById("login").style.display = "block";

      document.getElementById("login_form").password.value = "";
      document.getElementById("login_form").username.value = "";
   }
}
