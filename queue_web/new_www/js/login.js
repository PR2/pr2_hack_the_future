/* global veriables. ick, but useful */
var host = window.location.hostname;
var port = 9091;
var connection;
var token;
var is_admin;

function initialize_tabs() {
  $("#tabs").tabs();
}

function resize_stuff() {
  $("#content").height($(window).height() - $("#header").height() - $("#status-bar").height() - 8);
  $("#editor").height($("#content").height() - $("#editor-control").height());
  editor.resize();
}

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
               $(".login-form").hide();
              $("#editor-name").show();
              $("#content").show();
              initialize_tabs();
              editor = ace.edit("editor");
              editor.setTheme("ace/theme/cobalt");
              editor.getSession().setMode("ace/mode/python");
              $(window).resize(function() {
                resize_stuff();
              });
              resize_stuff();
              output("");

               if( is_admin ) {
                $("#admin-tab").show();
               }

               get_programs();
            } else {
               output("Login Failed");
            }
         });

  return false;
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
      document.getElementById("all_programs").innerHTML = "";
   }
   is_admin = False
}

/* general functions */
function output(m) {
  if (m === "") {
    m = "&nbsp;";
  }
  var _output = document.getElementById("status-bar");
  _output.innerHTML = m;
}

var timeout = 10;

function setup_ros() {
  output("Opening RosJs connection");
  connection = new ros.Connection("ws://" + host + ":" + port);
  connection.setOnClose(function(e) {
    $(".login-form").show();
    $("#editor-name").hide();
    $("#content").hide();
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
      $("#content").hide();
      output("");
      get_programs();
      timeout = 10;
    });
    if( is_admin ) {
      // if admin, show admin select button
      $("#admin-tab").show();
    }
  } else {
    connection.setOnOpen(function(e) {
      $(".login-form").show();
      $("#editor-name").hide();
      $("#content").hide();
      output("");
      timeout = 10;
    });
  }
}

function main() {
  $("input[type=submit], button").button();
  token = jaaulde.utils.cookies.get('token');
  is_admin = jaaulde.utils.cookies.get('is_admin');
  $("#admin-tab").hide();

  setup_ros();
}

$(document).ready(main);
