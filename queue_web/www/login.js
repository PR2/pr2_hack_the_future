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
            //output("Got login token: " + introspect("resp", resp, "", 4));
            var raw = JSON.parse(resp);
            //str = resp.replace(/ (\d)/g, ' "\$1').replace(/(\d),/g, '\$1",');
            //var msg = JSON.parse(str);
            output("resp: " + resp + "<br/>raw: " + introspect(raw));
               //"<br/>JSON: " + str + " <br/>Msg: " + introspect(msg));

            /* show editor section */ /*
            document.getElementById("login").style.display = "none";
            document.getElementById("edit").style.display = "block";
            */
         });
}

function createAccount(f) {
}

function introspect(name, obj, indent, levels) {
   indent = indent || "";
   if (typeof(levels) !== "number") levels = 1;
   var objType = typeof(obj);
   var result = [indent, name, " ", objType, " :"].join('');
   if (objType === "object") {
      if (levels > 0) {
         indent = [indent, "  "].join('');
         for (prop in obj) {
            var prop = this.introspect(prop, obj[prop], indent, levels - 1);
            result = [result, "\n", prop].join('');
         }
         return result;
      }
      else {
         return [result, " ..."].join('');
      }
   }
   else if (objType === "null") {
      return [result, " null"].join('');
   }
   return [result, " ", obj].join('');
}
