<!-- Begin

function login(form) {
  var url_array = window.location.href.split('/');
  var connection = new ConnectionWithOptionalParse("ws://" + url_array[2] + ":9091");
  connection.setOnClose(function (e) {
    alert('connection closed<br/>');
    //start up rosbridge, reload?
  });
  connection.setOnError(function (e) {
    alert('error!<br/>');
  });
  connection.setOnOpen(function (e) {

    connection.dontParse('/login');
    connection.addHandler('/login',function(msg) {
    });
    if (form.username.value != null && form.password.value != null) {
       connection.callService('/login','[\"' + form.username.value + "\", \"" + form.password.value + '\"]',function(resp) {
        var token = resp.replace(/[^0123456789]/g, '');
        if (resp !== null && token !== '0') {
          //go to the web abomination but also tell it stuff
          //Could put the token in the url, but not admin thing, super secret HTTP fun?
          //alert(typeof resp.token);
          var jsonResp = JSON.parse(resp);
          var is_admin = jsonResp['msg']['is_admin'];
          var url_array = window.location.href.split('/');
          var url = 'http://' + url_array[2] + "/web_abominations/web_abomination.html?tkn=" + token + "&uname=" + form.username.value + "&ad=" + is_admin;
          self.location.href = url;
        } else {
          alert("Bad try at logging in.");
        }
      });
    } else {
      alert("Don't leave stuff blank.");
      //form.username.focus();
    }   
  //only make the objects visible if rosbridge connected?
  });
 
}

function newUser(form) {
  var url_array = window.location.href.split('/');
  var connection = new ros.Connection("ws://" + url_array[2] + ":9091");
  connection.setOnClose(function (e) {
    alert('connection closed<br/>');
    //start up rosbridge, reload?
  });
  connection.setOnError(function (e) {
    alert('error!<br/>');
  });
  connection.setOnOpen(function (e) {
    if (form.username.value != null && form.password.value != null) {
      connection.callService('/create_user','[\"' + form.username.value + "\", \"" + form.password.value + '\"]',function(resp) {
        if (resp !== null && resp.token !== 0) {
          login(form);
        } else {
          alert("Bad try at logging in.");
        }
      });
    } else {
      alert("Don't leave stuff blank.");
      //form.username.focus();
    }
  });  
}


function main() {
  var url_array = window.location.href.split('/');
  var connection = new ros.Connection("ws://" + url_array[2] + ":9091");
  connection.setOnClose(function (e) {
    alert('connection closed<br/>');
    //start up rosbridge, reload?
  });
  connection.setOnError(function (e) {
    alert('error!<br/>');
  });
  connection.setOnOpen(function (e) {
    //only make the objects visible if rosbridge connected?
  });
}

// End -->
