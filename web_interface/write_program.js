<!-- Begin

function main() {

  Object.size = function(obj) {
    var size = 0, key;
    for (key in obj) {
        if (obj.hasOwnProperty(key)) size++;
    }
    return size;
  };

  var url_array = window.location.href.split('/');
  var url = 'http://' + url_array[2] + "/web_abominations/login.html";

  var connection = new ros.Connection("ws://" + url_array[2] + ":9091");
  var connectInfo = document.location.toString();
  var current_program = { info: {}, code: ""};
  var my_programs = new Array();
  var PYTHON = 1;
  var PUPPET = 2;


  current_program['info']['type'] = PYTHON;
  var tokenMatches = connectInfo.match(/tkn=([^&]*)/);
  if (tokenMatches === null) {
    self.location.href = url;
    return;
  }
  var token = tokenMatches[1];

  var usernameMatches = connectInfo.match(/uname=([^&]*)/);
  if (usernameMatches === null) {
    self.location.href = url;
    return;
  }
  var username = usernameMatches[1];

  var adminMatches = connectInfo.match(/.*&ad=([^&]*)/);
  if (adminMatches === null) {
    self.location.href = url;
    return;
  }
  var is_admin = adminMatches[1];

  document.getElementById("queue").href = 'http://' + url_array[2] + "/web_abominations/queue.html?tkn=" + token + "&uname=" + username + "&ad=" + is_admin;
  document.getElementById("home").href = 'http://' + url_array[2] + "/web_abominations/web_abomination.html?tkn=" + token + "&uname=" + username + "&ad=" + is_admin;
  document.getElementById("write_program").href = 'http://' + url_array[2] + "/web_abominations/write_program.html?tkn=" + token + "&uname=" + username + "&ad=" + is_admin;
  document.getElementById("my_programs").href = 'http://' + url_array[2] + "/web_abominations/my_programs.html?tkn=" + token + "&uname=" + username + "&ad=" + is_admin;  

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

    connection.callService('/get_my_programs', '[' + token + ']' ,function(resp) {
        my_programs = resp.programs;
        fillList();
    });

    function fillList() {
      var dropdown = document.getElementById("program_list");
      dropdown.options.length = 0;
      var empty = new Option("","");
      dropdown.options.add(empty);
      for (var i = 0; i < my_programs.length; i++) {
        var item = new Option(my_programs[i]["name"]);
        dropdown.options.add(item);
      }
    }

    var logout_button = document.getElementById("logout_button");
    logout_button.onclick = function() {
      connection.callService('/logout', '[' + token + ']' ,function(resp) {
        self.location.href = url;
      });
    };
    
    var save_button = document.getElementById("save_button");
    save_button.onclick = function() {
      var program_name = document.getElementById("program_name").value;
      var code = document.getElementById("code").value;
      if (Object.size(current_program) === 0 || program_name !== current_program["info"]["name"]) {
        connection.callService('/create_program', '[' + token + ']' ,function(resp) {
          current_program["info"]["id"] = resp.id;
          current_program['info']['name'] = program_name;
          current_program['info']['type'] = PYTHON;
          current_program['info']['owner'] = username;
          current_program['code'] = code;

          connection.callService('/update_program', '[' + token + ',' + JSON.stringify(current_program) + ']', function(resp) {
            connection.callService('/get_my_programs', '[' + token + ']' ,function(resp) {
              my_programs = resp.programs;
              fillList();
            });
          });

        });

      } else {
        current_program['info']['name'] = program_name;
        current_program['info']['type'] = PYTHON;
        current_program['info']['owner'] = username;
        current_program['code'] = code;

        connection.callService('/update_program', '[' + token + ',' + JSON.stringify(current_program) + ']', function(resp) { 
        });

      }
    
    };

    var queue_button = document.getElementById("add_to_queue");
    queue_button.onclick = function() {
      if (current_program['code'] === document.getElementById("code").value) {
        connection.callService('/queue_program', '[' + token + ',' + current_program['info']['id'] + ']' ,function(resp) {
        });
      } else {
        var program_name = document.getElementById("program_name").value;
        var code = document.getElementById("code").value;
        if (Object.size(current_program) === 0 || program_name !== current_program["info"]["name"]) {
          connection.callService('/create_program', '[' + token + ']' ,function(resp) {          
            current_program["info"]["id"] = resp.id;
            current_program['info']['name'] = program_name;
            current_program['info']['owner'] = username;
            current_program['code'] = code;

            connection.callService('/update_program', '[' + token + ',' + JSON.stringify(current_program) + ']', function(resp) {
              connection.callService('/get_my_programs', '[' + token + ']' ,function(resp) {
                my_programs = resp.programs;
                fillList();
              });             
              connection.callService('/queue_program', '[' + token + ',' + current_program['info']['id'] + ']' ,function(resp) {
              });
             });
          });
        } else {
          current_program['info']['name'] = program_name;
          current_program['info']['owner'] = username;
          current_program['code'] = code;
          connection.callService('/update_program', '[' + token + ',' + JSON.stringify(current_program) + ']', function(resp) {
            connection.callService('/queue_program', '[' + token + ',' + current_program['info']['id'] + ']' ,function(resp) {
            });
          });
        }
      }
    };

    var edit_existing = document.getElementById("program_list");
    edit_existing.onchange = function() {
      var program_name = document.getElementById("program_name");
      var dropdown = document.getElementById("program_list");
      program_name.value = dropdown.options[dropdown.selectedIndex].value;
      var id = my_programs[dropdown.selectedIndex -1].id;
      connection.callService('/get_program', '[' + id + ']', function(resp) {
        current_program = resp.program;
        var code_box = document.getElementById("code");
        code_box.value = current_program['code'];   
      });
    };
  
    var program_type = document.getElementById("program_type");
    program_type.onchange = function() {
      var type = document.getElementById("program_type");
      current_program['info']['type'] = type.selectedIndex + 1;
    };
  });
}
// End -->
