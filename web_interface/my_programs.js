<!-- Begin

function main() {
  var url_array = window.location.href.split('/');
  var connection = new ros.Connection("ws://" + url_array[2] + ":9091");
  var connectInfo = document.location.toString();
  var url = 'http://' + url_array[2] + "/web_abominations/login.html";

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

        /*var connection = null;
        try {
            connection = new ros.Connection('ws://' + address + ':' + port);
        } catch (err) {
            log('Problem creating proxy connection object!');
            return;
        }*/

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
    var logout_button = document.getElementById("logout_button");
    logout_button.onclick = function() {
      connection.callService('/logout', '[' + token + ']' ,function(resp) {
        self.location.href = url;
      });
    };

    connection.callService('/get_my_programs', '[' + token + ']' ,function(resp) {
      var my_programs = resp.programs;
      for (var i = 0; i<resp.programs.length;i++) {
          var programDiv = document.getElementById("program_div");
          var element = document.createElement("div");
          element.id = "div_" + resp.programs[i].id.toString();
          element.style="position:relative; text-align:right;";
          var text = document.createElement("label");
          text.innerHTML = resp.programs[i].name
          text.id = "text_"+ resp.programs[i].id.toString();
          text.for = resp.programs[i].id.toString();
          element.appendChild(text);
          if (is_admin === "true" || username === resp.programs[i].owner) {
            var output_button = document.createElement("button");
            output_button.innerHTML = "Output";
            output_button.id = resp.programs[i].id.toString();
            output_button.onclick = function() {
              connection.callService('/get_output', '[' + token + ',' + this.id + ',' + '1' +']' ,function(resp) {
              //alert(resp.output[resp.output.length -1].output);
              var left_corner = (screen.width - 600)/2;
              var top_corner = (screen.height - 600)/2;
              var w = window.open('', '', 'width=600,height=600,resizeable,scrollbars,top=' + top_corner.toString() + ',left=' + left_corner.toString() );
              w.document.body.innerHTML = '<textarea id="code" name="code" rows="20" cols="150">' + resp.output[resp.output.length -1].output + '</textarea>';
              w.document.close(); // needed for chrome and safari
            });
            }
            element.appendChild(output_button);

            var queue_button = document.createElement("button");
            queue_button.innerHTML = "Add to Queue";
            queue_button.id = resp.programs[i].id.toString();
            queue_button.onclick = function() {
              connection.callService('/queue_program', '[' + token + ',' + this.id + ']' ,function(resp) {
              });
            }
            element.appendChild(queue_button);
          }
          var show_code = document.createElement("button");
          show_code.innerHTML = "Show Code";
          show_code.id = resp.programs[i].id.toString();
          show_code.onclick = function() {
            connection.callService('/get_program', '[' + this.id + ']', function(resp) {
              var left_corner = (screen.width - 600)/2;
              var top_corner = (screen.height - 600)/2;
              var w = window.open('', '', 'width=600,height=600,resizeable,scrollbars,top=' + top_corner.toString() + ',left=' + left_corner.toString() );
              w.document.body.innerHTML = '<textarea id="code" name="code" rows="20" cols="150">' + resp.program.code + '</textarea>';
              w.document.close(); // needed for chrome and safari
            });
          }
          element.appendChild(show_code);
          programDiv.appendChild(element);
      }
    });

  });


}
// End -->
