var editor;
var programs;
var selected = 0;
var editor_mode = 'edit';

function nop() {}

// update the list of programs from the programs array
function update_list(selection) {
   var form = document.getElementById("program_info");
   form.program.innerHTML = "";
   for( var i = 0; i < programs.length; i++ ) {
      var opt = document.createElement('option');
      opt.text = "(" + programs[i].id + "): " + 
         programs[i].name;
      opt.value = opt.text;
      form.program.add(opt, null);
   }
   form.program.selectedIndex = selection;
}

// load the program specified on the form
function load(f) {
   selected = f.program.selectedIndex;
   var id = programs[selected].id;
   connection.callService('/get_program', JSON.stringify([id]),
      function(resp) {
         editor.setValue(resp.program.code);
         editor.navigateFileStart();
         f.program_name.value = resp.program.info.name;
         programs[selected] = resp.program.info;
         update_list(selected);
         jaaulde.utils.cookies.set('program_id', resp.program.info.id);
         update_outputs(nop);
      });
}

// save the program specified on the form
function save(f) {
   var program = {};
   program.info = programs[selected];
   program.info.name = f.program_name.value;
   program.code = editor.getValue();

   connection.callService('/update_program', 
         "[" + token + ", " + JSON.stringify(program) + "]",
         nop);

   update_list(selected);
}

// create a new program
function newprogram(f) {
   var sample = document.getElementById("sample").innerHTML;
   editor.setValue(sample);
   editor.navigateFileStart();
   f.program_name.value = "New Program";

   connection.callService('/create_program', '[' + token + ']',
      function(resp) {
         var program = {};
         program.code = sample;
         program.info = {};
         program.info.id = resp.id;
         program.info.type = 1;
         program.info.name = "New Program";
         selected = programs.length;
         programs.push(program.info);

         connection.callService('/update_program', 
            '[' + token + ', ' + JSON.stringify(program) + ']',
            nop); // may cause errors on console

         jaaulde.utils.cookies.set('program_id', resp.id);
         update_list(selected);
      });
}

// start the editor
function start_editor() {
   editor = ace.edit("editor");
   var PythonMode = require("ace/mode/python").Mode;
   editor.getSession().setMode(new PythonMode());
}

// populate the list of programs from the server
function get_programs() {
   // get program list; display in selection box
   connection.callService('/get_my_programs', '[' + token + ']',
      function(resp) {
         programs = [];
         // load last program ID from cookie
         var id = jaaulde.utils.cookies.get('program_id');
         for (var i = 0 ; i < resp.programs.length ; i++ ) {
            // only show python programs
            if( resp.programs[i].type === 1 ) {
               if( resp.programs[i].id === id ) {
                  selected = programs.length;
               }
               programs.push(resp.programs[i]);
            }
         }

         // populate pulldown list
         update_list(selected);

         // load the selected program
         load(document.getElementById("program_info"));
      });
}

// add current program to queue
function queue_program() {
   var id = programs[selected].id;
   connection.callService('/queue_program', '[' + token + ',' + id + ']',
         function(n) {
            if( is_admin ) {
               update_admin(nop);
            }
         });
}

// mode selection
function ui_mode(f) {
   var m = 0;
   for( var i=0; i<f.mode.length; i++ ) {
      if(f.mode[i].checked) {
         m = f.mode[i].value;
      }
   }
   if( m ) {
      editor_mode = m;
   }

   if( editor_mode === 'output' ) {
      update_outputs(function() {
            document.getElementById('program_output').style.display = 'block';
         });
   } else {
      document.getElementById('program_output').style.display = 'none';
   }

   document.getElementById('editor').style.display = (m === 'edit')?
      'block':'none';

   if( m === 'admin' ) {
      update_admin(function() {
            document.getElementById('admin_panel').style.display = 'block';
         });
   } else {
      document.getElementById('admin_panel').style.display = 'none';
   }
}

// update the outputs div
function update_outputs(f) {
   if( editor_mode === 'output' ) {
      var id = programs[selected].id;
      connection.callService('/get_output', '[' + token + ', ' + id + ', 0]',
         function(o) {
            var program_output = document.getElementById('program_output');
            program_output.innerHTML = '';
            for( var i=0; i<o.output.length; i++ ) {
               var date = new Date(o.output[i].header.stamp.secs*1000 + 
                  o.output[i].header.stamp.nsecs / 1000);
               program_output.innerHTML += '<h3>' + 
               date + '</h3>';
               program_output.innerHTML += '<pre>' + 
               o.output[i].output + '</pre>';
            }
            f();
         });
   }
}

// update the admin div
function update_admin(f) {
   if( editor_mode === 'admin' ) {
      var admin_panel = document.getElementById('admin_panel');
      admin_panel.style.display = 'block';
      connection.callService('/get_queue', '[]', function(queue) {
         var table = "<table>";
         for( var i=0; i<queue.programs.length; i++ ) {
            var p = queue.programs[i];
            table += "<tr><td>" + p.id + "</td>" + '<td>' + p.name + 
            ' <a href="javascript:run(' + p.id + ');">run</a></td></tr>';
         }
         table += "</table>";
         admin_panel.innerHTML = table;
         f();
      });
   }
}

// run a program
function run(id) {
   connection.callService('/run_program', '[' + token + ', ' + id + ']',
      function(n) {
         update_admin(nop);
      });
}
