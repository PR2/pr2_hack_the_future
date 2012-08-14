var editor;

var programs;

var selected = 0;

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
         editor.clearSelection();
         f.program_name.value = resp.program.info.name;
         programs[selected] = resp.program.info;
         update_list(selected);
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
         null);

   update_list(selected);
}

// create a new program
function newprogram(f) {
   var sample = document.getElementById("sample").innerHTML;
   editor.setValue(sample);
   editor.clearSelection();
   f.program_name.value = "New Program";

   connection.callService('/create_program', '[' + token + ']',
      function(resp) {
         console.log("Create new program: " + resp.id);
         var program = {};
         program.code = sample;
         program.info = {};
         program.info.id = resp.id;
         program.info.type = 1;
         program.info.name = "New Program";
         var sel = programs.length;
         programs.push(program.info);

         connection.callService('/update_program', 
            '[' + token + ', ' + JSON.stringify(program) + ']',
            null);

         update_list(sel);
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
         // display a program.
         //  if the last program is save in a cookie, show it
         //  else, show the first program
         programs = [];
         for (var i = 0 ; i < resp.programs.length ; i++ ) {
            // only show python programs
            if( resp.programs[i].type === 1 ) {
               programs.push(resp.programs[i]);
            }
         }

         // TODO: load program ID from cookie
         update_list(0);

         // load the selected program
         load(document.getElementById("program_info"));
      });
}
