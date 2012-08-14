var editor;

var programs;

function load(f) {
   var id = programs[f.program.selectedIndex].id;
   connection.callService('/get_program', JSON.stringify([id]),
         function(resp) {
            editor.setValue(resp.program.code);
            editor.clearSelection();
            f.program_name.value = resp.program.info.name;
         });
}

function save(f) {
   /*
   var program = {};
   program.info = {};
   program.info.name = f.program_name.value;
   program.info.type = 1;
   // program.info.owner is ignored.
   program.code = editor.getValue();

   connection.callService('/update_program', JSON.stringify([program]),
         null);
         */
}

function newprogram(f) {
   var sample = document.getElementById("sample").innerHTML;
   editor.setValue(sample);
   editor.clearSelection();
   f.program_name.value = "New Program";

   connection.callService('/create_program', '[' + token + ']',
         function(resp) {
            console.log("Create new program: " + resp.id);
            var program = {};
            program.info = {};
            program.code = sample;
            program.info.id = resp.id;
            program.info.type = 1;
            program.info.name = "New Program";
            programs = [];
            var p = {};
            p.token = token;
            p.program = program;
            console.log(JSON.stringify([p]));
            connection.callService('/update_program', JSON.stringify([p]),
               function(resp) {
                  console.log("Successfully updated new program");
                  get_programs();
               });
         });
}

function start_editor() {
   editor = ace.edit("editor");
   var PythonMode = require("ace/mode/python").Mode;
   editor.getSession().setMode(new PythonMode());
}

function get_programs() {
   // get program list; display in selection box
   connection.callService('/get_my_programs', '[' + token + ']',
         function(resp) {
            // display a program.
            //  if the last program is save in a cookie, show it
            //  else, show the first program
            resp.programs;

            var form = document.getElementById("program_info");
            form.program.options = [];
            console.log(form.program.options);
            programs = [];
            for (var i = 0 ; i < resp.programs.length ; i++ ) {
               console.log(resp.programs[i].type);
               if( resp.programs[i].type === 1 ) {
                  var opt = document.createElement('option');
                  opt.text = "(" + resp.programs[i].id + "): " + 
                     resp.programs[i].name;
                  opt.value = opt.text;
                  form.program.add(opt, null);
                  programs.add(resp.programs[i]);
               }
            }

            load(form);
         });

}
