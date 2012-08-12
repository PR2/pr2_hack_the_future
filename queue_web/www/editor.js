var editor;

function load(f) {
}

function save(f) {
}

function newprogram(f) {
   editor.setValue(document.getElementById("sample").innerHTML);
   editor.clearSelection();
   f.program_name.value = "New Program";
}

function start_editor() {
   editor = ace.edit("editor");
   var PythonMode = require("ace/mode/python").Mode;
   editor.getSession().setMode(new PythonMode());
}
