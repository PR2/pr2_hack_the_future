from django.template import Context, loader, Template
from robosite.pqr.models import Program
from robosite.pqr.models import Output
from django.http import HttpResponse, HttpResponseRedirect
from django.core.context_processors import csrf
from django.template import RequestContext
from django.forms import ModelForm
from django import forms
from django.template.defaultfilters import slugify
from django.contrib.auth.models import User
from django.contrib.auth import logout
from django.contrib.admin.views.decorators import staff_member_required

import subprocess
import tempfile
import stat
import os

def buildContext(request, c, title = 'Home'):
    c['program'] = request.session.get('program',None)
    c['admin'] = request.session.get('admin',None)
    c['title'] = title
    c['slug'] = slugify(title)
    return c


class ProgrammerForm(forms.Form):
    username = forms.CharField("Username")
    password = forms.CharField("Password",widget=forms.PasswordInput())
    password2 = forms.CharField("Password (again)",widget=forms.PasswordInput())
    first_name = forms.CharField("First Name")
    last_name = forms.CharField("Last Name")
    email = forms.CharField("Email")
    age = forms.CharField("Age")

class ProgramForm(ModelForm):
    def __init__(self, *args, **kwargs):
        super(ProgramForm, self).__init__(*args, **kwargs)

    class Meta:
        model = Program


def index(request):
    t = loader.get_template('index.html')
    c = buildContext(request, RequestContext(request), 'Home')
    return HttpResponse(t.render(c))

    

def program(request):
    program = request.session.get('program',None)
    output = None
    if program:
        request.session['program'] = program
        output = Output.objects.filter(program=program).order_by('date_run')
    form = ProgramForm(instance=program)
    del form.__dict__['fields']['ready_to_run']
    t = loader.get_template('programForm.html')
    c = buildContext(request, RequestContext(request), 'Program')
    c['form'] = form
    c['output'] = output
    c.update(csrf(request))
    return HttpResponse(t.render(c))

def output(request,program_id):
    program = Program.objects.get(id=program_id)
    if program:
        outputs = Output.objects.filter(program=program).order_by('date_run')
        if len(outputs) > 0:
            output = outputs[0]
        else:
            output = None
        t = loader.get_template('output.html')
        c = buildContext(request, RequestContext(request), 'Output')
        c['program'] = program
        c['output'] = output
        c.update(csrf(request))
        return HttpResponse(t.render(c))
    else:
        return HttpResponseRedirect('/')
        

def addProgram(request):
    form = ProgramForm(request.POST)
    if form.is_valid():
        program = form.save()
        request.session['program'] = program
        return HttpResponseRedirect('/program')
    else:
        t = loader.get_template('programForm.html')
        c = buildContext(request, RequestContext(request), 'Program')
        c['form'] = form
        c.update(csrf(request))
        return HttpResponse(t.render(c))

def showProgram(request,id):
    program = Program.objects.get(id=id)
    t = loader.get_template('program.html')
    c = buildContext(request, RequestContext(request), 'Program')
    c['program'] = program
    c.update(csrf(request))
    return HttpResponse(t.render(c))
    
@staff_member_required
def runProgram(request, id):
    program = Program.objects.get(id=id)
    if program.language == "py":
        program.ready_to_run = False
        program.save()

        (tmp, path) = tempfile.mkstemp(prefix="pqr", suffix=".py")
        os.write(tmp, "#!/bin/bash\n")
        os.write(tmp, "source /u/future/setup.sh\n")
        os.write(tmp, "python << EOF \n")
        os.write(tmp, program.code)
        os.write(tmp, "\n")
        os.write(tmp, "EOF\n")
        os.close(tmp)
        
        os.chmod(path, stat.S_IRUSR | stat.S_IXUSR)
        
        tmp = subprocess.Popen(path, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (o,e) = tmp.communicate()
        tmp.wait()

        os.remove(path)
        
        output = Output.objects.create(stderr=e, stdout=o,
              program=program)
        output.save()
        t = loader.get_template('output.html')
        c = buildContext(request, RequestContext(request), 'Admin')
        c['program'] = program
        c['output'] = output
        c.update(csrf(request))
        return HttpResponse(t.render(c))
    elif program.language == "pu":
        print "RUN PUPPET SCRIPT:"
        print program.code
        program.ready_to_run = False
        program.save()

        (tmp, path) = tempfile.mkstemp(prefix="pqr", suffix=".py")
        os.write(tmp, "#!/bin/bash\n")
        os.write(tmp, "source /u/future/setup.sh\n")
        os.write(tmp, "python << EOF \n")
        os.write(tmp, """import roslib; roslib.load_manifest('gesture_builder_headless')
import rospy
import actionlib

from gesture_builder_headless.msg import *

GOAL_TIMEOUT = 120.0; # sec

if __name__ == '__main__':
    rospy.init_node('headless_test')
    client = actionlib.SimpleActionClient('gesture_run_program', gesture_run_programAction);
    rospy.loginfo("Waiting for gesture_builder_headless service...");
    client.wait_for_server()
    rospy.loginfo("gesture_builder_headless service is online.");

    goal = gesture_run_programGoal();

    goal.programID = "%s";
    goal.program = ["""%program)

        lines = program.code.split("\r\n")
        for l in lines:
           os.write(tmp, "\"%s\","%l)    
        os.write(tmp, """                    ];
    client.send_goal(goal)
    programRunFinished = client.wait_for_result(rospy.Duration.from_sec(GOAL_TIMEOUT));
    if programRunFinished:
        resultMsg = client.get_result();
        rospy.loginfo("Program " + goal.programID + " finished. Result msg: programID: " +\\
                      resultMsg.programID +\\
                      "; outcome: " + str(resultMsg.outcome) + ". ErrorMsg: '" + resultMsg.errorMessage + "'");
    else:
        rospy.loginfo("Program " + goal.programID + " never finished; timed out.");""")
        os.write(tmp, "\n")
        os.write(tmp, "EOF\n")
        os.close(tmp)
        
        os.chmod(path, stat.S_IRUSR | stat.S_IXUSR)
        
        tmp = subprocess.Popen(path, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        (o,e) = tmp.communicate()
        tmp.wait()

        os.remove(path)
        
        output = Output.objects.create(stderr=e, stdout=o,
              program=program)
        output.save()
        t = loader.get_template('output.html')
        c = buildContext(request, RequestContext(request), 'Admin')
        c['program'] = program
        c['output'] = output
        c.update(csrf(request))
        return HttpResponse(t.render(c))
    else:
        print "Unrecognized program language"
        pass
    return HttpResponseRedirect('/administer')

def run(request):
    program = request.session.get('program',None)
    if program:
        program.ready_to_run = True
        program.save()
        return queue(request)
    else:
        return HttpResponseRedirect('/program')
    

def queue(request):
    programs = Program.objects.filter(ready_to_run=True)
    t = loader.get_template('queue.html')
    c = buildContext(request, RequestContext(request), 'Queue')
    c['programs'] = programs
    c['allprograms'] = Program.objects.all()
    c.update(csrf(request))
    return HttpResponse(t.render(c))
    

def logout(request):
    request.session.flush()
    return HttpResponseRedirect('/')

@staff_member_required
def admin(request):
    programs = Program.objects.filter(ready_to_run=True)
    t = loader.get_template('adminqueue.html')
    c = buildContext(request, RequestContext(request), 'Admin')
    c['programs'] = programs
    c['allprograms'] = Program.objects.all()
    request.session['admin'] = True
    c.update(csrf(request))
    return HttpResponse(t.render(c))
    
