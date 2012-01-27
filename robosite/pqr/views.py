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
    if program:
        request.session['program'] = program
    form = ProgramForm(instance=program)
    del form.__dict__['fields']['ready_to_run']
    t = loader.get_template('programForm.html')
    c = buildContext(request, RequestContext(request), 'Program')
    c['form'] = form
    c.update(csrf(request))
    return HttpResponse(t.render(c))


def addProgram(request):
    form = ProgramForm(request.POST)
#     programmer = request.session.get('programmer',None)
#     form.instance.__dict__['programmer_id'] = programmer.id
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
        print "RUN PYTHON:"
        print program.code
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
    request.session['admin'] = True
    c.update(csrf(request))
    return HttpResponse(t.render(c))
    

