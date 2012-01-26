from django.template import Context, loader, Template
from robosite.pqr.models import Program
from django.http import HttpResponse, HttpResponseRedirect
from django.core.context_processors import csrf
from django.template import RequestContext
from django.forms import ModelForm
from django import forms
from django.template.defaultfilters import slugify
from django.contrib.auth.models import User
from django.contrib.auth import logout
from django.contrib.admin.views.decorators import staff_member_required

def buildContext(request, c, title = 'Home'):
    c['programmer'] = request.session.get('programmer',None)
    c['program'] = request.session.get('program',None)
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
#     programmer = request.session.get('programmer',None)
#     if programmer and not program:
#         programs = Program.objects.filter(programmer=programmer)
#         if len(programs) == 0:
#             program = None
#         else:
#             program = programs[0]
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
    c.update(csrf(request))
    return HttpResponse(t.render(c))
    


# def newProgrammer(request):
#     t = loader.get_template('programmerForm.html')
#     c = buildContext(request, RequestContext(request), 'Program')
#     c['form'] = ProgrammerForm()
#     c.update(csrf(request))
#     return HttpResponse(t.render(c))

# def addProgrammer(request):
#     form = ProgrammerForm(request.POST)
#     if form.is_valid():
#         programmer = User.objects.create_user(form.data['username'],
#                          form.data['email'],
#                          form.data['password'])
#         request.session['programmer'] = programmer
#         return HttpResponseRedirect('/program')
#     else:
#         t = loader.get_template('programmerForm.html')
#         c = buildContext(request, RequestContext(request), 'Programmer')
#         c['form'] = form
#         c.update(csrf(request))
#         return HttpResponse(t.render(c))

# def programmer(request):
#     programmer = request.session.get('programmer',None)
#     if programmer:
#         t = loader.get_template('programmer.html')
#         c = buildContext(request, RequestContext(request), 'Programmer')
#     else:
#         form = ProgrammerForm()
#         loginform = LoginForm()
#         t = loader.get_template('loginPage.html')
#         c = buildContext(request, RequestContext(request), 'Programmer')
#         c['form'] = form
#         c['loginform'] = loginform
#         c.update(csrf(request))
#     return HttpResponse(t.render(c))

# def signIn(request):
#     form = LoginForm(request.POST)
#     username = form.data['username']
#     password = form.data['password']
#     try:
#         programmer = Programmer.objects.get(username=username)
#         if password == programmer.password:
#             request.session['programmer'] = programmer
#             return HttpResponseRedirect('/program')
#         else:
#             raise forms.ValidationError('Invalid Username or Password')
#     except Programmer.DoesNotExist:
#         t = loader.get_template('loginPage.html')
#         c = buildContext(request, RequestContext(request), 'Programmer')
#         c['loginform'] = form
#         c['form'] = ProgrammerForm()
#         c.update(csrf(request))
#         return HttpResponse(t.render(c))
