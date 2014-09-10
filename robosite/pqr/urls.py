from django.conf.urls.defaults import *
import django

import views

urlpatterns = patterns('',
    (r'^$', views.index),
    (r'^home/$', views.index),
    (r'^programmer/$', 'django.contrib.auth.views.login', 
     {'template_name': 'login.html'}),
    (r'^program/$', views.program),
    (r'^addProgram/$', views.addProgram),
    (r'^showProgram/(?P<id>.*)/$', views.showProgram),
    (r'^runProgram/(?P<id>.*)/$', views.runProgram),
    (r'^output/(?P<program_id>.*)/$', views.output),
    (r'^queue/$', views.queue),
    (r'^run/$', views.run),
    (r'^administer/$', views.admin),
    (r'^logout/$', views.logout),
)
