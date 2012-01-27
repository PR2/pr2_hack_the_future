from django.db import models
from django import forms
from django.core.exceptions import ValidationError
from django.contrib.auth.models import User
from django.db.models.signals import post_save

def validate_age(value):
    if value < 2 or value > 100:
        raise ValidationError('Please enter your real age')

PROGRAMMING_LANGUAGE_CHOICES = (
    ('py', 'Python'),
    ('pu', 'Puppet Script'),
)

class Program(models.Model):
    programmer_name = models.CharField(max_length=200)
    program_name = models.CharField(max_length=200)
    code = models.TextField()
    language = models.CharField(choices=PROGRAMMING_LANGUAGE_CHOICES,
                                max_length=200,
                                default='py')
    date_added = models.DateTimeField(auto_now_add=True)
    ready_to_run = models.BooleanField(default=False)

    def __unicode__(self):
        return "%s-%s" % (self.programmer_name, self.program_name)
    
class Output(models.Model):
   date_run = models.DateTimeField(auto_now_add=True)
   stdout = models.TextField()
   stderr = models.TextField()
   program = models.ForeignKey(Program)

   def __unicode__(self):
      return "%s-%s at %s" % (self.program.programmer_name, 
            self.program.program_name, self.date_run)
    
