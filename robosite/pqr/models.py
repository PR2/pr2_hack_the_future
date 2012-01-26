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

# class Programmer(models.Model):
#     user = models.OneToOneField(User)
#     age = models.IntegerField(validators=[validate_age], null=True)
# #    picture = models.ImageField()


# def create_user_profile(sender, instance, created, **kwargs):
#     if created:
#         Programmer.objects.create(user=instance)

# post_save.connect(create_user_profile, sender=User)

class Program(models.Model):
#    programmer = models.ForeignKey(User, editable=False)
    programmer_name = models.CharField(max_length=200)
    program_name = models.CharField(max_length=200)
    code = models.TextField()
    language = models.CharField(choices=PROGRAMMING_LANGUAGE_CHOICES,
                                max_length=200,
                                default='py')
    date_added = models.DateTimeField(auto_now_add=True)
    ready_to_run = models.BooleanField(default=False)

    def __unicode__(self):
        return self.program_name
    
