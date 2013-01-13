#!/usr/bin/env python
#
# A program management and queueing system with a ROS service interface
#
# Author: Austin Hendrix

# Design notes:
#  At the moment, the db backend is sqlite. In the process of making things
#   concise I think I've abstacted this to the point that it could be easily
#   changed
#  Token IDs are random 63-bit numbers.
#  * When there are 4.31E8 tokens in existence, the probability of a collision
#    is 0.01
#  * When there are 1 million tokens in existence, the probability of a
#    collision is 5.4E-8
#  * See the Birthday problem for more in-depth coverage of the statistics
#    that makes this possible
#  * Token generation is checked and will retry until a new unique token is 
#    created
#  
#
# TODO: 
#  * more validation
#  * add username to ProgramInfo


import roslib; roslib.load_manifest('program_queue')
import rospkg
import rospy

from program_queue.srv import *
from program_queue.msg import *
from std_srvs.srv import Empty
from std_msgs.msg import Header

#import sqlite3
import MySQLdb as mysql
import bcrypt
import random
import subprocess
import base64
import threading

class Queue:
   def __init__(self):
      # set up database

      # get the database path from the parameter server; or default to the
      #  user's ~/.ros/ directory
      #self.dbpath = rospy.get_param('~dbpath', rospkg.get_ros_home() + '/program_queue.db')
      self.dbhost = rospy.get_param('~dbhost', 'demobase3')
      self.dbuser = rospy.get_param('~dbuser', 'willow')
      self.dbpass = rospy.get_param('~dbpass', 'willow')
      self.dbname = rospy.get_param('~dbpass', 'pr2_programs')

      self.dblock = threading.Lock()

      db = mysql.connect(self.dbhost, self.dbuser, self.dbpass, self.dbname)
      self.dbconn = db

      with self.dblock:
         cur = db.cursor()
   
         from warnings import filterwarnings, resetwarnings
         filterwarnings('ignore', category = mysql.Warning)
         # create tables if they don't exist. 
         cur.execute('create table if not exists users(id integer primary key auto_increment, name varchar(256) unique not null, password_hash varchar(60) not null, admin int not null)')
         cur.execute('create table if not exists tokens(id bigint primary key, user_id integer references users(id))') # TODO: add an issue/expiration date to tokens
         cur.execute('create table if not exists programs(id integer primary key auto_increment, user_id integer references users(id), name text not null, type integer, code text not null)')
         cur.execute('create table if not exists output(id integer primary key auto_increment, program_id integer references programs(id), time double not null, output text not null)')
         cur.execute('create table if not exists queue(id integer primary key auto_increment, program_id integer unique references programs(id))')
         resetwarnings()
   
         # create an admin user if one doesn't exist
         admin_hash = bcrypt.hashpw('admin', bcrypt.gensalt())
         cur.execute("insert ignore into users (name, password_hash, admin) values (%s, %s, %s)", ('admin', admin_hash, 1))
   
         db.commit()
         cur.close()

      # set up services
      rospy.Service('clear_queue',     ClearQueue,     self.handle_clear_queue)
      rospy.Service('create_program',  CreateProgram,  
            self.handle_create_program)
      rospy.Service('create_user',     CreateUser,     self.handle_create_user)
      rospy.Service('dequeue_program', DequeueProgram, 
            self.handle_dequeue_program)
      rospy.Service('get_my_programs', GetMyPrograms,  
            self.handle_get_my_programs)
      rospy.Service('get_output',      GetOutput,      self.handle_get_output)
      rospy.Service('get_program',     GetProgram,     self.handle_get_program)
      rospy.Service('get_programs',    GetPrograms,    self.handle_get_programs)
      rospy.Service('get_queue',       GetQueue,       self.handle_get_queue)
      rospy.Service('login',           Login,          self.handle_login)
      rospy.Service('logout',          Logout,         self.handle_logout)
      rospy.Service('queue_program',   QueueProgram,   
            self.handle_queue_program)
      rospy.Service('run_program',     RunProgram,     self.handle_run_program)
      rospy.Service('update_program',  UpdateProgram, 
            self.handle_update_program)

      rospy.Service('start_queue',     Empty,          self.handle_start_queue)
      rospy.Service('stop_queue',     Empty,           self.handle_stop_queue)

      #rospy.wait_for_service('run_slider_program')

      rospy.loginfo("Queue ready")

   def db(self):
      #return sqlite3.connect(self.dbpath)
      return self.dbconn

   class User:
      def __init__(self, id, name, pwhash, admin):
         self.id = id
         self.name = name
         self.pwhash = pwhash
         self.admin = admin

   def get_user(self, db, token):
      # take a token and return a User object
      cur = db.cursor()
      cur.execute('select users.id, users.name, users.password_hash, users.admin from users join tokens on users.id = tokens.user_id where tokens.id = %s', (token,))
      rows = cur.fetchall()
      #row = cur.fetchone()
      row = None
      if rows:
         row = rows[0]
      ret = None
      if row:
         ret = Queue.User(row[0], row[1], row[2], row[3])
      cur.close()
      return ret

   def get_program_info(self, db, program_id):
      # take a program_id and return a Program object
      cur = db.cursor()
      cur.execute('select programs.id, programs.user_id, programs.name, programs.type, users.name from programs join users on programs.user_id = users.id where programs.id = %s', (program_id,))
      row = cur.fetchone()
      ret = (None, None)
      if row:
         ret = (row[1], ProgramInfo(row[0], row[2].encode('ascii'), row[3], row[4].encode('ascii')))
      cur.close()
      return ret

   def get_program(self, db, program_id):
      # take a program_id and return a Program object
      cur = db.cursor()
      cur.execute('select programs.id, programs.user_id, programs.name, programs.type, programs.code, users.name from programs join users on programs.user_id = users.id where programs.id = %s', (program_id,))
      row = cur.fetchone()
      ret = (None, None)
      if row:
         ret = (row[1], Program(ProgramInfo(row[0], row[2].encode('ascii'), row[3], row[5].encode('ascii')), row[4].encode('ascii')))
      cur.close()
      return ret

   def token(self, cur, uid):
      rows = 0
      t = 0
      while rows < 1:
         # generate a random 63-bit token
         t = random.getrandbits(63)
         # ensure that our token is never 0
         while t == 0:
            t = random.getrandbits(63)
         cur.execute('insert ignore into tokens values (%s, %s)', (t, uid))
         rows = cur.rowcount
         if rows < 1:
            rospy.logwarn("Tried to insert duplicate token %d"%t)
      return t

   def handle_clear_queue(self, req):
      with self.dblock:
         db = self.db()
         cur = db.cursor()
         user = self.get_user(db, req.token)
         if user and user.admin:
            cur.execute('delete from queue')
            db.commit()
         elif user:
            rospy.loginfo("ClearQueue called by non-admin user %s" % user.name)
         cur.close()
         return ClearQueueResponse()

   def handle_create_program(self, req):
      with self.dblock:
         db = self.db()
         user = self.get_user(db, req.token)
         if user:
            rospy.loginfo("Creating new program for user %s"%user.name)
         else:
            # TODO: handle this failure case better
            rospy.logwarn("No user for token %d"%req.token)
            return False
   
         cur = db.cursor()
         cur.execute('insert into programs (user_id, name, code) values (%s, %s, %s)', (user.id, 'myprogram', ''))
         db.commit()
         program_id = cur.lastrowid
         cur.close()
         return CreateProgramResponse(program_id)

   def handle_create_user(self, req):
      pwhash = bcrypt.hashpw(req.password, bcrypt.gensalt())
      with self.dblock:
         db = self.db()
         cur = db.cursor()
         cur.execute('insert ignore into users (name, password_hash, admin) values (%s, %s, %s)', (req.name, pwhash, 0,))
         db.commit()
         if cur.rowcount > 0:
            userid = cur.lastrowid
            token = self.token(cur, userid)
            cur.close()
            return CreateUserResponse(token)
         else:
            cur.close()
            rospy.loginfo("User %s already exists"%req.name)
            return CreateUserResponse(0)

   def handle_dequeue_program(self, req):
      # TODO: test
      with self.dblock:
         db = self.db()
         user = self.get_user(db, req.token)
         if user:
            (owner, program) = self.get_program_info(db, req.id)
            if user.id != owner and not user.admin:
               rospy.loginfo("User %s is not allowed to dequeue %d"%(user.name, req.id))
            else:
               cur = db.cursor()
               cur.execute('delete from queue where program_id = %s',(req.id,))
               db.commit()
               cur.close()
         return DequeueProgramResponse()

   def handle_get_my_programs(self, req):
      resp = GetMyProgramsResponse()
      with self.dblock:
         db = self.db()
         user = self.get_user(db, req.token)
         if not user:
            # if we don't have a user, return an empty list
            return resp
         cur = db.cursor()
         cur.execute('select programs.id, programs.name, programs.type, users.name from programs join users on programs.user_id = users.id where users.id = %s', (user.id,))
         for r in cur.fetchall():
            resp.programs.append(ProgramInfo(r[0], r[1].encode('ascii'), r[2], r[3].encode('ascii')))
   
         cur.close()
         return resp

   def handle_get_output(self, req):
      resp = GetOutputResponse()
      with self.dblock:
         db = self.db()
         user = self.get_user(db, req.token)
         if user:
            (owner, program) = self.get_program_info(db, req.program_id)
            if user.id != owner:
               rospy.loginfo("User %s is not allowed to get output from program %d"%(user.name, req.program_id))
            else:
               cur = db.cursor()
               # TODO: enforce output limit
               cur.execute('select time, output from output where program_id = %s order by time desc',
                     (req.program_id,))
               for r in cur.fetchall():
                  resp.output.append(Output(Header(0, rospy.Time(r[0]), ''), r[1].encode('ascii')))
               cur.close()
   
         return resp

   def handle_get_program(self, req):
      with self.dblock:
         db = self.db()
         (owner, program) = self.get_program(db, req.id)
         return GetProgramResponse(program)

   def handle_get_programs(self, req):
      with self.dblock:
         db = self.db()
         cur = db.cursor()
         cur.execute('select programs.id, programs.name, programs.type, users.name from programs join users on programs.user_id = users.id')
         resp = GetProgramsResponse()
         for r in cur.fetchall():
            resp.programs.append(ProgramInfo(r[0], r[1].encode('ascii'), r[2], r[3].encode('ascii')))
         cur.close()
         return resp

   def handle_get_queue(self, req):
      with self.dblock:
         db = self.db()
         cur = db.cursor()
         cur.execute('select programs.id, programs.name, programs.type, users.name from programs join queue on programs.id = queue.program_id join users on programs.user_id = users.id order by queue.id')
         resp = GetQueueResponse()
         for r in cur.fetchall():
            resp.programs.append(ProgramInfo(r[0], r[1].encode('ascii'), r[2], r[3].encode('ascii')))
   
         cur.close()
         return resp

   def handle_login(self, req):
      with self.dblock:
         db = self.db()
         cur = db.cursor()
         cur.execute('select id, password_hash, admin from users where name = %s', (req.name,))
         row = cur.fetchone()
         if row == None:
            rospy.loginfo("No user named %s"%req.name)
            cur.close()
            return LoginResponse(0, False)
         else:
            if bcrypt.hashpw(req.password, row[1]) == row[1]:
               token = self.token(cur, row[0])
               rospy.loginfo("Logged in %s"%req.name)
               cur.close()
               return LoginResponse(token, row[2] != 0)
            else:
               rospy.loginfo("Password failed for %s"%req.name)
               cur.close()
               return LoginResponse(0, False)

   def handle_logout(self, req):
      with self.dblock:
         db = self.db()
         cur = db.cursor()
         cur.execute('delete from tokens where id = %s', (req.token,))
         db.commit()
         cur.close()
         return LogoutResponse()

   def handle_queue_program(self, req):
      with self.dblock:
         db = self.db()
         user = self.get_user(db, req.token)
         (owner, program) = self.get_program_info(db, req.program_id)
         p = 0
         if user and program:
            if user.admin or user.id == owner:
               cur = db.cursor()
               cur.execute('insert ignore into queue (program_id) values (%s)',
                     (req.program_id,))
               db.commit()
               # FIXME: only works when appending to queue. find an efficient way
               #  to do this
               cur.execute('select count(*) from queue')
               row = cur.fetchone()
               p = row[0] - 1
               cur.close()
            else:
               rospy.loginfo("User %s does not have permission to put program %d into the queue"%(user.name, req.program_id))
         else:
            rospy.loginfo("Bad token %d or program id %d"%(token, req.program.info.id))
   
         return QueueProgramResponse(p)

   def handle_run_program(self, req):
      # FIXME: stub
      db = self.db()
      user = self.get_user(db, req.token)
      if user and user.admin:
         with self.dblock:
            cur = db.cursor()
            cur.execute('select type, code from programs where id = %s', (req.id,))
            row = cur.fetchone()
            cur.close()
         if row:
            if row[0] == ProgramInfo.PYTHON:
               rospy.loginfo("Run python program %d"%(req.id))
               py = subprocess.Popen(['python'], stdin=subprocess.PIPE, 
                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
               output = py.communicate(row[1])[0]
            elif row[0] == ProgramInfo.PUPPET:
               output = "Puppet program execution is not supported"
               rospy.logerr(output)
            elif row[0] == ProgramInfo.SLIDER:
               rospy.loginfo("Run slider program %d"%(req.id))
               rospy.ServiceProxy('run_slider_program', CallProgram)(row[1])
               output = "Slider program run"
            elif row[0] == 4:
               rospy.loginfo("Run web slider program %d"%(req.id))
               rospy.ServiceProxy('/museum/run_web_slider_program', CallProgram)(row[1])
               output = "Web slider program run"
            else:
               output = "Error: Unknown program type " + row[0]
               rospy.logerr(output)
   
            with self.dblock:
               cur = db.cursor()
               cur.execute('insert into output (program_id, time, output) values'+
                  '(%s, %s, %s)', (req.id, rospy.Time.now().to_sec(), output,))
               cur.execute('delete from queue where program_id = %s', (req.id,))
               db.commit()
               cur.close()
         else:
            rospy.logerror("Bad program: " + req.id)
      else:
         rospy.loginfo("%s is not allowed to run programs"%user.name)
   
   
      return RunProgramResponse()

   def handle_start_queue(self, req):
      # TODO: require an admin token
      return EmptyResponse()

   def handle_stop_queue(self, req):
      # TODO: require an admin token
      return EmptyResponse()

   def handle_update_program(self, req):
      with self.dblock:
         db = self.db()
         user = self.get_user(db, req.token)
         (owner, program) = self.get_program_info(db, req.program.info.id)
         if user and program:
            # Food for thought: allow admins to edit any program?
            if owner == user.id:
               cur = db.cursor()
               cur.execute('update programs set name=%s, type=%s, code=%s where id=%s',
                     (req.program.info.name, req.program.info.type, 
                        req.program.code, req.program.info.id))
               db.commit()
               cur.close()
            else:
               rospy.loginfo("User %s does not own program %d"%(user.name, 
                  req.program.info.id))
         else:
            rospy.loginfo("Bad token %d or program id %d"%(req.token, req.program.info.id))
         return UpdateProgramResponse()


if __name__ == '__main__':
   rospy.init_node('program_queue')

   queue = Queue()

   rospy.spin()
