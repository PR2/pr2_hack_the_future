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
import rospy

from program_queue.srv import *
from program_queue.msg import *
from std_srvs.srv import Empty

import sqlite3
import bcrypt
import random

class Queue:
   def __init__(self):
      # set up database

      # get the database path from the parameter server; or default to the
      #  user's ~/.ros/ directory
      self.dbpath = rospy.get_param('~dbpath', roslib.rosenv.get_ros_home() + '/program_queue.db')

      db = sqlite3.connect(self.dbpath)

      # create tables if they don't exist. 
      db.execute('create table if not exists users(id integer primary key asc autoincrement, name text unique not null, password_hash text not null, admin int not null)')
      db.execute('create table if not exists tokens(id integer primary key, user_id integer references users(id))') # TODO: add an issue/expiration date to tokens
      db.execute('create table if not exists programs(id integer primary key asc autoincrement, user_id integer references users(id), name text default "myprogram" not null, type integer, code text default "")')
      db.execute('create table if not exists output(id integer primary key asc autoincrement, program_id integer references programs(id), time text, output text)')
      db.execute('create table if not exists queue(id integer primary key asc autoincrement, program_id integer unique references programs(id))')

      # create an admin user if one doesn't exist
      admin_hash = bcrypt.hashpw('admin', bcrypt.gensalt())
      db.execute("insert or ignore into users (name, password_hash, admin) values (?, ?, ?)", ('admin', admin_hash, 1,))

      db.commit()
      db.close()

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


      rospy.loginfo("Queue ready")

   def db(self):
      return sqlite3.connect(self.dbpath)

   class User:
      def __init__(self, id, name, pwhash, admin):
         self.id = id
         self.name = name
         self.pwhash = pwhash
         self.admin = admin

   def get_user(self, db, token):
      # take a token and return a User object
      cur = db.cursor()
      cur.execute('select users.id, users.name, users.password_hash, users.admin from users join tokens on users.id = tokens.user_id where tokens.id = ?', (token,))
      row = cur.fetchone()
      if row:
         return Queue.User(row[0], row[1], row[2], row[3])
      else:
         # no tokens; return none
         return None

   def get_program_info(self, db, program_id):
      # take a program_id and return a Program object
      print program_id
      cur = db.cursor()
      cur.execute('select programs.id, programs.user_id, programs.name, programs.type, users.name from programs join users on programs.user_id = users.id where programs.id = ?', (program_id,))
      row = cur.fetchone()
      if row:
         return (row[1], ProgramInfo(row[0], row[2].encode('ascii'), row[3], row[4].encode('ascii')))
      else:
         return (None, None)

   def get_program(self, db, program_id):
      # take a program_id and return a Program object
      cur = db.cursor()
      cur.execute('select programs.id, programs.user_id, programs.name, programs.type, programs.code, users.name from programs join users on programs.user_id = users.id where programs.id = ?', (program_id,))
      row = cur.fetchone()
      if row:
         return (row[1], Program(ProgramInfo(row[0], row[2].encode('ascii'), row[3], row[5].encode('ascii')), row[4].encode('ascii')))
      else:
         return (None, None)

   def token(self, cur, uid):
      rows = 0
      t = 0
      while rows < 1:
         # generate a random 63-bit token
         t = random.getrandbits(63)
         # ensure that our token is never 0
         while t == 0:
            t = random.getrandbits(63)
         cur.execute('insert or ignore into tokens values (?, ?)', (t, uid))
         rows = cur.rowcount
         if rows < 1:
            rospy.logwarn("Tried to insert duplicate token %d"%t)
      return t

   def handle_clear_queue(self, req):
      db = self.db()
      user = self.get_user(db, req.token)
      if user and user.admin:
         db.execute('delete from queue')
         db.commit()
      elif user:
         rospy.loginfo("ClearQueue called by non-admin user %s" % user.name)
      db.close()
      return ClearQueueResponse()

   def handle_create_program(self, req):
      db = self.db()
      user = self.get_user(db, req.token)
      if user:
         rospy.loginfo("Creating new program for user %s"%user.name)
      else:
         # TODO: handle this failure case better
         rospy.logwarn("No user for token %d"%req.token)
         return False

      c = db.cursor()
      c.execute('insert into programs (user_id) values (?)', (user.id,))
      program_id = c.lastrowid
      c.close()
      db.commit()
      db.close()
      return CreateProgramResponse(program_id)

   def handle_create_user(self, req):
      pwhash = bcrypt.hashpw(req.password, bcrypt.gensalt())
      db = self.db()
      cur = db.cursor()
      cur.execute('insert or ignore into users (name, password_hash, admin) values (?, ?, ?)', (req.name, pwhash, 0,))
      if cur.rowcount > 0:
         userid = cur.lastrowid
         token = self.token(cur, userid)
         db.commit()
         db.close()
         return CreateUserResponse(token)
      else:
         rospy.loginfo("User %s already exists"%req.name)
         return CreateUserResponse(0)

   def handle_dequeue_program(self, req):
      # TODO
      return DequeueProgramResponse()

   def handle_get_my_programs(self, req):
      # TODO
      return GetMyProgramsResponse()

   def handle_get_output(self, req):
      # TODO
      return GetOutputResponse()

   def handle_get_program(self, req):
      db = self.db()
      (owner, program) = self.get_program(db, req.id)
      db.close()
      return GetProgramResponse(program)

   def handle_get_programs(self, req):
      # TODO
      return GetProgramsResponse()

   def handle_get_queue(self, req):
      db = self.db()
      cur = db.cursor()
      cur.execute('select programs.id, programs.name, programs.type, users.name from programs join queue on programs.id = queue.program_id join users on programs.user_id = users.id')
      resp = GetQueueResponse()
      for r in cur.fetchall():
         resp.programs.append(ProgramInfo(r[0], r[1].encode('ascii'), r[2], r[3].encode('ascii')))

      return resp

   def handle_login(self, req):
      db = self.db()
      cur = db.cursor()
      cur.execute('select id, password_hash from users where name = ?', (req.name,))
      row = cur.fetchone()
      if row == None:
         rospy.loginfo("No user named %s"%req.name)
         return LoginResponse(0)
      else:
         if bcrypt.hashpw(req.password, row[1]) == row[1]:
            token = self.token(cur, row[0])
            cur.close()
            db.commit()
            db.close()
            rospy.loginfo("Logged in %s"%req.name)
            return LoginResponse(token)
         else:
            rospy.loginfo("Password failed for %s"%req.name)
            return LoginResponse(0)

      return LoginResponse(0)

   def handle_logout(self, req):
      # TODO
      return LogoutResponse()

   def handle_queue_program(self, req):
      db = self.db()
      user = self.get_user(db, req.token)
      (owner, program) = self.get_program_info(db, req.program_id)
      p = 0
      if user and program:
         if user.admin or user.id == owner:
            db.execute('insert or ignore into queue (program_id) values (?)',
                  (req.program_id,))
            cur = db.cursor()
            # FIXME: only works when appending to queue. find an efficient way
            #  to do this
            cur.execute('select count(*) from queue')
            row = cur.fetchone()
            p = row[0] - 1
            cur.close()
            db.commit()
         else:
            rospy.loginfo("User %s does not have permission to put program %d into the queue"%(user.name, req.program_id))
      else:
         rospy.loginfo("Bad token %d or program id %d"%(token, req.program.info.id))

      db.close()
      return QueueProgramResponse(p)

   def handle_run_program(self, req):
      # TODO
      return RunProgramResponse()

   def handle_start_queue(self, req):
      # TODO: require an admin token
      return EmptyResponse()

   def handle_stop_queue(self, req):
      # TODO: require an admin token
      return EmptyResponse()

   def handle_update_program(self, req):
      db = self.db()
      user = self.get_user(db, req.token)
      (owner, program) = self.get_program_info(db, req.program.info.id)
      if user and program:
         if owner == user.id:
            db.execute('update programs set name=?, type=?, code=? where id=?',
                  (req.program.info.name, req.program.info.type, 
                     req.program.code, req.program.info.id))
            db.commit()
         else:
            rospy.loginfo("User %s does not own program %d"%(user.name, 
               req.program.info.id))
      else:
         rospy.loginfo("Bad token %d or program id %d"%(token, req.program.info.id))
      db.close()
      return UpdateProgramResponse()


if __name__ == '__main__':
   rospy.init_node('program_queue')

   queue = Queue()

   rospy.spin()
