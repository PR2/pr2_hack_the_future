#!/bin/bash

if [ ! -e queue-web ] 
then
   echo "current directory should be the queue_web package"
   exit 1
fi

cp -i queue-web /etc/apache2/sites-available/

a2dissite default
a2dissite webui
a2ensite queue-web

WWW=/var/queue-web

if [ ! -e /var/queue-web ] 
then
   DIR=`pwd`
   echo $DIR
   ln -s $DIR/www $WWW
else
   echo "$WWW already exists; please make sure it is correct"
fi
