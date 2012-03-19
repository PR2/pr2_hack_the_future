#!/bin/bash
# Re-generate the .hgignore file for this repository

# Base patterns; add static things here
cat <<EOF > .hgignore
.*/build
.*/srv_gen
.*/msg_gen
.*/bin
.*\.so$
.*\.pyc
EOF

# generate ignores for generated messages in (.*)/src/$1
for d in *
do
   if [ -d $d ]
   then
      echo ^$d/src/$d >> .hgignore
   fi
done

# generate ignores for messages generated from actionlib
for d in *
do
   if [ -d $d/action ]
   then
      (cd $d/action
      for action in *
      do
         action=${action%.action}
         echo $d/msg/$action.*
      done) >> .hgignore
   fi
done
