#!/bin/sh
echo && echo '   --> Converting the video to image : [file=$1]'
dst=${1%.*} ; mkdir $dst
ffmpeg -i $1 -vf "select=not(mod(n\,5))" -vsync vfr -qscale:v 2 -start_number 0 $dst/%05d.jpg