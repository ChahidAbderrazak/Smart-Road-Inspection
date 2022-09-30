#!/bin/sh
mkdir $2
# ffmpeg -i $1 -vf scale="512:512" output.avi
ffmpeg -i output.avi -vf "select=not(mod(n\,5))" -vsync vfr -qscale:v 2 -start_number 0 $2/%05d.jpg
# rm output.avi