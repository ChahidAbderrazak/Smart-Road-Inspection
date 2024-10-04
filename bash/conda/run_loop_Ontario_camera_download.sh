#!/bin/bash
#####################  Ontario511 CAMERA LIVE  ##########################
clear

cd src
cnt=0
while [ 0 -le $cnt ]
do
  cnt=$(( $cnt + 1 ))
  bash bash/run_Ontario511_download.sh $cnt
done

