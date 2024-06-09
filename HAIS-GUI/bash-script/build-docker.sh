#!/bin/bash
echo " ==> [Docker] Building the docker image ..." && echo ""
docker image prune
echo ""
docker build -t hais-desktop:v1 .