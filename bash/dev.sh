clear
# echo " ==> list of Docker containers :"
# docker ps -a
# docker run --rm -it \

###------------------------------------------------------------------------------
## Run dockers compose
echo " ==> [Docker-compose] running the HAIS Desktop application  ..." && echo ""
docker compose run  \
    -v $(pwd)/src:/app/src \
    -v $(pwd)/data/download:/app/data/download \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -u appuser \
    app 

