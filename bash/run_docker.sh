clear
# echo " ==> list of Docker containers :"
# docker ps -a
# docker run --rm -it \

###------------------------------------------------------------------------------
## Run dockers compose
echo " ==> [Docker-compose] running the HAIS Desktop application  ..." && echo ""
docker-compose run  \
    -v $(pwd)/data/download:/app/download \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -u appuser \
    app 

# echo " ==> [Docker-compose] running the HAIS Webserver ..." && echo ""
# docker-compose run  \
#     -p 8080:8080 \
#     -v $(pwd)/data/download:/app/download \
#     -u appuser \
#     webserver 
