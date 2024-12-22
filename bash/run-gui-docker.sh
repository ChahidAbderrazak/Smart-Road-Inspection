#### -----------------------   PREPARING THE WORKSPACE  -------------------------------
docker system prune -f
clear
. .env

###------------------------------------------------------------------------------

echo " ==> [${PROJECT_NAME}][Docker-compose] running the HAIS Desktop application  ..." && echo ""
docker compose run  \
    -v $(pwd)/data/download:/app/download \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -u appuser \
    app 
