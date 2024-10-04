#### -----------------------   PREPARING THE WORKSPACE  -------------------------------
docker system prune -f
clear
. .env

###------------------------------------------------------------------------------

echo " ==> [${PROJECT_NAME}][Docker-compose] running the HAIS Webserver ..." && echo ""
docker-compose run  \
    -p 8080:8080 \
    -v $(pwd)/data/download:/app/download \
    webserver 
