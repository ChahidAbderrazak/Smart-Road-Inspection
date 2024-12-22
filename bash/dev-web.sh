clear
# echo " ==> list of Docker containers :"
# docker ps -a
# docker run --rm -it \


echo " ==> [Docker-compose] running the HAIS Webserver ..." && echo ""
docker compose run  \
    -p 8080:8080 \
    -v $(pwd)/data/download:/app/data/download \
    -v $(pwd)/webapp:/app/ \
    -v $(pwd)/models:/app/models \
    webserver 
