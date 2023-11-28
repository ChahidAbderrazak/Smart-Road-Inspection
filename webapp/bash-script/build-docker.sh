 #!/bin/bash
echo " ==> [Docker] Building the HAIS Webserver ..." && echo ""
docker system prune

# Build dockers 
docker build -t hais-webserver:v1 .
