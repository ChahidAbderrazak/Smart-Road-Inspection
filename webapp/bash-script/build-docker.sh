 #!/bin/bash
echo " ==> [Docker] Building the HAIS Webserver ..." && echo ""
docker system prune -f

# Build dockers 
docker build -t hais-webserver:v1 .
