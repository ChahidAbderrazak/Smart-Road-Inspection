#!/bin/bash

# load the other env variables
. .env

# #### -----------------------   RUNNING THE PROJECT DOCKER  -------------------------------
# docker compose  -p "${PROJECT_NAME}" -f docker-compose.yml up -d

#### -----------------------   IP Adresses  -------------------------------
echo && echo "[${PROJECT_NAME}][IP Adresses] Getting the IP adresses of the different servers..."
# # for Dockerfile
# docker inspect -f '{{.Name}} - {{.NetworkSettings.IPv4Address }}' $(docker ps -aq) > .env-ip

# for docker-compsoe
docker inspect -f '{{.Name}} - {{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $(docker ps -aq) > .env-ip
sed -i 's/ - /=/g' .env-ip
sed -i 's|/||g' .env-ip
sed -i '/=$/d' .env-ip
# cat .env-ip
. .env-ip

#### -----------------------   RUNNING SERVERS  -------------------------------

echo && echo "[${PROJECT_NAME}][Servers] Open the project server(s) on the browser"
#### -----------------------   APP  -------------------------------
eval "APP_CNTNR_IP=\$$SQL_CNTNR_NAME"
if [ "$APP_CNTNR_IP" != "" ] ; then
	SQL_HOST="$APP_CNTNR_IP"
	echo && echo "-- SQL_HOST = ${SQL_HOST}"
	sed -i '/SQL_HOST/d' .env-ip
	echo "SQL_HOST=${SQL_HOST}" >>.env-ip
	# xdg-open "${project_SERVER_URL}"
fi

# cp .env-ip ../.env-ip