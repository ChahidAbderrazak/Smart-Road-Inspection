#/bin/bash
#### -----------------------   PREPARING THE WORKSPACE  -------------------------------
clear
docker system prune -f
. .env

#### -----------------------   BUILDING THE PROJECT DOCKER  -------------------------------
# build  the the project containers
echo && echo "[${PROJECT_NAME}][Docker-Compose] Building the app containers... "
docker-compose  -p "${PROJECT_NAME}" -f docker-compose.yml up -d --build

#### -----------------------   GENERATE THE DOCKER IP ADRESSES  -------------------------------
bash bash/open-servers-browser.sh

#### ----------------   NOTIFICATION MESSAGE -------------------------
notify-send "[${PROJECT_NAME}][Docker-Compose] is built successfully!!"