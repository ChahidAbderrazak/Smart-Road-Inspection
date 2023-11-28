clear
cd ..
root=$(pwd)
# root=/?media/abdo2020/DATA1/running/GitHub/Smart-Road-Inspection
echo " - root folder: $root" && echo ""
echo " ==> [Docker] running the HAIS Visualization   ..." && echo ""
docker run --rm -it   \
    -p 8080:8080 \
    -v $root/config:/app/config \
    -v $root/workspace/data/download:/data/download \
    -v $root/workspace/output:/app/output \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    hais-webserver:v1

