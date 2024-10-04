clear
echo " ==> [Docker] running the HAIS Desktop code ..." && echo ""
# docker run -p 127.0.0.1:3000:3000 hais-desktop

docker run --rm -it \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    -u appuser \
    hais-desktop:v1