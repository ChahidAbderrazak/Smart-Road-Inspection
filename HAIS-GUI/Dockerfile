FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive

# Add user
RUN adduser --quiet --disabled-password appuser && usermod -a -G audio appuser

# This fix: libGL error: No matching fbConfigs or visuals found
ENV LIBGL_ALWAYS_INDIRECT=1

# # This fix: qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
# ENV QT_DEBUG_PLUGINS=1

# copy te code files
WORKDIR /app
COPY ./ /app

# Install Python 3, PyQt5
RUN apt-get update && apt-get install -y python3-pip python3-pyqt5 

# Install pip requirements
COPY requirements.txt .
RUN pip3 install -r requirements.txt

## avoid PyQT issue :  no Qt platform plugin could be initialized
RUN pip3 uninstall -y opencv-python

# ENV DEBIAN_FRONTEND teletype

# set the execution commande
# CMD ["python3", "main.py", "--cfg", "config.yml"]
CMD ["python3", "main_gui.py", "--cfg", "config.yml"]