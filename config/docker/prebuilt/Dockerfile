
FROM dennisss/tansa-dev:latest

RUN cd /opt; git clone https://github.com/dennisss/tansa
RUN cd /opt/tansa; make build
RUN cd /opt/tansa; npm install; make build_firmware

# Preload all the models as we will assume that no internet connection is available
RUN cd /opt/tansa; ./config/gazebo/preload.sh

CMD cd /opt/tansa; bash ./start.sh
