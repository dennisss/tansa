
FROM dennisss/tansa-dev:latest

RUN cd /opt; git clone https://github.com/dennisss/tansa
RUN cd /opt/tansa; git checkout new; make build
RUN cd /opt/tansa; npm install; make build_firmware

CMD cd /opt/tansa; ./start.sh
