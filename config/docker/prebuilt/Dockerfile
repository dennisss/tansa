
FROM dennisss/tansa-dev:latest

RUN cd /opt; git clone https://github.com/dennisss/tansa
RUN cd /opt/tansa; git checkout new; npm install; make build; make build_firmware

CMD cd /opt/tansa; ./start.sh
