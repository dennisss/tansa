
FROM dennisss/tansa-dev:latest

RUN cd /opt; git clone https://github.com/dennisss/tansa
RUN cd /opt/tansa; git checkout new; make build; make build_firmware

CMD /opt/tansa/start.sh
