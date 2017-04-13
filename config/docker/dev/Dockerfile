
FROM ubuntu:16.04

RUN apt-get update

RUN apt-get install -y --force-yes build-essential wget git cmake

# Gazebo simulator & Eigen & Boost
RUN apt-get install -y --force-yes libprotobuf-dev libprotoc-dev protobuf-compiler libeigen3-dev gazebo7 libgazebo7-dev

RUN apt-get install -y --force-yes libgtest-dev
RUN cd /usr/src/gtest; cmake CMakeLists.txt; make; cp *.a /usr/lib

RUN apt-get install -y --force-yes curl
RUN curl -sL https://deb.nodesource.com/setup_6.x | bash -
RUN apt-get install -y --force-yes nodejs

# We only get QT5 because CGAL 4.7 has a CMAKE issue (should fix itself once ubuntu updates to 4.8)
RUN apt-get install -y --force-yes libcgal-dev libcgal-qt5-dev

# Needed for building the firmware
RUN apt-get install -y python-setuptools python-dev build-essential

RUN apt-get install -y python-empy
RUN easy_install pip
RUN pip install catkin_pkg
RUN apt-get install -y libopencv-dev

RUN apt-get install -y sudo

RUN apt-get install -y python-jinja2
