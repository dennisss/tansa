#!/bin/bash

# To allow the simulation to run without an internet connect, this will download some of the models that sometimes need to be downloaded

cd config/gazebo/db

wget http://models.gazebosim.org/sun/model.tar.gz
tar -xvzf model.tar.gz
rm model.tar.gz

wget http://models.gazebosim.org/ground_plane/model.tar.gz
tar -xvzf model.tar.gz
rm model.tar.gz
