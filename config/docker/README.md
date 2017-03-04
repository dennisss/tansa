Docker Images
=============

Included here are the Docker images we use in our test environment


Overview
--------

All these images are available prebuilt on Docker Hub.

- `dev` is used for automated CI testing. It only has core dependencies installed without any tansa code
- `prebuilt` contains the tansa repository prebuilt with the run command defaulting to the quickstart script


Quickstart
----------

- To get a running environment, do
	- Run `docker pull dennisss/tansa-prebuilt:latest`
	- Run `docker run --rm -p 4000:4000 dennisss/tansa-prebuilt` every time you want to start it
	- Rerun the `pull` command from above whenever there are new changes to this repo

- For development, you can use the `tansa-dev` image to run Tansa out of a local git repository.
 	- In the source folder, run ``docker run -v `pwd`:`pwd`:rw -w=`pwd` -p 4000:4000 -it dennisss/tansa-dev bash``
	- The above command will give you a shell which you can use to do `make build`, etc.
	- You can run the Gazebo simulator in this mode, but only headless


Advanced
--------

These are the commands I use to

- `docker build -t dennisss/tansa-prebuilt .`
	- `docker tag 7d9495d03763 dennisss/tansa-prebuilt:latest` (optionally if tagged with something else using the tag id from `docker images`)
	- The `--no-cache=true` option for building is also very useful
- `docker login`
- `docker push dennisss/tansa-prebuilt`
- Handy command to start up a shell `docker run -it tansa-dev bash`
