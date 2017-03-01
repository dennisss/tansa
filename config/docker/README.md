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
	- Run `docker run --rm -p 3000:3000 tansa-prebuilt` every time you want to start it
	- Rerun the `pull` command from above whenever there are new changes to this repo


Advanced
--------

These are the commands I use to

- `docker build -t dennisss/tansa-prebuilt .`
	- `docker tag 7d9495d03763 dennisss/tansa-prebuilt:latest` (optionally if tagged with something else using the tag id from `docker images`)
	- The `--no-cache=true` option for building is also very useful
- `docker login`
- `docker push dennisss/tansa-prebuilt`
- Handy command to start up a shell `docker exec -it tansa-dev bash`
