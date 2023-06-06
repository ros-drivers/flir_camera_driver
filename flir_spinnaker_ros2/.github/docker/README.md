# How to update the docker images

## For Galactic on Ubuntu 20.04
### Create the base image (ROS1 and ROS2 combined)
Change into the ``.github/docker`` directory and build the base image:

```
dockerhub_name=<your_dockerhub_name_here>
os_flavor=focal
ros1_flavor=noetic
ros2_flavor=galactic
combined=${os_flavor}_${ros1_flavor}_${ros2_flavor}
docker build -t ${dockerhub_name}/${combined} -f Dockerfile.${combined} .
```

Upload the base image to dockerhub:
```
docker login
docker push ${dockerhub_name}/${combined}
```

### Build docker image with Spinnaker SDK and other missing packages on it

Download the Spinnaker SDK and copy it into the .github/docker
directory. Uncompress/untar it there. Hack the install_spinnaker.sh
file to work interactively. This can be done by substituting ``read
confirm`` by ``confirm=no`` except for the first two prompts where you
need ``confirm=yes``. Also change the dpkg install commands like so:
```
yes | sudo dpkg -i libspinnaker_*.deb
```

Build and push image with remaining packages:
```
docker build -t ${dockerhub_name}/${combined}_spinnaker -f Dockerfile.${combined}_spinnaker .
docker push ${dockerhub_name}/${combined}_spinnaker
```

## For Foxy on Ubuntu 20.04

```
dockerhub_name=<your_dockerhub_name_here>
os_flavor=focal
ros2_flavor=foxy
combined=${os_flavor}_${ros2_flavor}_spinnaker
docker build -t ${dockerhub_name}/${combined} -f Dockerfile.${combined} .
docker push ${dockerhub_name}/${combined}
```

## For Humble on Ubuntu 22.04

NOTE: at the moment the FLIR spinnaker SDK requires qt5-default, which is not present on Ubuntu 22.04.
To create the dockerhub image, a
[fake qt5-package was created](https://askubuntu.com/questions/1335184/qt5-default-not-in-ubuntu-21-04).
```
dockerhub_name=<your_dockerhub_name_here>
os_flavor=jammy
ros2_flavor=humble
combined=${os_flavor}_${ros2_flavor}_spinnaker
docker build -t ${dockerhub_name}/${combined} -f Dockerfile.${combined} .
docker push ${dockerhub_name}/${combined}
```
