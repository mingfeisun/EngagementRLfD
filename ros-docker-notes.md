## [How to use the ros docker image](https://hub.docker.com/_/ros/)


### Create a Dockerfile in your ROS app project

```bash
FROM ros:indigo
# place here your application's setup specifics
CMD [ "roslaunch", "my-ros-app my-ros-app.launch" ]
```

You can then build and run the Docker image:

```bash
docker build -t my-ros-app .
docker run -it --rm --name my-running-app my-ros-app
```

### Deployment use cases
**Deployment suggestions**

The available tags include supported distros along with a hierarchy tags based off the most common meta-package dependencies, designed to have a small footprint and simple configuration:

* *ros-core*: barebone ROS install
* *ros-base*: basic tools and libraries (also tagged with distro name with LTS version as latest)
* *robot*: basic install for robots
* *perception*: basic install for perception tasks

The rest of the common meta-packages such as desktop and desktop-full are hosted on automatic build repos under OSRF's Docker Hub profile. These meta-packages include graphical dependencies and hook a host of other large packages such as X11, X server, etc. So in the interest of keep the official images lean and secure, the desktop packages are just be hosted with OSRF's profile.

**Volumes**

ROS uses the ~/.ros/ directory for storing logs, and debugging info. If you wish to persist these files beyond the lifecycle of the containers which produced them, the ~/.ros/ folder can be mounted to an external volume on the host, or a derived image can specify volumes to be managed by the Docker engine. By default, the container runs as the root user, so /root/.ros/ would be the full path to these files.

For example, if one wishes to use their own .ros folder that already resides in their local home directory, with a username of ubuntu, we can simple launch the container with an additional volume argument:

```bash
docker run -v "/home/ubuntu/.ros/:/root/.ros/" ros
```

**Devices**

Some application may require device access for acquiring images from connected cameras, control input from human interface device, or GPUs for hardware acceleration. This can be done using the --device run argument to mount the device inside the container, providing processes inside hardware access.

**Networks**

The ROS runtime "graph" is a peer-to-peer network of processes (potentially distributed across machines) that are loosely coupled using the ROS communication infrastructure. ROS implements several different styles of communication, including synchronous RPC-style communication over services, asynchronous streaming of data over topics, and storage of data on a Parameter Server. To abide by the best practice of one process per container, Docker networks can be used to string together several running ROS processes. 

### Deployment example
If we want our all ROS nodes to easily talk to each other, we'll can use a virtual network to connect the separate containers. In this short example, we'll create a virtual network, spin up a new container running roscore advertised as the master service on the new network, then spawn a message publisher and subscriber process as services on the same network.

**Build image**

Build a ROS image that includes ROS tutorials using this Dockerfile:

```bash
# get ros
FROM ros:kinetic-ros-base

# install ros tutorials packages
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-tutorials \
    ros-kinetic-common-tutorials \
    && rm -rf /var/lib/apt/lists/
```



build the image from within the same directory

```bash
docker build --tag ros:ros-tutorials .
```

**Create network**

To create a new network foo, we use the network command:

```bash
docker network create foo
```
Now that we have a network, we can create services. Services advertise there location on the network, making it easy to resolve the location/address of the service specific container. We'll use this make sure our ROS nodes can find and connect to our ROS master.

**Run services**

To create a container for the ROS master and advertise its service:

```bash
docker run -it --rm \
    --net foo \
    --name master \
    ros:ros-tutorials \
    roscore
```

Now you can see that master is running and is ready manage our other ROS nodes. To add our talker node, we'll need to point the relevant environment variable to the master service:

```bash
docker run -it --rm \
    --net foo \
    --name talker \
    --env ROS_HOSTNAME=talker \
    --env ROS_MASTER_URI=http://master:11311 \
    ros:ros-tutorials \
    rosrun roscpp_tutorials talker
```

Then in another terminal, run the listener node similarly:

```bash
docker run -it --rm \
    --net foo \
    --name listener \
    --env ROS_HOSTNAME=listener \
    --env ROS_MASTER_URI=http://master:11311 \
    ros:ros-tutorials \
    rosrun roscpp_tutorials listener
```

Alright! You should see listener is now echoing each message the talker broadcasting. You can check like this:

```bash
# list containers
docker service ls

# list services
docker ps
```

**Introspection**

Ok, now that we see the two nodes are communicating, let get inside one of the containers and do some introspection what exactly the topics are:

```bash
docker exec -it master bash
source /ros_entrypoint.sh
```

If we then use rostopic to list published message topics, we should see something like this:

```bash
$ rostopic list
```

**Tear down**

To tear down the structure we've made, we just need to stop the containers and the services. We can stop and remove the containers using Ctrl^C where we launched the containers or using the stop command with the names we gave them:

```bash
docker stop master talker listener
docker rm master talker listener
```

**Compose**

Now that you have an appreciation for bootstrapping a distributed ROS example manually, lets try and automate it using docker-compose.

Start by making a folder named rostutorials and moving the Dockerfile we used earlier inside this directory. Then create a yaml file named docker-compose.yml in the same directory and paste the following inside:

```bash
version: '2'
services:
  master:
    build: .
    container_name: master
    command:
      - roscore

  talker:
    build: .
    container_name: talker
    environment:
      - "ROS_HOSTNAME=talker"
      - "ROS_MASTER_URI=http://master:11311"
    command: rosrun roscpp_tutorials talker

  listener:
    build: .
    container_name: listener
    environment:
      - "ROS_HOSTNAME=listener"
      - "ROS_MASTER_URI=http://master:11311"
    command: rosrun roscpp_tutorials listener
```

Now from inside the same folder, use docker-compose to launch our ROS nodes and specify that they coexist on their own network:

```bash
docker-compose up -d
```

Notice that a new network named rostutorials_default has now been created, we can do following operations:

```bash
# inspect it further
docker network inspect rostutorials_default

# monitor the logged output of each service
docker-compose logs listener

# stop and remove all the relevant containers
docker-compose stop
docker-compose rm
```

Note: the auto-generated network, rostutorials_default, will persist over the life of the docker engine or until you explicitly remove it using docker network rm.

