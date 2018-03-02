# [Get started with Docker](https://docs.docker.com/get-started/)

## Docker concepts

Docker is a platform for developers and sysadmins to develop, deploy and runapplications with containers.

Containerization is increasingly popular because containers are:

* Flexible: Even the most complex applications can be containerized
* Lightweight: Containers leverage and share the host kernel
* Interchangeable: You can deploy updates and upgrades on-the-fly
* Portable: You can build locally, deploy to the cloud, and run anywhere
* Scalable: You can increase and automatically distribute container replicas
* Stackable: You can stack services vertically and on-the-fly


### Images and containers
A container is launched by running an image. An Image is an executable package that includes everything needed to run an application - the code, a runtime, libraries, environment variables, and configuration files.

A container is a runtime instance of an image - what the image becomes in memory when executed (that is, an image with state, or a user process). You can see a list of your running containers with the command, *docker ps*, just as you would in Linux.

### Containers and virtual machines
A container runs natively on Linux and shares the kernel of the host machine with other containers. It runs a discrete process, taking no more memory than any other executable, making it lightweight.

By contrast, a virtual machine runs a full-blown "guest" operating system with virtual access host resources through a hypervisor. In general, virtual machines provide an environment with more resources than most applications need.

## Prepare your Docker environment
Mange Docker as a non-root user

```bash
# 1.create the docker group
sudo groupadd docker

# 2.add user to the docker group
sudo usermod -aG docker $USER

# 3.apply new membership
su - $USER

# 4.verify
docker run hello-world
```

Docker commands

```bash
# testing Docker version
docker --version

# for more details
docker version
docker info

# for testing
docker run hello-world

# list image
docker image ls

# list containers (running, all, all in quiet mode)
docker container ls
docker container ls -all
docker container ls -a -q
```

## Develop an app the Docker way
We start at the bottom of the hierarchy of such an app, which is a container. Above this level is a service, which defines how containers behave in production. Finally, at the top level is the stack, defining the interactions of all the services.

* Stack
* Services
* Container

In the past, if you were to start writing a Python app, your first order of business was to install a Python runtime onto your machine. But, that creates a situation where the environment on your machine needs to be perfect for your app to run as expected, and also needs to match your production environment.

With Docker, you can just grab a portable Python runtime as an image, no installation encessary. Then, your build can include the base Python Image right alongside your app code, ensuring that your app, its dependencies, and the runtime, all travel together.


### Container
*Dockerfile* defines what goes on in the environment inside your container. Access to resources like networking interfaces and disk drivers is virtualized inside this environment, which is isolated from the rest of your system, so you need to map ports to the outside world, and be specific about what files you want to "copy in" to that environment. However, after doing that, you can expect that the build of your app defined in this *Dockerfile* behaves exactly the same wherever it runs.

*Dockerfile*

```bash
FROM python:2.7-slim
WORKDIR /app
ADD . /app
RUN pip install --trusted-host pypi.python.org -r requirements.txt
EXPOSE 80
ENV NAME World
CMD ["python", "app.py"]
```

Proxy servers can block connections to your web app once it's up and running. If you are behind a proxy server, add the following lines to your Dockerfile, using the ENV command to specify the host and port for your proxy servers:

```bash
ENV http_proxy host:port
ENV https_proxy host:port
```
Add these lines before the call to pip so that the installation succeeds.

*requirements.txt*

```bash
Flask
Redis
```

*app.py*

```python
from flask import Flask
from redis import Redis, RedisError
import os
import socket

# Connect to Redis
redis = Redis(host="redis", db=0, socket_connect_timeout=2, socket_timeout=2)

app = Flask(__name__)

@app.route("/")
def hello():
try:
visits = redis.incr("counter")
except RedisError:
visits = "<i>cannot connect to Redis, counter disabled</i>"

html = "<h3>Hello {name}!</h3>" \
"<b>Hostname:</b> {hostname}<br/>" \
"<b>Visits:</b> {visits}"
return html.format(name=os.getenv("NAME", "world"), hostname=socket.gethostname(), visits=visits)

if __name__ == "__main__":
app.run(host='0.0.0.0', port=80)
```

Note:  Accessing the name of the host when inside a container retrieves the container ID, which is like the process ID for a running executable.

```bash
# Build the app (make sure you are still at the top level of your new directory).
docker build -t friendlyhello .

# check the built image
docker image ls

# run the app
docker run -p 4000:80 friendlyhello

# run the app in the background: detached mode
docker run -d -p 4000:80 friendlyhello

# check the instance
docker container ls

# terminate container
docker container stop CONTAINER_ID

```

### Image
A registry is a collection of repositories, and a repository is a collection of images -- sort of like a GitHub repository, except the code is already built. An account on a registry can create many reposities. The docker CLI uses Docker's public registry by default.

```bash
# log in to the Docker public registry
docker login
```

The notation for associating a local image with a repository on a registry is *username/repository:tag*. The tag is optional, but recommended since it is the mechanism that registries use to give Docker images a version. Give the repository and tag meaningful names for the context.

```bash
# docker tag <image> username/repository:tag
docker tag friendlyhello john/get-started:part2

# see the newly tagged image
docker image ls
```

```bash
# upload tagged image
docker push username/repository:tag

# pull to run
docker run -p 4000:80 username/repository:tag
```

### Services
In a distributed application, different pieces of the app are called “services.” Services are really just “containers in production.” A service only runs one image, but it codifies the way that image runs—what ports it should use, how many replicas of the container should run so the service has the capacity it needs, and so on. Scaling a service changes the number of container instances running that piece of software, assigning more computing resources to the service in the process.

*docker-compose.yml*

```bash
version: "3"
services:
	web:
	# replace username/repo:tag with your name and image details
	image: username/repo:tag
	deploy:
		replicas: 5
	resources:
		limits:
			cpus: "0.1"
			memory: 50M
	restart_policy:
condition: on-failure
ports:
- "80:80"
networks:
- webnet
networks:
webnet:
```

This docker-compose.yml file tells Docker to do the following:

* Pull the image we uploaded in step 2 from the registry.
* Run 5 instances of that image as a service called web, limiting each one to use, at most, 10% of the CPU (across all cores), and 50MB of RAM.
* Immediately restart containers if one fails.
* Map port 80 on the host to web’s port 80.
* Instruct web’s containers to share port 80 via a load-balanced network called webnet. (Internally, the containers themselves publish to web’s port 80 at an ephemeral port.)
* Define the webnet network with the default settings (which is a load-balanced overlay network).

Run the load-balanced app:

```bash
docker swarm init
docker stack deploy -c docker-compose.yml getstartedlab

# get service ID for one service
docker service ls
```
A single container running in a service is called task. Tasks are given unique IDs that numerically increment, up to the number of replicas defined in docker-compose.yml.

```bash
# list the tasks
docker container ls -q
```

You can scale the app by changing the replicas value in docker-compose.yml, saving the change, and re-running the docker stack deploy command:

```bash
docker stack deploy -c docker-compose.yml getstartedlab
```

```bash
# take the app down
docker stack rm getstartedlab

# take down the swarm
docker swarm leave --force
```

### Swarm
A swarm is a group of machines that are running Docker and joined into a cluster. After that has happened, you continue to run the Docker commands you’re used to, but now they are executed on a cluster by a swarm manager. The machines in a swarm can be physical or virtual. After joining a swarm, they are referred to as nodes.

Swarm managers are the only machines in a swarm that can execute your commands, or authorize other machines to join the swarm as workers. Workers are just there to provide capacity and do not have the authority to tell any other machine what it can and cannot do.

A swarm is made up of multiple nodes, which can be either physical or virtual machines. The basic concept is simple enough: run

```bash
docker swarm init
```
to enable swarm mode and make your current machine a swarm manager, then run

```bash
docker swarm join
```
on other machines to have them join the swarm as workers.

```bash
# create a couple of VMs using docker-machine
docker-machine create --driver virtualbox myvm1
docker-machine create --driver virtualbox myvm2

# list machines and get IP addresses
docker-machine ls

# initialize the swarm
docker-machine ssh myvm1 "docker swarm init --advertise-addr <myvm1 ip>"

# join the swarm as worker
docker-machine ssh myvm2 "docker swarm join --token <token> <ip>:2377"

# view nodes on the swarm
docker-machine ssh myvm2 "docker node ls"
```

Always run *docker swarm init* and *docker swarm join* with port 2377 (the swarm management port), or no port at all and let it take the default. The machine IP addresss returned by *docker-machine ls* include port 2376, which is the Docker daemon port. Do not use this port.

So far, you’ve been wrapping Docker commands in docker-machine ssh to talk to the VMs. Another option is to run docker-machine env <machine> to get and run a command that configures your current shell to talk to the Docker daemon on the VM. This method works better for the next step because it allows you to use your local docker-compose.yml file to deploy the app “remotely” without having to copy it anywhere.

```bash
# get the command to configure the shell
docker-machine env myvm1

# configure shell
eval $(docker-machine env myvm1)

# verify the active machine
docker-machine ls

# tear down the stack
docker stack rm getstartedlab

# unsetting docker-machine shell variable settings
eval $(docker-machine env -u)

# restarting docker machines
docker-machine start <machine-name>
```

### Stack

A stack is a group of interrelated services that share dependencies, and can be orchestrated and scaled together. A single stack is capable of defining and coordinating the functionality of an entire application (though very complex applications may want to use multiple stacks).

```bash
version: "3"
services:
web:
# replace username/repo:tag with your name and image details
image: username/repo:tag
deploy:
replicas: 5
restart_policy:
condition: on-failure
resources:
limits:
cpus: "0.1"
memory: 50M
ports:
- "80:80"
networks:
- webnet
visualizer:
image: dockersamples/visualizer:stable
ports:
- "8080:8080"
volumes:
- "/var/run/docker.sock:/var/run/docker.sock"
deploy:
placement:
constraints: [node.role == manager]
networks:
- webnet
redis:
image: redis
ports:
- "6379:6379"
volumes:
- "/home/docker/data:/data"
deploy:
placement:
constraints: [node.role == manager]
command: redis-server --appendonly yes
networks:
- webnet
networks:
webnet:
```

Notice two new things here: a volumes key, giving the visualizer access to the host’s socket file for Docker, and a placement key, ensuring that this service only ever runs on a swarm manager – never a worker.

