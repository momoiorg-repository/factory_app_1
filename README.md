# Application Sample for @factory

@factory is a research and development platform for physical AI that allows for the simulation of small-scale factories and the various machine tools and robots operating there on NVIDIA Isaac Sim. By connecting robot control programs (hereinafter referred to as applications) using physical AI to this via the ROS protocol, users can develop and test them in a virtual factory. Factory configurations and robot definitions are built using a plug-in structure, allowing them to be freely replacing different implementations.

This repository shows an example of an @factory application that runs using specific factory and robot definitions. At runtime, it consists of two docker containers: the @factory itself and an application with a Behavior Tree structure. The docker container (isaac-sim-ws, main docker container) that runs the @factory itself can be easily started with single command.

The other docker, which runs applications (application docker), is designed to be run by VSCode to make development easier.

## ◯ Installation
The script you run will differ depending on whether you're installing docker for the first time or using an existing docker installation.

### * If you're installing the main docker for the first time on the server(run only once)

Create a working directory before proceeding.

```
mkdir factory
cd factory
git clone https://github.com/momoiorg-repository/factory_app_1.git
./factory_app_1/init.sh
```

This will build and start the main docker container, and a prompt from shell in the docker will appear.

### * If the main docker container is already running on the server (run only once)

This assumes that the main docker container is already running.

```
git clone https://github.com/momoiorg-repository/factory_app_1.git
./factory_app_1/attach.sh
```

This completes the connection to the main docker and a prompt from shell in the docker appears.

Installation is now complete.

When installation is complete, the PATH variable is rewritten automatically, allowing you to use the “fct” command to operate the main docker. After installation, open another shell session on the server and enter

```
fct
```
to display instructions. You can run, start, stop, connect, and perform other operations on the main docker.

## ◯ Launching Isaac Sim in the main docker container

With the main Docker container shell prompt displayed, execute the command to launch Isaac Sim, which runs @factory:

```
run_world
```

This will launch Isaac Sim and enable WebRTC connections. Specify the server's IP address to connect.

## ◯ Building the application docker

Enter the following commands from server shell.

```
act build
act run
```

This will launch the application docker. 

The "act" command has the ability to control the application docker in a similar way that the "fct" command controls the main docker. 

From now on, attach it from VSCode and use it. Assume the current directory is /root.  

From the VSCode terminal, execute the following command once:

```
cd pytwb_ws
pytwb
> create cm1
> Y
```

The application to be executed is launched by pytwb. pytwb is run by executing

```
pytwb_ws/src/cm1/cm1/app_main.py
```

from VSCode. If necessary, you can run the entire application program in debug mode by launching pytwb in debug mode.

## ◯ Customization using environment variables

You can create a .factory_env file in the working directory where you first cloned factory_app_1 to store the initial values ​​of environment variables. The relevant environment variables are:

ROS_DOMAIN_ID: The ROS_DOMAIN_ID for ROS communication  
FACTORY_MAIN_CONTAINER_NAME: The name of the main container  
FACTORY_APP_CONTAINER_NAME: The name of the application container  
