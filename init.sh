mkdir config

while (( $# > 0))
do
  case $1 in
    '--cloud')
      echo "CLOUD_IP=$2" >> ./config/.env
      shift
      ;;
  esac
  shift
done

export FACTORY_BASE=`pwd`
echo "export FACTORY_BASE=`pwd`" >> ~/.bashrc
echo "PATH=\$PATH:$FACTORY_BASE/factory_app_1/lbin" >> ~/.bashrc
git clone https://github.com/momoiorg-repository/at_factory.git
cd at_factory
git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
./init.sh
./finstall.sh https://github.com/momoiorg-repository/melon_ros2.git
./finstall.sh https://github.com/momoiorg-repository/factory_world1.git world
./run_isaac_sim_docker.sh
docker cp $FACTORY_BASE/factory_app_1/fenv/run_world isaac-sim-ws:/usr/local/bin/run_world
./connect_to_container.sh
