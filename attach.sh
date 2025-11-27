export FACTORY_BASE=`pwd`
echo "export FACTORY_BASE=`pwd`" >> ~/.bashrc
echo "PATH=\$PATH:$FACTORY_BASE/factory_app_1/lbin" >> ~/.bashrc
./factory_app_1/lbin/fct con
