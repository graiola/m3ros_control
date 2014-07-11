### Meka Ros Control


This package implements the ros_control component to control the Meka M3 Robot at Ensta ParisTech.


```bash
git clone https://github.com/ahoarau/m3ros_control.git
cd m3ros_control
mkdir build;cd build
cmake ..
make -j$(nproc)
```
Then make sure roscore is started and overlay the config file for the m3 server to find it:
```bash
source m3ros_control/setup.bash
```

Note: currently robot_config/m3_config.yml contains an hardcoded path (the library to be dlopened), make sure it's correct. A temporary fix would be to install the libs.

>Authors: Gennaro Raiola, Antoine Hoarau
