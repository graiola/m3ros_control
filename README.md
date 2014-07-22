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

>Authors: Gennaro Raiola, Antoine Hoarau
