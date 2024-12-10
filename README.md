# Install Dependencies

1) Install [`config_utilities`](https://github.com/MIT-SPARK/config_utilities)
```
git clone git@github.com:MIT-SPARK/config_utilities.git
cd config_utilities/config_utilities
mkdir build
cd build
cmake ..
make -j
sudo make install
```

2) Install [`cnpy`](https://github.com/rogersce/cnpy)

```
git clone git@github.com:rogersce/cnpy.git
cd cnpy
mkdir build
cd build
cmake ..
make
sudo make install
```

3) Install `yaml-cpp`
```
sudo apt install libyaml-cpp-dev
```

# Install Ruby Lidar driver
```
cd <path to where you have cloned this repo>
mkdir build
cd build
cmake ..
make
```

# Running the node
```
./lakibeam1_scan_node
```
