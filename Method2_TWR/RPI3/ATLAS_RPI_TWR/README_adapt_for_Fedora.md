cpp-atlas
================

This is the C++11 code for the ATLAS Localization Server.

[![DOI](https://zenodo.org/badge/20743/tudo-cni-atlas/cpp-atlas.svg)](https://zenodo.org/badge/latestdoi/20743/tudo-cni-atlas/cpp-atlas)

## Dependencies

* GCC 4.9 and later
* CMake 2.6 and later
* LibYAML (C++)
* Boost (Currently, due to LibYAML dependency)
* Armadillo (C++)


## Installing

On Debian or similar:

```Shell
sudo apt-get install git cmake libyaml-cpp-dev libboost-dev libarmadillo-dev
```

On Fedora:

```Shell
so    (passwort: fhtw for Fedora user: fhtw)   --> log in as root user
yum install git					--> install newest git
yum install gcc					--> install newest GCC
yum install cmake 				--> install newest CMake
yum install yaml-cpp-devel		--> install newest LibYAML
yum install boost-devel			--> install newest Boost
yum install armadillo-devel		--> install newest Armadillo
```

## Building

Building should be straight forward, as CMake is used.

```Shell
cmake .
make
```

## Configuration

The `config.yaml` is used to configure the server. 
Since all anchors are connected to the server via USB, a unique symlink is created for each anchor to differentiate between them. 
This is done using the `99-usb-uwb.rules` udev rules file.


## License

See the [LICENSE](LICENSE.md) file for license rights and limitations (MIT).
