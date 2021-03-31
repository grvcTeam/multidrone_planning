# Autonomous cinematograpy with multiple drones #

This repository contains a software architecture with algorithms for cooperative planning and mission execution in autonomous cinematography with multiple drones. This work was developed within the framework of the [_MultiDrone_](https://multidrone.eu/) project.

## How to install ##

Download the repository:

```
git clone https://github.com/grvcTeam/multidrone_planning.git
```
And compile the code with:

```
catkin build
```

## Dependencies ##

* [grvc-ual](https://github.com/grvcTeam/grvc-ual)
* [geodesy (from geographic_info)](http://wiki.ros.org/geodesy): `sudo apt-get install ros-$(rosversion -d)-geographic-info`
* [libusbp](https://github.com/pololu/libusbp)
* [cmake_modules](http://wiki.ros.org/cmake_modules): `sudo apt-get install ros-$(rosversion -d)-cmake-modules`
* [gazebo_plugins](http://wiki.ros.org/gazebo_plugins): `sudo apt-get install ros-$(rosversion -d)-gazebo-plugins`


## Testing ##

You can find detailed instructions about the system and how to use it in the [Wiki](https://github.com/grvcTeam/multidrone_planning/wiki).

## References

Apart from the documentation in the Wiki, you can find a detailed description of the system and the methods for mission planning and execution in the following references. Please consider to cite them if you find this repository helpful for your research.

For a general overview of the architecture for cinematography with multiple drones:

```
@article{alcantara_ACCESS20, 
    author={A. {Alcántara} and J. {Capitán} and A. {Torres-González} and R. {Cunha} and A. {Ollero}},
    journal={IEEE Access},
    title={Autonomous Execution of Cinematographic Shots with Multiple Drones},
    year={2020},
    doi={10.1109/ACCESS.2020.3036239}
}
```

For details about planning methods in cinematography missions with multiple drones:

```
@inproceedings{caraballo_iros20, 
    address = {Las Vegas, USA}, 
    author = {Caraballo, Luis Evaristo and Montes-Romero, Angel and Diaz-Ba{\~{n}}ez, Jose Miguel and Capitan, Jesus and Torres-Gonzalez, Arturo and Ollero, Anibal}, 
    booktitle = {International Conference on Intelligent Robots and Systems (IROS)}, 
    title = {{Autonomous Planning for Multiple Aerial Cinematographers}}, 
    year = {2020},
    doi = {10.1109/IROS45743.2020.9341622}
}
```

For director tools to design cinematography missions with multiple drones and details about the XML language for mission description:

```
@Article{montes_appsci20,
AUTHOR = {Montes-Romero, Ángel and Torres-González, Arturo and Capitán, Jesús and Montagnuolo, Maurizio and Metta, Sabino and Negro, Fulvio and Messina, Alberto and Ollero, Aníbal},
TITLE = {Director Tools for Autonomous Media Production with a Team of Drones},
JOURNAL = {Applied Sciences},
VOLUME = {10},
YEAR = {2020},
NUMBER = {4},
ARTICLE-NUMBER = {1494},
ISSN = {2076-3417},
DOI = {10.3390/app10041494}
}
```
