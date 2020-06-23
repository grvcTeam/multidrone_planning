# multidrone_planning #

Algorithms for cooperative planning and mission execution for autonomous media production with multiple drones.

## How do I get set up? ##

```
git clone https://github.com/grvcTeam/multidrone_planning.git
```

## Dependencies ##

* [grvc-ual](https://github.com/grvcTeam/grvc-ual)
* [geodesy (from geographic_info)](http://wiki.ros.org/geodesy): `sudo apt-get install ros-ros-$(rosversion -d)-geographic-info`
* [libusbp](https://github.com/pololu/libusbp)
* [cmake_modules](http://wiki.ros.org/cmake_modules): `sudo apt-get install ros-$(rosversion -d)-cmake-modules`
* [gazebo_plugins](http://wiki.ros.org/gazebo_plugins): `sudo apt-get install ros-$(rosversion -d)-gazebo-plugins`


You can find detailed instructions about the system and how to use it in the [Wiki](https://github.com/grvcTeam/multidrone_planning/wiki)

## Citation
If you find multidrone_planning useful in your research, please consider citing the following works. For a general architecture for cinematography with multiple drones:

```
@article{alcantara_arxiv20,
    title={Autonomous Execution of Cinematographic Shots with Multiple Drones},
    author={Alfonso Alcántara and Jesús Capitán and Arturo Torres-González and Rita Cunha and Aníbal Ollero},
    year={2020},
    eprint={2006.12163},
    archivePrefix={arXiv},
    primaryClass={cs.RO}
}
```

For planning methods in cinematography missions with multiple drones:

```
@article{caraballo_arxiv20,
    title={Autonomous Planning for Multiple Aerial Cinematographers},
    author={Luis-Evaristo Caraballo and Angel Montes-Romero and Jose-Miguel Diaz-Bañez and Jesss Capitan and Arturo Torres-Gonzalez and Anibal Ollero},
    year={2020},
    eprint={2005.07237},
    archivePrefix={arXiv},
    journal = {ArXiv e-prints},
    primaryClass={cs.RO}
}
```

For director tools to design and encode cinematography missions with multiple drones:

```
@Article{montes_arxiv20,
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
