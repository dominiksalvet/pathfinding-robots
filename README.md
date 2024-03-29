# pathfinding-robots

[![GitPack](https://img.shields.io/badge/-GitPack-571997)](https://github.com/topics/gitpack)
[![standard-readme compliant](https://img.shields.io/badge/readme_style-standard-brightgreen.svg)](https://github.com/RichardLitt/standard-readme)
[![Hits](https://hits.seeyoufarm.com/api/count/incr/badge.svg?url=https%3A%2F%2Fgithub.com%2Fdominiksalvet%2Fpathfinding-robots&count_bg=%2379C83D&title_bg=%23555555&icon=&icon_color=%23E7E7E7&title=hits&edge_flat=false)](https://hits.seeyoufarm.com)

> Simple terminal game with pathfinding

This project was created during the first semester of my bachelor's studies within a programming course. It uses a very simple pathfinding implementation and demonstrates it on a robot-chasing game with intuitive terminal controls. It is provided in its original form with some small modifications.

First, it generates a two dimensional world of free fields and barriers. Free fields always create a connected graph so that robots can move to any free field. When the world is generated, you can take a look at it. After, you can start the world simulation and robots will start to move. When the simulation ends, you can browse historical path of each robot.

## Table of Contents

* [Install](#install)
  * [Dependencies](#dependencies)
* [Usage](#usage)
  * [Example](#example)
* [Contributing](#contributing)
* [License](#license)

## Install

This project supports [GitPack](https://github.com/dominiksalvet/gitpack). Local installation/update:

```sh
gitpack install github.com/dominiksalvet/pathfinding-robots
```

### Dependencies

* Python 3

## Usage

To run the game, execute the `pathfinding-robots.py` command. Then type a number based on the current options and press the enter key. Feel free to take a look at the example below and get inspired.

### Example

<p align="center">
    <img src="img/example.gif" alt="pathfinding-robots example">
</p>

## Contributing

Do you want to contribute? Do you have any questions? Then the [*CONTRIBUTING.md*](CONTRIBUTING.md) file is here for you.

## License

This project is licensed under the [MIT License](LICENSE).
