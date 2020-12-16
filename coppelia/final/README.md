# CSE 360 Final Project: A Mobile Robot Capable of Launching Small Objects

## Usage:
All the scenes used for developing and testing are located in [`final/scenes/`](https://github.com/walker-finlay/CSE360-MobileRobotics/tree/master/coppelia/final/scenes).
- The full demonstration was tested on [`final/scenes/more_obstacles_launcherv2.3.ttt`](https://github.com/walker-finlay/CSE360-MobileRobotics/blob/master/coppelia/final/scenes/more_obstacles_launcherv2.3.ttt)
  - To run the full test, simply run `python main.py`
- The launcher alone was tested on [`final/scenes/launcher_v2.3.ttt`](https://github.com/walker-finlay/CSE360-MobileRobotics/blob/master/coppelia/final/scenes/launcher_v2.3.ttt)
  - To run the launcher test, run `python test.py Test.test_fire`
    - Note: the unit tests are not perfect, so do not run the entire file, as they may clash with one another and potentially each open their own connection to the simluation. Instead run tests individually as above.

Additionally, the logs from the laser scanner are saved in `more_obstacles.csv` and the program can be run without coppeliasim by running `final/testing/messin.py` and uncommenting the plotting function calls to see the results.

## TODO:
- Splines
- Check airspace
- Fix pitch control
