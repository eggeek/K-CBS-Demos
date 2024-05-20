# K-CBS-Demos

1. `git clone --recurcive git@github.com:eggeek/K-CBS-Demos.git`;
2. `cd K-CBS-Demos/ompl`;
3. Follow the instruction in `K-CBS-Demos/ompl/Readme.md` to compile `ompl`;
4. Back to the top layer and run `make fast` to compile `K-CBS-Demos`;
5. Run experiments
 - `./bin/expr`: original experiment script
 - `./bin/simple_expr`: simplified experiment script that ignores feasibility checking in transition
6. Plot result: `python pathplot.py`

## References
1. J. Kottinger, S. Almagor and M. Lahijanian, "Conflict-Based Search for Multi-Robot Motion Planning with Kinodynamic Constraints," 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Kyoto, Japan, 2022, pp. 13494-13499, doi: 10.1109/IROS47612.2022.9982018.
2. Theurkauf, Anne, Kottinger, Justin, and Lahijanian, Morteza. "Chance-Constrained Multi-Robot Motion Planning under Gaussian Uncertainties." arXiv preprint arXiv:2303.11476 (2023).
