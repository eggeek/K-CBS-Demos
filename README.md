# K-CBS-Demos

1. `git clone --recursive git@github.com:eggeek/K-CBS-Demos.git`;
2. `cd K-CBS-Demos/ompl`;
3. Follow the instruction in `K-CBS-Demos/ompl/Readme.md` to compile `ompl`;
4. Back to the top layer and run `make fast` to compile `K-CBS-Demos`;
5. Run experiments
 - Create soft links for generated scenarios:
     - `ln -s <large-mapf-scen> large-mapf-scen`
     - `ln -s <special-mapf-scen> special-mapf-scen`
 - `./json2scen.py`: generate `.scen` file based `.json`
 - `./run_expr.py`: run all scenarios in `large-mapf-scen` and `special-mapf-scen`
6. Visualize result: `./pathplot.py <scenfile> <resfile>`

- Turn on/off high order dynamics:

```cpp

// set the state propagation routine

// without high order dynamics
// si->setStatePropagator(myDemoPropagateFunction);

// using ODE solver for high order dynamics
auto odeSolver(
    std::make_shared<oc::ODEBasicSolver<>>(si, &SecondOrderCarODE));
si->setStatePropagator(oc::ODESolver::getStatePropagator(
    odeSolver, &SecondOrderCarODEPostIntegration));
```

- Turn on/off `systemmerger`: author's original `systemmerger` causes a dangerous warning, you may want to turn-off it.

```cpp

// set the system merger

// proposed system merger in author's experiments
// auto merger = std::make_shared<homogeneous2ndOrderCarSystemMerger>(
//     ma_si, ma_pdef, robot_map, obsts, 10, starts, goals);

// empty system merger
auto merger = std::make_shared<myDemoSystemMerger>(ma_si, ma_pdef);
ma_si->setSystemMerger(merger);

```

## References
1. J. Kottinger, S. Almagor and M. Lahijanian, "Conflict-Based Search for Multi-Robot Motion Planning with Kinodynamic Constraints," 2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Kyoto, Japan, 2022, pp. 13494-13499, doi: 10.1109/IROS47612.2022.9982018.
2. Theurkauf, Anne, Kottinger, Justin, and Lahijanian, Morteza. "Chance-Constrained Multi-Robot Motion Planning under Gaussian Uncertainties." arXiv preprint arXiv:2303.11476 (2023).
