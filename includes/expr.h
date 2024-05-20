#pragma once
#include "Obstacle.h"

#include <ostream>
#include <fstream>
#include <map>
#include <set>


using vmap = std::map<std::string, std::pair<double, double>>;
using oset = std::set<Obstacle *>;


struct Obs {
  double xleft, yleft, xL, yL;
  void print(std::ostream &out) {
    out << xleft << " " << yleft << " " << xL << " " << yL << std::endl;
  }
};

struct Env {
  double maxx, maxy, // workspace
      xL, yL;        // robot size
};

struct Scen {
  vmap starts, goals;
  oset obsts;
  Env env;

  Scen(vmap& s, vmap& g, oset& o, Env& e): starts(s), goals(g), obsts(o), env(e) {};
  Scen(std::string& path) {
    std::ifstream fin(path);
    fin >> env.maxx >> env.maxy >> env.xL >> env.yL;
    int obsNum, botNum;
    fin >> obsNum;
    for (int _=0; _<obsNum; _++) {
      double x, y, xL, yL;
      fin >> x >> y >> xL >> yL;
      obsts.insert(new RectangularObstacle(x, y, xL, yL));
    }

    fin >> botNum;
    for (int _=0; _<botNum; _++) {
      std::string name;
      double x, y;
      fin >> name >> x >> y;
      starts[name] = {x, y};
    }

    for (int _=0; _<botNum; _++) {
      std::string name;
      double x, y;
      fin >> name >> x >> y;
      goals[name] = {x, y};
    }
  }
};
