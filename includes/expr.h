#pragma once
#include "Obstacle.h"

#include <cstdio>
#include <fstream>
#include <istream>
#include <map>
#include <ostream>
#include <set>

using vmap = std::map<std::string, std::pair<double, double>>;
using oset = std::set<Obstacle *>;

struct Obs {
  double xleft, yleft, xL, yL;
  void print(std::ostream &out) {
    out << xleft << " " << yleft << " " << xL << " " << yL << std::endl;
  }
  void read(std::istream &in) { in >> xleft >> yleft >> xL >> yL; }
};

struct Agent {
  std::string name;
  double xL, yL;         // rect shape
  double sx, sy, gx, gy; // start / goal loc

  void read(std::istream &in) {
    while (name.empty()) {
      std::getline(in, name);
    }
    in >> xL >> yL >> sx >> sy >> gx >> gy;
  }

  void print(std::ostream &out) {
    out << name << std::endl;
    out << xL << " " << yL << " " << sx << " " 
        << sy << " " << gx << " " << gy << std::endl;
  }
};

struct Env {
  // workspace
  double minx = 0, maxx = 0, miny = 0, maxy = 0, maxv = 1;
  std::vector<Obs> obsts;

  void read(std::istream &in) {
    in >> minx >> maxx >> miny >> maxy >> maxv;
    int num;
    in >> num;
    obsts.resize(num);
    for (int i = 0; i < num; i++) {
      obsts[i].read(in);
    }
  }
  void print(std::ostream &out) {
    out << minx << " " << maxx << " " << miny << " " << maxy << " " << maxv << std::endl;
    out << obsts.size() << std::endl;
    for (auto ob : obsts)
      ob.print(out);
  }
};

struct Scen {
  Env env;
  std::vector<Agent> bots;

  void init(const vmap &starts, const vmap &goals, double xL, double yL,
            const Env &e) {
    env = Env(e);
    bots.resize(starts.size());
    auto cur = bots.begin();
    for (auto it : starts) {
      cur->name = it.first;
      cur->sx = starts.at(cur->name).first;
      cur->sy = starts.at(cur->name).second;
      cur->gx = goals.at(cur->name).first;
      cur->gy = goals.at(cur->name).second;
      cur->xL = xL;
      cur->yL = yL;
      cur++;
    }
  }

  void save(std::string fname) {
    std::ofstream fout(fname);
    env.print(fout);
    fout << bots.size() << std::endl;
    for (auto bot : bots)
      bot.print(fout);
  }
  void read(std::string fname) {
    std::ifstream fin(fname);
    env.read(fin);
    int botNum;
    fin >> botNum;
    bots.resize(botNum);
    for (int i = 0; i < botNum; i++) {
      bots[i].read(fin);
    }
  }

  oset obsts() const {
    oset res;
    for (auto obs : env.obsts) {
      res.insert(new RectangularObstacle(obs.xleft, obs.yleft, obs.xL, obs.yL));
    }
    return res;
  }

  vmap starts() const {
    vmap res;
    for (auto bot : bots) {
      res[bot.name] = {bot.sx, bot.sy};
    }
    return res;
  }

  vmap goals() const {
    vmap res;
    for (auto bot : bots) {
      res[bot.name] = {bot.gx, bot.gy};
    }
    return res;
  }
};
