#!/usr/bin/env python
import os
import json
import time
import sys
from math import sqrt
from pathplot import readPath

OK: int = 0


def genResJson(resfile: str, runtime: float) -> dict:
    res = {}
    paths = readPath(resfile)
    lb = 0
    ub = 0
    for path in paths.values():
        for i in range(len(path) - 1):
            cx, cy = path[i]
            nx, ny = path[i + 1]
            ub += sqrt((cx - nx) ** 2 + (cy - ny) ** 2)
        sx, sy = path[0]
        gx, gy = path[-1]
        lb += sqrt((sx - gx) ** 2 + (sy - gy) ** 2)
    res["lb"] = lb
    res["ub"] = ub
    res['runtime'] = runtime
    res["paths"] = paths
    return res


def run(scenfile: str):
    dirname = os.path.dirname(scenfile)
    resfile = os.path.join(dirname, "kcbs.plan")
    tstart = time.perf_counter()
    code = os.system(f"./bin/expr {scenfile} {resfile}")
    tend = time.perf_counter()
    runtime = tend - tstart
    if code == OK and os.path.exists(resfile):
        resDict = genResJson(resfile, runtime)
        resJsonPath = os.path.join(dirname, "kcbs.json")
        print (f"Saving result to {resJsonPath}")
        json.dump(resDict, open(resJsonPath, "w"), indent=2)


def runall(dirname: str):
    from pathplot import draw_sol
    for scen in os.listdir(dirname):
        scenfile = os.path.join(dirname, scen, "data.scen")
        if os.path.exists(scenfile):
            print (f"Running scen {scenfile}")
            run(scenfile)

            resfile = os.path.join(dirname, scen, "kcbs.plan")
            if not os.path.exists(resfile):
                continue
            print ("Generating animation ...")
            ani = draw_sol(scenfile, resfile, interval = 20)
            anifile = os.path.join(dirname, scen, "kcbs.mp4")
            ani.save(anifile)
            print ("Done.")



if __name__ == "__main__":
    if sys.argv[1] == "all":
        runall("./cbs-scen")
    else:
        run(sys.argv[1])
