#!/usr/bin/env python
import os
import json
import time
import sys
from math import sqrt
from pathplot import readPath

OK: int = 0
TLM: float = 500.0


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
    res["runtime"] = runtime
    res["paths"] = paths
    return res


def run(scenfile: str, timelimit: float):
    dirname = os.path.dirname(scenfile)
    resfile = os.path.join(dirname, "kcbs.plan")
    runtime = timelimit
    resJsonPath = os.path.join(dirname, "kcbs.json")

    tstart = time.perf_counter()
    os.system(f"./bin/expr {scenfile} {resfile} {timelimit}")
    tend = time.perf_counter()
    runtime = tend - tstart

    if os.path.exists(resfile):
        resDict = genResJson(resfile, runtime)
        print(f"Saving result to {resJsonPath}")
        json.dump(resDict, open(resJsonPath, "w"), indent=2)


def runall(dirname: str):
    torun = []
    for scen in os.listdir(dirname):
        scenfile = os.path.join(dirname, scen, "data.scen")
        if not os.path.exists(scenfile):
            print (f"Missing scen file [{scenfile}], skip")
        resfile = os.path.join(dirname, scen, "kcbs.plan")
        resJsonPath = os.path.join(dirname, scen, "kcbs.json")
        print(f"Processing scen {scenfile}")
        
        # # run expr if no resfile or the resfile is not the right version
        exists = os.path.exists(resJsonPath)
        runtime = -1 if not exists else json.load(open(resJsonPath, 'r')).get('runtime', -1)
        if not exists or runtime < TLM:
            torun.append(scenfile)
        else:
            print (f"Existing result [{resJsonPath}] seems the right version, skip.")
            continue

    print ("To run: ", '\n'.join([f"  {i}: [{task}]" for i, task in enumerate(torun)]))

    for scenfile in torun:
        print(f"Running scen {scenfile}")
        run(scenfile, TLM)

        if not os.path.exists(resfile):
            print(f"No solution: {scenfile}")
            continue


if __name__ == "__main__":
    if sys.argv[1] == "all":
        runall("./cbs-scen")
    else:
        run(sys.argv[1], TLM)
