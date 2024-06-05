#!/usr/bin/env python
import os
import numpy as np
import json
import sys
from math import sqrt
from pathplot import readPath

OK: int = 0
TLM: float = 500.0


def readStatistic(resfile: str):
    res = {}
    with open(resfile, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if line.startswith('='):
                k1, v1 = lines[i+1].split()
                res[k1] = float(v1)

                k2, v2 = lines[i+2].split()
                res[k2] = float(v2)
                break
    return res


def genResJson(scenJson: str, resfile: str) -> dict:
    print ("scen:", scenJson)
    res = readStatistic(resfile)

    scens = json.load(open(scenJson, 'r'))['scens'][0]
    lb = 0
    for i in range(len(scens['s_x'])):
        sx, sy = scens['s_x'][i], scens['s_y'][i]
        gx, gy = scens['d_x'][i], scens['d_y'][i]
        lb += sqrt((sx - gx) ** 2 + (sy - gy) ** 2)

    paths = readPath(resfile)
    ub = 0
    for path in paths.values():
        for i in range(len(path) - 1):
            cx, cy = path[i]
            nx, ny = path[i + 1]
            ub += sqrt((cx - nx) ** 2 + (cy - ny) ** 2)
        sx, sy = path[0]
        gx, gy = path[-1]
    res["lb"] = lb
    res["ub"] = ub
    res["paths"] = paths
    return res


def run(scenfile: str, timelimit: float):
    dirname = os.path.dirname(scenfile)
    resfile = os.path.join(dirname, "kcbs.plan")
    resJsonPath = os.path.join(dirname, "kcbs.json")

    os.system(f"./bin/expr {scenfile} {resfile} {timelimit}")
    if os.path.exists(resfile):
        resDict = genResJson(scenfile.removesuffix(".scen")+".json", resfile)
        print(f"Saving result to {resJsonPath}")
        json.dump(resDict, open(resJsonPath, "w"), indent=2)


def runall(dirname: str):
    torun = []
    for scen in os.listdir(dirname):
        scenfile = os.path.join(dirname, scen, "data.json")
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

    print ("To run:\n", '\n'.join([f"  {i}: [{task}]" for i, task in enumerate(torun)]))

    for scenfile in torun:
        resfile = os.path.join(os.path.dirname(scenfile), "kcbs.plan")
        print(f"Running scen {scenfile}")
        run(scenfile, TLM)

        if not os.path.exists(resfile):
            print(f"No solution: {resfile} not exists")
            continue


def genall(dirname: str):
    to_gen_res = []
    no_sol = []
    for scen in os.listdir(dirname):
        resfile = os.path.join(dirname, scen, "kcbs.plan")
        jsonfile = os.path.join(dirname, scen, "kcbs.json")
        if os.path.exists(resfile) and not os.path.exists(jsonfile):
            to_gen_res.append(scen)
        if not os.path.exists(resfile):
            no_sol.append(scen)

    print ("To gen json:\n", '\n'.join(to_gen_res))
    for scen in to_gen_res:
        scenfile = os.path.join(dirname, scen, "data.json")
        resfile = os.path.join(dirname, scen, "kcbs.plan")
        jsonfile = os.path.join(dirname, scen, "kcbs.json")
        resDict = genResJson(scenfile, resfile)
        json.dump(resDict, open(jsonfile, 'w'), indent=2)

    print ("No sol:\n", '\n'.join(no_sol))
    for scen in no_sol:
        envfile = os.path.join(dirname, scen, "data.json")
        jsonfile = os.path.join(dirname, scen, "kcbs.json")
        scens = json.load(open(envfile, 'r'))['scens'][0]
        nagent = len(scens['s_x'])
        lb = 0
        for i in range(nagent):
            sx, sy = scens["s_x"][i], scens['s_y'][i]
            gx, gy = scens["d_x"][i], scens['d_y'][i]
            lb += sqrt((sx-gx)**2 + (sy-gy)**2)
        resDict = dict(lb=lb, ub=np.inf, runtime=TLM, paths={})
        json.dump(resDict, open(jsonfile, 'w'), indent=2)




if __name__ == "__main__":
    if sys.argv[1] == "all":
        runall("./cbs-scen")
    elif sys.argv[1] == "gen":
        genall("./cbs-scen")
    else:
        run(sys.argv[1], TLM)
