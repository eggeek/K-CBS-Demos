#!/usr/bin/env python
import os
import numpy as np
import json
import sys
from math import sqrt
from pathplot import readPath

OK: int = 0
STEPSIZE: float = 0.2
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


def genResJson(scenJson: str, resfile: str, sid: int) -> dict:
    print ("scen:", scenJson)
    res = readStatistic(resfile)

    scens = json.load(open(scenJson, 'r'))['scens'][sid]
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


def run(scenfile: str, timelimit: float, stepsize: float):
    dirname = os.path.dirname(scenfile)
    sid = int(os.path.basename(scenfile).split('-')[0])
    resfile = os.path.join(dirname, f"{sid}-kcbs.plan")
    logfile = os.path.join(dirname, f"{sid}-kcbs.log")
    resJsonPath = os.path.join(dirname, f"{sid}-kcbs.json")

    if os.path.exists(logfile):
        print(f"Clean existing logfile [{logfile}]")
        os.remove(logfile)

    cmd = f"./bin/expr {scenfile} {resfile} {timelimit} {stepsize}"
    print (f"run [{cmd}]")
    os.system(cmd)
    if os.path.exists(resfile):
        resDict = genResJson(os.path.join(dirname, "data.json"), resfile, sid)
        print(f"Saving result to {resJsonPath}")
        json.dump(resDict, open(resJsonPath, "w"), indent=2)


def runall(dirname: str):
    torun = []
    for scen in os.listdir(dirname):
        if not os.path.isdir(os.path.join(dirname, scen)):
            continue
        nscen = get_nscen(dirname, scen)
        for i in range(nscen):
            scenfile = os.path.join(dirname, scen, f"{i}-data.scen")
            if not os.path.exists(scenfile):
                print (f"Missing scen file [{scenfile}], skip")
            resfile = os.path.join(dirname, scen, f"{i}-kcbs.plan")
            resJsonPath = os.path.join(dirname, scen, f"{i}-kcbs.json")
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
        ith = int(os.path.basename(scenfile).split('-')[0])
        resfile = os.path.join(os.path.dirname(scenfile), f"{ith}-kcbs.plan")
        print(f"Running scen {scenfile}")
        run(scenfile, TLM, STEPSIZE)

        if not os.path.exists(resfile):
            print(f"No solution: {resfile} not exists")
            continue


def get_nscen(dirname: str, scen: str):
    datafile = os.path.join(dirname, scen, "data.json")
    return len(json.load(open(datafile, 'r'))['scens'])


def genall(dirname: str):
    to_gen_res: list[tuple[str, int]] = []
    no_sol: list[tuple[str, int]] = []
    for scen in os.listdir(dirname):
        if not os.path.isdir(os.path.join(dirname, scen)):
            continue
        nscen = get_nscen(dirname, scen)
        for i in range(nscen):
            resfile = os.path.join(dirname, scen, f"{i}-kcbs.plan")
            jsonfile = os.path.join(dirname, scen, f"{i}-kcbs.json")
            if os.path.exists(resfile) and not os.path.exists(jsonfile):
                to_gen_res.append((scen, i))
            if not os.path.exists(resfile):
                no_sol.append((scen, i))

    print ("To gen json:\n", '\n'.join([f"{i}-{scen}" for scen, i in to_gen_res]))
    for scen, i in to_gen_res:
        scenfile = os.path.join(dirname, scen, "data.json")
        resfile = os.path.join(dirname, scen, f"{i}-kcbs.plan")
        jsonfile = os.path.join(dirname, scen, f"{i}-kcbs.json")
        resDict = genResJson(scenfile, resfile, i)
        json.dump(resDict, open(jsonfile, 'w'), indent=2)

    print ("No sol:\n", '\n'.join([f"{i}-{scen}" for scen, i in no_sol]))
    for scen, i in no_sol:
        envfile = os.path.join(dirname, scen, "data.json")
        jsonfile = os.path.join(dirname, scen, f"{i}-kcbs.json")
        scens = json.load(open(envfile, 'r'))['scens']
        nagent = len(scens[0]['s_x'])
        lb = 0
        for j in range(nagent):
            sx, sy = scens["s_x"][j], scens['s_y'][j]
            gx, gy = scens["d_x"][j], scens['d_y'][j]
            lb += sqrt((sx-gx)**2 + (sy-gy)**2)
        resDict = dict(lb=lb, ub=np.inf, runtime=TLM, paths={})
        json.dump(resDict, open(jsonfile, 'w'), indent=2)


if __name__ == "__main__":
    if sys.argv[1] == "all":
        runall("./mapf-scen")
    elif sys.argv[1] == "gen":
        genall("./mapf-scen")
    else:
        run(sys.argv[1], TLM, STEPSIZE)
