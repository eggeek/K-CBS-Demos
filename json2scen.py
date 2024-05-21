#!/usr/bin/env python
import json
from sys import argv
from pathplot import Env, Poly, Agent

def run(file: str, out = None):
    dat = json.load(open(file, "r"))
    edat = dat['env']
    minx, maxx = edat['minx'], edat['maxx']
    miny, maxy = edat['miny'], edat['maxy']

    obsts: list[Poly] = edat['obsts']
    agents = {}
    scen = dat['scens'][0]
    for i in range(len(scen['s_x'])):
        xL, yL = edat['d_thres'], edat['d_thres']
        sx = scen['s_x'][i]
        sy = scen['s_y'][i]
        gx = scen['d_x'][i]
        gy = scen['d_y'][i]
        agents[i] = Agent(f"robot {i}", xL, yL, sx, sy, gx, gy)

    env: Env = Env(minx, maxx, miny, maxy, obsts, agents)
    print (env, file=out)

def runall(dirname: str):
    from os import listdir
    import os.path as op
    for scen in listdir(dirname):
        jsonPath = op.join(dirname, scen, "data.json")
        scenPath = op.join(dirname, scen, "data.scen")
        if op.exists(jsonPath):
            print (f"Convert {jsonPath} to {scenPath}")
            run(jsonPath, out=open(scenPath, "w"))
    

if __name__ == "__main__":
    """
    ./json2scen data.json > file.scen
    """
    if argv[1] == "all":
        runall("./cbs-scen")
    else:
        run(argv[1])
