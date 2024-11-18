#!/usr/bin/env python
import json
from sys import argv
from pathplot import Env, Poly, Agent


def run(file: str, scale: float = 1, sid: int=0, out=None):
    dat = json.load(open(file, "r"))
    edat = dat["env"]
    minx, maxx = edat["minx"], edat["maxx"]
    miny, maxy = edat["miny"], edat["maxy"]
    maxv = dat['v_max']

    obsts: list[Poly] = [
        [(x * scale, y * scale) for x, y in obs] for obs in edat["obsts"]
    ]

    agents = {}
    scen = dat["scens"][sid]
    for i in range(len(scen["s_x"])):
        xL, yL = edat["d_thres"] * scale, edat["d_thres"] * scale
        sx = scen["s_x"][i] * scale
        sy = scen["s_y"][i] * scale
        gx = scen["d_x"][i] * scale
        gy = scen["d_y"][i] * scale
        agents[i] = Agent(f"robot {i}", xL, yL, sx, sy, gx, gy)

    env: Env = Env(
        minx * scale, maxx * scale, miny * scale, maxy * scale, 
        maxv * scale,
        obsts, agents
    )
    print(env, file=out)


def runall(dirname: str):
    from os import listdir
    import os.path as op

    for scen in listdir(dirname):
        if not op.isdir(op.join(dirname, scen)):
            continue
        jsonPath = op.join(dirname, scen, "data.json")
        nscen = len(json.load(open(jsonPath, 'r'))['scens'])
        for i in range(nscen):
            scenPath = op.join(dirname, scen, f"{i}-data.scen")
            if op.exists(jsonPath):
                print(f"Convert {jsonPath} to {scenPath}")
                run(jsonPath, scale=1, out=open(scenPath, "w"), sid=i)


if __name__ == "__main__":
    """
    ./json2scen data.json > file.scen
    """
    if argv[1] == "all":
        runall("./large-mapf-scen")
        runall("./special-mapf-scen")
    else:
        run(argv[1])
