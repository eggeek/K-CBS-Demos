#!/usr/bin/env python
import json
from sys import argv
from pathplot import Env, Poly, Agent


def run(file: str, scale: float = 1, out=None):
    dat = json.load(open(file, "r"))
    edat = dat["env"]
    minx, maxx = edat["minx"], edat["maxx"]
    miny, maxy = edat["miny"], edat["maxy"]
    maxv = dat['v_max']

    obsts: list[Poly] = [
        [(x * scale, y * scale) for x, y in obs] for obs in edat["obsts"]
    ]

    agents = {}
    scen = dat["scens"][0]
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
        jsonPath = op.join(dirname, scen, "data.json")
        scenPath = op.join(dirname, scen, "data.scen")
        if op.exists(jsonPath):
            print(f"Convert {jsonPath} to {scenPath}")
            run(jsonPath, scale=1, out=open(scenPath, "w"))


if __name__ == "__main__":
    """
    ./json2scen data.json > file.scen
    """
    if argv[1] == "all":
        runall("./cbs-scen")
    else:
        run(argv[1])
