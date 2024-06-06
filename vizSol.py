#!/usr/bin/env python
import os
import sys
import pathplot as pp
from enum import Enum

class Result(Enum):
    ANI = 0 # animation
    FIG = 1 # figure
    ENV = 2 # environment


def run(scendir: str, fname: str, rtype: Result = Result.ANI):
    resfile = os.path.join(scendir, fname)
    dirname = os.path.dirname(resfile)
    envfile = os.path.join(dirname, "data.scen")
    anifile = os.path.join(dirname, fname.removesuffix(".json") + ".gif")

    if not os.path.exists(resfile):
        print (f"File [{resfile}] not exists, skip ...")
        return

    if len(pp.readJsonPath(resfile)) == 0:
        print (f"Path not exists in file [{resfile}], skip ...")
        return

    if rtype is Result.FIG:
        print (f"Generate fig for [{resfile}]")
        figfile = os.path.join(dirname, fname.removesuffix(".json") + ".jpg")
        fig, _ = pp.draw_sol_static(envfile, resfile)
        fig.savefig(figfile)
        pp.plt.close()
        print (f"Saved to [{figfile}] ...")
        return

    if rtype is Result.ENV:
        print (f"Generate environment fig for [{envfile}]")
        figfile = os.path.join(dirname, "env.jpg")
        env = pp.readEnv(envfile)
        subp = pp.plt.subplots(figsize=(15, 15))
        fig: pp.Figure = subp[0]
        ax: pp.Axes = subp[1]
        ax.set_xlim(env.xbound())
        ax.set_ylim(env.ybound())
        fig.tight_layout()
        pp._draw_static_obstacles2D(ax, env.obsts, alpha=0.8)
        fig.savefig(figfile)
        pp.plt.close()
        print (f"Saved to [{figfile}] ...")

    if rtype is Result.ANI:
        print (f"Generate gif for [{resfile}]")
        exists = os.path.exists(anifile)
        if exists:
            return
        ani = pp.draw_sol(envfile, resfile, 20, 500)
        print (f"Generating animation for [{resfile}] ...")
        print (f"Saving to [{anifile}] ...")
        ani.save(anifile)
        pp.plt.close()
        print ("Done.")

def runall(dirname: str, rtype: Result = Result.ANI):
    for scen in os.listdir(dirname):
        run(os.path.join(dirname, scen), "kcbs.json", rtype)
        run(os.path.join(dirname, scen), "micp-UB.json", rtype)


if __name__ == "__main__":
    rtype: Result = Result.ANI
    
    if len(sys.argv) > 2:
        if sys.argv[2] == "fig":
            rtype = Result.FIG
        elif sys.argv[2] == 'env':
            rtype = Result.ENV
        elif sys.argv[2] == 'ani':
            rtype = Result.ANI
    if sys.argv[1] == "all":
        runall("./cbs-scen", rtype)
    else:
        run(sys.argv[1], "kcbs.json", rtype)
