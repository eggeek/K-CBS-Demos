#!/usr/bin/env python
import os
import sys
import pathplot as pp

def run(scendir: str, fname: str):
    resfile = os.path.join(scendir, fname)
    if not os.path.exists(resfile):
        print (f"File [{resfile}] not exists, skip ...")
        return
    # else:
    print (f"Generate gif for [{resfile}]")
    #     return
    dirname = os.path.dirname(resfile)
    envfile = os.path.join(dirname, "data.scen")
    anifile = os.path.join(dirname, fname.removesuffix(".json") +".gif")

    exists = os.path.exists(anifile)
    if exists or len(pp.readJsonPath(resfile)) == 0:
        return
    ani = pp.draw_sol(envfile, resfile, 20, 500)
    print (f"Generating animation for [{resfile}] ...")
    print (f"Saving to [{anifile}] ...")
    ani.save(anifile)
    pp.plt.close()
    print ("Done.")

def runall(dirname: str):
    for scen in os.listdir(dirname):
        run(os.path.join(dirname, scen), "kcbs.json")
        run(os.path.join(dirname, scen), "micp-UB.json")


if __name__ == "__main__":
    if sys.argv[1] == "all":
        runall("./cbs-scen")
    else:
        run(sys.argv[1], "kcbs.json")
