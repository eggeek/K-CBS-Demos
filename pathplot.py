from matplotlib.figure import Figure
from matplotlib.axes import Axes
from matplotlib.patches import Rectangle, Circle
import matplotlib.pyplot as plt
from matplotlib import animation
import matplotlib.colors as mcolors

from typing import TypeAlias

CellPath: TypeAlias = tuple[list[tuple[int, int]], list[float]]
CIDPath: TypeAlias = tuple[list[int], list[float]]

TimeSteps: TypeAlias = list[float]
Vert: TypeAlias = tuple[float, float]
Poly: TypeAlias = list[Vert]
VertPath: TypeAlias = tuple[list[Vert], TimeSteps]


class Env:
    def __init__(
        self,
        minx: float = 0,
        maxx: float = 0,
        miny: float = 0,
        maxy: float = 0,
        maxt: float = 0,
        obsts: list[Poly] | None = None,
        robots: dict[int, Vert] | None = None,
    ):
        self.minx: float = minx
        self.maxx: float = maxx
        self.miny: float = miny
        self.maxy: float = maxy
        self.maxt: float = maxt

        if obsts is None:
            obsts = []
        if robots is None:
            assert False
        self.robots = {rid: bot for rid, bot in robots.items()}
        self.set_obsts(obsts)

    def bbox(self) -> Poly:
        return [
            (self.minx, self.miny),
            (self.minx, self.maxy),
            (self.maxx, self.maxy),
            (self.maxx, self.miny),
        ]

    def xbound(self) -> tuple[float, float]:
        return (self.minx, self.maxx)

    def ybound(self) -> tuple[float, float]:
        return (self.miny, self.maxy)

    def set_obsts(self, obsts: list[Poly]):
        self.obsts = obsts
        # self.inflated_obsts = [ObstacleInflation(obs, self) for obs in obsts]


def _draw_static_obstacles2D(ax, obsts: list[Poly], alpha=0.2):
    for obs in obsts:
        c00, _, c11, _ = obs
        x, y = c00[0], c00[1]
        dx, dy = c11[0] - c00[0], c11[1] - c00[1]
        patch = Rectangle((x, y), dx, dy, fill=True, color="black", alpha=alpha)
        ax.add_patch(patch)


def _draw_point(ax: Axes, v: Vert, t: float, is_2d=False, **kws):
    """
    kws: parameters for scatter
    """
    if is_2d:
        ax.plot([v[0]], [v[1]], **kws)
    else:
        ax.plot([v[0]], [v[1]], [t], **kws)


def _draw_path(ax: Axes, path: list[Vert], ts: TimeSteps, is_2d=False, **kws):
    color = kws.get("color", "blue")
    alpha = kws.get("alpha", 1)

    for i, (x_u, y_u) in enumerate(path[:-1]):
        t_u = ts[i]
        x_v, y_v = path[i + 1]
        t_v = ts[i + 1]

        if is_2d:
            point_x, point_y = ([x_u, x_v], [y_u, y_v])
            if i != 0 and i + 1 < len(path) - 1:
                ax.scatter(point_x, point_y, color=color, alpha=alpha)
            ax.plot(point_x, point_y, color=color, alpha=alpha)
        else:
            point_x, point_y, t = ([x_u, x_v], [y_u, y_v], [t_u, t_v])
            if i != 0 and i + 1 < len(path) - 1:
                ax.scatter(point_x, point_y, t, color=color, alpha=alpha)
            ax.plot(point_x, point_y, t, color=color, alpha=alpha)

    _draw_point(
        ax, path[0], ts[0], is_2d, color=color, mfc="none", alpha=1, marker="o", ms=10
    )
    _draw_point(ax, path[-1], ts[-1], is_2d, alpha=1, marker="+", ms=10, color=color)


def plot_path(
    agent_paths: dict[int, list[Vert]],
    env: Env,
    T: TimeSteps,
    frame_delay: float,
    numframes: int = 50,
    figsize=(15, 15),
):
    subp = plt.subplots(figsize=figsize)
    fig: Figure = subp[0]
    ax: Axes = subp[1]
    ax.set_xlim(env.xbound())
    ax.set_ylim(env.ybound())
    fig.tight_layout()

    tmax = max(T)
    dt = tmax / numframes

    _draw_static_obstacles2D(ax, env.obsts, alpha=0.8)

    # colors except gray
    clst = list(mcolors.TABLEAU_COLORS.keys())
    colors = {rid: clst[rid % len(clst)] for rid in agent_paths.keys()}
    labels = {}
    header = ax.annotate(
        "Time: 0\nStep: 0",
        xy=(env.maxx * 0.9, env.maxy * 0.9),
        xytext=(env.maxx * 0.9, env.maxy * 0.9),
    )
    for rid, path in agent_paths.items():
        x, y = path[0]
        labels[rid] = ax.annotate(
            f"{rid}", xy=(x, y), fontsize=10, ha="center", va="center", weight="bold"
        )
    rects: dict[int, Rectangle] = {}
    circles: dict[int, Circle] = {}

    for rid in agent_paths.keys():
        color = colors[rid]
        v = agent_paths[rid][0]
        xL, yL = env.robots[rid]
        rects[rid] = Rectangle(
            (v[0]-xL/2, v[1]-yL/2), xL, yL, color=color, alpha=0.3, linewidth=3
        )
        circles[rid] = Circle(v, radius=(xL + yL) / 20, alpha=0.7, color=color)
        ax.add_patch(rects[rid])
        ax.add_patch(circles[rid])
        _draw_path(ax, agent_paths[rid], T, alpha=0.1, is_2d=True, color=color)

    from bisect import bisect_right

    def update_plot(frame: int):
        curT: float = frame * dt
        ts: int = bisect_right(T, curT) - 1
        header.set_text(f"Time: {curT:.4f}\nStep: {ts}")
        for rid, path in agent_paths.items():
            if ts + 1 < len(path):
                x1, y1 = path[ts]
                x2, y2 = path[ts + 1]
                vx = (x2 - x1) / (T[ts + 1] - T[ts])
                vy = (y2 - y1) / (T[ts + 1] - T[ts])
                x = x1 + vx * (curT - T[ts])
                y = y1 + vy * (curT - T[ts])
            else:
                x, y = path[ts]
            labels[rid].set_position((x, y + 0.1))
            xL, yL = env.robots[rid]
            rects[rid].set_xy((x - xL/2, y - yL/2))
            # locs[rid].set_data([x], [y])
            circles[rid].set_center((x, y))
        # for rid, path in agent_paths.items():
        #     locs[rid].set_data([path[ts][0]], [path[ts][1]])
        # return [header] + list(labels.values()) + list(rects.values()) + list(locs.values())
        return (
            [header]
            + list(labels.values())
            + list(rects.values())
            + list(circles.values())
        )

    from tqdm.auto import tqdm
    import numpy as np
    # Create animation
    ani = animation.FuncAnimation(
        fig, update_plot, frames=tqdm(np.arange(numframes), initial=1), interval=frame_delay, blit=True
    )
    return ani


def readEnv(mapfile: str) -> Env:
    """
    <minx maxx>
    <miny maxy>
    <numBots>
    <botXL botYL> x numBots
    <numObs>
    <xleft yleft xL yL> x numObs
    """

    with open(mapfile, "r") as f:
        minx, maxx = map(float, f.readline().split())
        miny, maxy = map(float, f.readline().split())

        numBots = int(f.readline().strip())
        bots = {}
        for i in range(numBots):
            xL, yL = map(float, f.readline().split())
            bots[i] = (xL, yL)

        numObs = int(f.readline())
        obsts = []
        for _ in range(numObs):
            x, y, xL, yL = map(float, f.readline().split())
            obsts.append(((x, y), (x, y+yL), (x+xL, y+yL), (x+xL, y)))


        env = Env(minx, maxx, miny, maxy, 10, obsts, bots)
        return env


def readPath(pathfile: str, prefix: str = "Robot") -> dict[int, list[Vert]]:
    paths: dict[int, list[Vert]] = {}
    with open(pathfile, "r") as f:
        rid = -1
        for line in f.readlines():
            line = line.strip()
            if len(line) == 0:
                continue
            if line.startswith(prefix):
                _, srid = line.split(' ')
                rid = int(srid)
                paths[rid] = []
                continue
            vars = list(map(float, line.split(' ')))
            x, y = vars[:2]
            paths[rid].append((x, y))
    return paths


def draw_sol(envfile: str, pathfile: str, interval: float=10, numframes: int=-1):
    env = readEnv(envfile)
    paths = readPath(pathfile)
    maxSteps = max([len(path) for path in paths.values()])
    stepSize = 0.1
    T = [i*stepSize for i in range(maxSteps)]
    for rid in paths:
        while len(paths[rid]) < len(T):
            paths[rid].append(paths[rid][-1]) 
    
    if numframes == -1:
        numframes = len(T)
    ani = plot_path(paths, env, T, interval, numframes=numframes)
    return ani

if __name__ == "__main__":
    envfile = "./n4-10x10-rect2-env.txt"
    resfile = "./K-CBS-n4-10x10-rect2-plan.txt"
    print ("Generating animation ...")
    ani = draw_sol(envfile, resfile)
    print ("Done, showing aniamtion ...")
    plt.show()
    print ("Saving to file 'ani.mp4' ...")
    ani.save("ani.mp4")
