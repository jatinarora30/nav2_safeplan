from safeplan.algos.a_star import AStar
from safeplan.algos.voronoi_planner import VoronoiPlanner
from safeplan.algos.upp import UPP
from safeplan.algos.rrt import RRT
from safeplan.algos.sdf_astar import SDFAStar
from safeplan.algos.cbf_rrt import CBFRRT
from safeplan.algos.optimized_astar import OptimizedAStar
from safeplan.algos.fs_planner import FSPlanner
from safeplan.algos.safe_astar import SafeAStar

def planner_factory(name, args):
    if name == 'AStar':
        return AStar(**args)
    elif name == 'RRT':
        return RRT(**args)
    elif name == 'UPP':
        return UPP(**args)
    elif name == 'CBFRRT':
        return CBFRRT(**args)
    elif name == 'FSPlanner':
        return FSPlanner(**args)
    elif name == 'SDFAStar':
        return SDFAStar(**args)
    elif name == 'OptimizedAStar':
        return OptimizedAStar(**args)
    elif name == 'SafeAStar':
        return SafeAStar(**args)
    elif name == 'VoronoiPlanner':
        return VoronoiPlanner(**args)
    else:
        raise ValueError(f"Unknown planner: {name}")
