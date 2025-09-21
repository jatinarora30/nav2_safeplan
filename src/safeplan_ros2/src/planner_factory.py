from benchmark_algos.astar import AStarPlanner
from benchmark_algos.rrt import RRTPlanner
from benchmark_algos.upp import UPPPlanner
from benchmark_algos.cbfrrt import CBFRRTPlanner
from benchmark_algos.fsplanner import FSPlanner
from benchmark_algos.sdf_astar import SDFAStarPlanner
from benchmark_algos.optimized_astar import OptimizedAStarPlanner
from benchmark_algos.safe_astar import SafeAStarPlanner
from benchmark_algos.voronoi import VoronoiPlanner

def planner_factory(name, args):
    if name == 'AStar':
        return AStarPlanner(**args)
    elif name == 'RRT':
        return RRTPlanner(**args)
    elif name == 'UPP':
        return UPPPlanner(**args)
    elif name == 'CBFRRT':
        return CBFRRTPlanner(**args)
    elif name == 'FSPlanner':
        return FSPlanner(**args)
    elif name == 'SDFAStar':
        return SDFAStarPlanner(**args)
    elif name == 'OptimizedAStar':
        return OptimizedAStarPlanner(**args)
    elif name == 'SafeAStar':
        return SafeAStarPlanner(**args)
    elif name == 'VoronoiPlanner':
        return VoronoiPlanner(**args)
    else:
        raise ValueError(f"Unknown planner: {name}")
