"""Lists the objects of the scene loaded in CoppeliaSim (Q-01) and the simulator version.
Usage: python3 tools/sim_bench/inspect_scene.py [port]
"""
import sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

port = int(sys.argv[1]) if len(sys.argv) > 1 else 23000
sim = RemoteAPIClient(port=port).require("sim")
print("version:", sim.getInt32Param(sim.intparam_program_full_version))
print("scene:", sim.getStringParam(sim.stringparam_scene_path_and_name))
for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
    print(f"{h:6d}  {sim.getObjectAlias(h, 1)}  [{sim.getObjectType(h) if hasattr(sim, 'getObjectType') else '?'}]")
