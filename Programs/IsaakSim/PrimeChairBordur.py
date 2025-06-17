from isaacsim import SimulationApp

assetPath = "F:\Creations\Robots\IsaakSimScenes\PrimeChair\PrimeChairAsembArticulation.usd"

simulation_app = SimulationApp({"headless": False, })


import omni
import numpy as np
import math
from pxr import UsdPhysics

from omni.isaac.core import World
from omni.isaac.core import SimulationContext
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

from omni.isaac.core.utils.stage import add_reference_to_stage, is_stage_loading
from omni.isaac.core.materials import PhysicsMaterial, OmniPBR
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.utils.prims import create_prim

world = World(stage_units_in_meters=1)
sim_context = None

add_reference_to_stage(assetPath, "/PrimeChair")
create_prim("/DistantLight", "DistantLight",
             attributes={
            "inputs:intensity": 2500,
            "inputs:color": (1, 1, 1)
        }) 

groundMat = PhysicsMaterial(prim_path="/World/groundMaterial",static_friction=1, dynamic_friction=0.8)

omnipbr = OmniPBR(prim_path="/World/bordurVMat", color=np.array([0.4, 0.4, 0.4]))

gp = GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0.5, 0.5, 0.5]), physics_material=groundMat)

bordur = DynamicCuboid(prim_path="/World/bordur", scale=np.array([2.5, 1.0, 0.15]), visual_material = omnipbr, position=np.array([3, 0.0, 0.0]), mass=670, physics_material=groundMat)

arti_view = Articulation(prim_path="/PrimeChair", name="PrimeChairArcticulation")
world.scene.add(gp)
world.scene.add(arti_view)
world.scene.add(bordur)
# wait for things to load
simulation_app.update()
while is_stage_loading():
    simulation_app.update()

stage = omni.usd.get_context().get_stage()


world.reset()
#world.initialize_physics()

lb = arti_view.get_dof_index("LB_J")
rb = arti_view.get_dof_index("RB_J")
lf = arti_view.get_dof_index("LF_J")
rf = arti_view.get_dof_index("RF_J")

nlb = arti_view.get_dof_index("NLB_J")
nrb = arti_view.get_dof_index("NRB_J")
nlf = arti_view.get_dof_index("NLF_J")
nrf = arti_view.get_dof_index("NRF_J")

slb = arti_view.get_dof_index("SLB_J")
slf = arti_view.get_dof_index("SLF_J")
srb = arti_view.get_dof_index("SRB_J")
srf = arti_view.get_dof_index("SRF_J")

sim_context = SimulationContext.instance()

world.play()

for i in range(30*60):
    if ((i > 60*1) and (i < 60*3.5)):
        action = ArticulationAction(joint_positions=np.array([0, -0.78, 0, 0.78]), joint_indices=np.array([nlb,nlf,nrb,nrf]))
        arti_view.apply_action(action)

    if ((i > 60*2)):
        #velocityes in rad/s
        action = ArticulationAction(joint_velocities=np.array([2, 2, 2, 2]), joint_indices=np.array([lb,lf,rb,rf]))
        arti_view.apply_action(action)
    
    if ((i > 60*3.5) and (i < 60*13)):
        #print("legs on")
        action = ArticulationAction(joint_positions=np.array([-2.09, 2.09, 2.09, -2.09]), joint_indices=np.array([nlb,nlf,nrb,nrf]))
        arti_view.apply_action(action)


    if (i > (60*7)):
        action = ArticulationAction(joint_velocities=np.array([-2, -2, 2, 2]), joint_indices=np.array([slb,slf,srb,srf]))
        arti_view.apply_action(action)


    if (i > (60*13)):
        action = ArticulationAction(joint_positions=np.array([0, -0.78, 0, 0.78]), joint_indices=np.array([nlb,nlf,nrb,nrf]))
        arti_view.apply_action(action)
    
    if (i < (60*16)):
        world.step(render = True)
    else:
        world.pause()

simulation_app.close()