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
from omni.isaac.core.materials import PhysicsMaterial
from omni.isaac.core.objects.ground_plane import GroundPlane
from omni.isaac.core.utils.prims import create_prim

world = World(stage_units_in_meters=1)
sim_context = None

def V_REG(arti, jointIndexes, targetVels, p_coeff=0.1, delta = 0.05):
    nowVels = arti.get_joint_velocities(joint_indices=np.array(jointIndexes))
    nowEfforts = arti.get_measured_joint_efforts(joint_indices=np.array(jointIndexes))

    errVels = targetVels - nowVels
    newEfforts = nowEfforts+ p_coeff*errVels
    arti.set_joint_efforts(efforts=np.array(newEfforts), joint_indices=np.array(jointIndexes))

def P_REG(arti, jointIndexes, targetPos, p_coeff=500, delta = 0.05):
    print(f"Legs Target Pos: {targetPos}")
    nowPos = arti.get_joint_positions(joint_indices=np.array(jointIndexes))
    print(f"Legs now Pos: {nowPos}")
    nowEfforts = arti.get_measured_joint_efforts(joint_indices=np.array(jointIndexes))

    errPos = targetPos - nowPos
    newEfforts = nowEfforts+ p_coeff*errPos
    arti.set_joint_efforts(efforts=np.array(newEfforts), joint_indices=np.array(jointIndexes))


def set_drive_parameters(drive, target_type, target_value, stiffness=None, damping=None, max_force=None):
    """Enable velocity drive for a given joint"""

    if target_type == "position":
        if not drive.GetTargetPositionAttr():
            drive.CreateTargetPositionAttr(target_value)
        else:
            drive.GetTargetPositionAttr().Set(target_value)
    elif target_type == "velocity":
        if not drive.GetTargetVelocityAttr():
            drive.CreateTargetVelocityAttr(target_value)
        else:
            drive.GetTargetVelocityAttr().Set(target_value)

    if stiffness is not None:
        if not drive.GetStiffnessAttr():
            drive.CreateStiffnessAttr(stiffness)
        else:
            drive.GetStiffnessAttr().Set(stiffness)

    if damping is not None:
        if not drive.GetDampingAttr():
            drive.CreateDampingAttr(damping)
        else:
            drive.GetDampingAttr().Set(damping)

    if max_force is not None:
        if not drive.GetMaxForceAttr():
            drive.CreateMaxForceAttr(max_force)
        else:
            drive.GetMaxForceAttr().Set(max_force)


add_reference_to_stage(assetPath, "/PrimeChair")
create_prim("/DistantLight", "DistantLight") 
groundMat = PhysicsMaterial(prim_path="/World/groundMaterial",static_friction=1, dynamic_friction=0.8)
gp = GroundPlane(prim_path="/World/groundPlane", size=10, color=np.array([0.5, 0.5, 0.5]), physics_material=groundMat)

arti_view = Articulation(prim_path="/PrimeChair", name="PrimeChairArcticulation")
world.scene.add(gp)
world.scene.add(arti_view)
# wait for things to load
simulation_app.update()
while is_stage_loading():
    simulation_app.update()

stage = omni.usd.get_context().get_stage()
lb = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/base_link/LB_J"), "angular")
lf = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/base_link/LF_J"), "angular")
rb = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/base_link/RB_J"), "angular")
rf = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/base_link/RF_J"), "angular")

# nlf = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/base_link/NLF_J"), "angular")
# nlb = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/base_link/NLB_J"), "angular")
# nrf = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/base_link/NRF_J"), "angular")
# nrb = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/base_link/NRB_J"), "angular")

slb = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/NLB/SLB_J"), "angular")
slf = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/NLF/SLF_J"), "angular")
srb = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/NRB/SRB_J"), "angular")
srf = UsdPhysics.DriveAPI.Get(stage.GetPrimAtPath("/PrimeChair/NRF/SRF_J"), "angular")


world.reset()
#world.initialize_physics()

# lb = arti_view.get_dof_index("LB_J")
# rb = arti_view.get_dof_index("RB_J")
# lf = arti_view.get_dof_index("LF_J")
# rf = arti_view.get_dof_index("RF_J")

nlb = arti_view.get_dof_index("NLB_J")
nrb = arti_view.get_dof_index("NRB_J")
nlf = arti_view.get_dof_index("NLF_J")
nrf = arti_view.get_dof_index("NRF_J")

# slb = arti_view.get_dof_index("SLB_J")
# slf = arti_view.get_dof_index("SLF_J")
# srb = arti_view.get_dof_index("SRB_J")
# srf = arti_view.get_dof_index("SRF_J")

sim_context = SimulationContext.instance()

world.play()

for i in range(40*60):
    if ((i > 60*2) and (i < 60*6)):
        #velocityes in rad/s
        # action = ArticulationAction(joint_velocities=np.array([2, 2, 2, 2]), joint_indices=np.array([lb,lf,rb,rf]))
        # arti_view.apply_action(action)
        set_drive_parameters(lb, "velocity", math.degrees(2), 0, math.radians(1e8))
        set_drive_parameters(lf, "velocity", math.degrees(2), 0, math.radians(1e8))

        set_drive_parameters(rb, "velocity", math.degrees(2), 0, math.radians(1e8))
        set_drive_parameters(rf, "velocity", math.degrees(2), 0, math.radians(1e8))

    if ((i > 60*6) and (i < 60*15)):
        #velocityes in rad/s
         #action = ArticulationAction(joint_velocities=np.array([5, 5, -5, -5]), joint_indices=np.array([lb,lf,rb,rf]))
         #arti_view.apply_action(action)
        #arti_view.set_joint_velocities(velocities=np.array([2, 2, -2, -2]), joint_indices=np.array([lb,lf,rb,rf]))
        set_drive_parameters(lb, "velocity", math.degrees(-1), 0, math.radians(1e8))
        set_drive_parameters(lf, "velocity", math.degrees(-1), 0, math.radians(1e8))
        set_drive_parameters(rb, "velocity", math.degrees(1), 0, math.radians(1e8))
        set_drive_parameters(rf, "velocity", math.degrees(1), 0, math.radians(1e8))


    
    if (i > 60*15):
        #print("legs on")
        action = ArticulationAction(joint_positions=np.array([-3.14, 3.14, 3.14, -3.14]), joint_indices=np.array([nlb,nlf,nrb,nrf]))
        arti_view.apply_action(action)


    if (i > (60*20)):
        # action = ArticulationAction(joint_velocities=np.array([-2, -2, 2, 2]), joint_indices=np.array([slb,slf,srb,srf]))
        # arti_view.apply_action(action)
        set_drive_parameters(slb, "velocity", math.degrees(2), 0, math.radians(1e8))
        set_drive_parameters(slf, "velocity", math.degrees(2), 0, math.radians(1e8))

        set_drive_parameters(srb, "velocity", math.degrees(2), 0, math.radians(1e8))
        set_drive_parameters(srf, "velocity", math.degrees(2), 0, math.radians(1e8))

    world.step(render = True)

world.stop()
simulation_app.close()