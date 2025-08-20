# Scipt to show the output of the Infinigen scene in Isaac Sim

import argparse
import time
import os
import sys
import numpy as np

# Parse command line arguments before starting SimulationApp
parser = argparse.ArgumentParser(description='Setup Infinigen environment in Isaac Sim')
parser.add_argument('usd_path', 
                    help='Path to USD/USDC file')
parser.add_argument('--coarse-approximation', '-ca', 
                    choices=['none', 'convexDecomposition','convexHull',
                     'boundingSphere', 'boundingCube', 'meshSimplification',
                      'triangleMesh'],
                    default='convexHull',
                    help='Collision approximation type (default: convexHull)')
parser.add_argument('--hide-walls', 
                    action='store_true',
                    default=True,
                    help='Hide ceiling and exterior walls (default: True)')

args = parser.parse_args()

from isaacsim import SimulationApp
simulation_app = SimulationApp(launch_config={"headless": False})

import omni.usd
import infinigen_sdg_utils as infinigen_utils
from pxr import Usd, UsdGeom, PhysxSchema, UsdPhysics

# ANSI escape codes for log colors
RED = "\033[91m"
YELLOW = "\033[93m"
GREEN = "\033[92m"
RESET = "\033[0m"

# Fine collider types are the ones that require a more precise collider approximation
fine_collider_types = ["skirt", "_wall", "_floor"]
# Rigid body types are the ones that can be moved by the robot and has dynamics 
rigid_body_types = ["Table", "Chair", "Rug"]

def verify_positions_unchanged(stage, before_transforms: dict, after_transforms: dict):
    """
    Verify that object positions haven't changed
    
    Args:
        stage: USD stage
        before_transforms: Dictionary of prim paths to transform matrices before setup
        after_transforms: Dictionary of prim paths to transform matrices after setup
    """
    changes_detected = 0
    total_checked = 0
    
    for prim_path, before_transform in before_transforms.items():
        if prim_path in after_transforms:
            after_transform = after_transforms[prim_path]
            total_checked += 1
            
            # Compare transform matrices (with small tolerance for floating point precision)
            if not transforms_are_equal(before_transform, after_transform, tolerance=1e-6):
                print(f"{YELLOW}Position changed for: {prim_path}{RESET}")
                changes_detected += 1
    
    if changes_detected == 0:
        print(f" Position verification: All {total_checked} object positions preserved")
    else:
        print(f" Position verification: {changes_detected}/{total_checked} objects moved")
    
    return changes_detected == 0

def transforms_are_equal(transform1, transform2, tolerance=1e-6):
    """Check if two transform matrices are equal within tolerance"""
    try:
        # Convert to numpy arrays for comparison
        if transform1 is None or transform2 is None:
            return transform1 == transform2
        
        # Simple comparison for basic cases
        return abs(transform1 - transform2) < tolerance if hasattr(transform1, '__sub__') else transform1 == transform2
    except:
        # Fallback to direct comparison
        return transform1 == transform2

def capture_transforms(stage, root_path="/Environment"):
    """
    Capture all transform data under the specified root path
    
    Args:
        stage: USD stage
        root_path: Root path to capture transforms from
        
    Returns:
        Dictionary mapping prim paths to their transform data
    """
    
    transforms = {}
    root_prim = stage.GetPrimAtPath(root_path)
    
    if not root_prim.IsValid():
        return transforms
    
    for prim in Usd.PrimRange(root_prim):
        if prim.HasAttribute("xformOp:translate"):
            transform_data = {
                'translate': prim.GetAttribute("xformOp:translate").Get(),
                'rotation': prim.GetAttribute("xformOp:rotateXYZ").Get() if prim.HasAttribute("xformOp:rotateXYZ") else None,
                'scale': prim.GetAttribute("xformOp:scale").Get() if prim.HasAttribute("xformOp:scale") else None,
                'orient': prim.GetAttribute("xformOp:orient").Get() if prim.HasAttribute("xformOp:orient") else None
            }
            transforms[str(prim.GetPath())] = transform_data
    
    return transforms

def setup_infinigen_scene(usd_file_path: str, coarse_approximation_type: str = "convexHull", hide_top_walls: bool = True):
    """
    Load Infinigen USD file and setup environment
    
    Args:
        usd_file_path: Full path to the USD file
        coarse_approximation_type: Collision approximation type
        hide_top_walls: Whether to hide ceiling and exterior walls
    """
    print(f"=== Starting Infinigen Environment Setup ===")
    print(f"File: {usd_file_path}")
    print(f"Coarse approximation: {coarse_approximation_type}")
    print(f"Hide walls: {hide_top_walls}")
    
    # 1. Create new stage
    print("Creating new stage...")
    omni.usd.get_context().new_stage()
    
    # 2. Load environment
    print(f"Loading environment: {usd_file_path}")
    try:
        env_prim = infinigen_utils.load_env(
            usd_path=usd_file_path, 
            prim_path="/Environment", 
            remove_existing=True
        )
        print(f"Environment successfully loaded to: /Environment{RESET}")
    except Exception as e:
        print(f"{RED}Loading failed: {e}{RESET}")
    
    # 3. Set default prim
    print("Setting /Environment as default prim...")
    setup_default_prim("/Environment")
    
    # 4. Update app to ensure loading is complete
    simulation_app.update()
    
    # 5. Capture initial transforms for verification
    stage = omni.usd.get_context().get_stage()
    print("Capturing initial object positions...")
    initial_transforms = capture_transforms(stage, "/Environment")
    print(f"Captured {len(initial_transforms)} object transforms")
    
    # 6. Setup environment with custom collider/rigid body options
    print("Setting up environment...")
    modify_environment(
        root_path="/Environment",
        coarse_approximation_type=coarse_approximation_type,
        hide_top_walls=hide_top_walls,
    )
    
    # 7. Verify positions haven't changed
    print("Verifying object positions...")
    final_transforms = capture_transforms(stage, "/Environment")
    position_preserved = verify_positions_unchanged(stage, initial_transforms, final_transforms)
    
    print(f"Environment setup complete.")
    
    # 8. Resolve scaling issues
    print("Resolving scaling issues...")
    infinigen_utils.resolve_scale_issues_with_metrics_assembler()
    print("Scaling issues resolved")
    
    # 9. Setup complete
    print("All setup completed")
    print(f"=== Environment Setup Complete! ===")

def setup_default_prim(prim_path: str = "/Environment"):
    """
    Set the specified prim as the default prim for the stage
    
    Args:
        prim_path: Path to the prim to set as default (default: "/Environment")
    """
    stage = omni.usd.get_context().get_stage()
    
    # Get the prim at the specified path
    prim = stage.GetPrimAtPath(prim_path)
    
    if not prim.IsValid():
        print(f"{YELLOW}Warning: Prim at path '{prim_path}' is not valid. Cannot set as default prim.{RESET}")
        return False
    
    # Set the default prim
    stage.SetDefaultPrim(prim)
    
    print(f"Default prim set to: {prim_path}")
    return True


def add_colliders(root_prim: Usd.Prim, approximation_type: str = "convexHull") -> None:
    """Add collision attributes to mesh and geometry primitives under the root prim."""
    for desc_prim in Usd.PrimRange(root_prim):
        if desc_prim.IsA(UsdGeom.Gprim):
            if not desc_prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_api = UsdPhysics.CollisionAPI.Apply(desc_prim)
            else:
                collision_api = UsdPhysics.CollisionAPI(desc_prim)
            collision_api.CreateCollisionEnabledAttr(True)
            
        if desc_prim.IsA(UsdGeom.Mesh):
            if approximation_type == "triangleMesh":
                # Use PhysX triangle mesh collision
                if not desc_prim.HasAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI):
                    PhysxSchema.PhysxTriangleMeshCollisionAPI.Apply(desc_prim)
            else:
                # Use standard USD Physics mesh collision
                if not desc_prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                    mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(desc_prim)
                else:
                    mesh_collision_api = UsdPhysics.MeshCollisionAPI(desc_prim)
                mesh_collision_api.CreateApproximationAttr().Set(approximation_type)

def modify_environment(root_path: str = None, coarse_approximation_type: str = "convexHull", hide_top_walls: bool = True):
    """
    Custom setup environment function with options for colliders only or colliders + rigid body
    
    Args:
        root_path: Root path for the environment
        coarse_approximation_type: Collision approximation type
        hide_top_walls: Whether to hide ceiling and exterior walls
    """
    # Fix ceiling lights: meshes are blocking the light and need to be set to invisible
    print("Fixing ceiling lights...")
    ceiling_light_meshes = infinigen_utils.find_matching_prims(["001_SPLIT_GLA"], root_path, "Xform")
    for light_mesh in ceiling_light_meshes:
        light_mesh.GetAttribute("visibility").Set("invisible")

    # Hide ceiling light meshes for lighting fix
    infinigen_utils.hide_matching_prims(["001_SPLIT_GLA"], root_path, "Xform")

    # Hide top walls for better debug view, if specified
    if hide_top_walls:
        print("Hiding ceiling and exterior walls...")
        infinigen_utils.hide_matching_prims(["_exterior", "_ceiling"], root_path)

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPseudoRoot() if root_path is None else stage.GetPrimAtPath(root_path)

    for prim in Usd.PrimRange(prim):
        if prim.IsA(UsdGeom.Mesh):
            prim_name = prim.GetName()
            
            # Determine approximation type based on fine collider types
            if any(fine_type in prim_name for fine_type in fine_collider_types):
                print(f"Adding triangle mesh collider to {prim_name}")
                collider_approx = "triangleMesh"
            else:
                collider_approx = coarse_approximation_type
            
            # Add collider with appropriate approximation
            add_colliders(prim, collider_approx)
            
            # Check if this prim should have rigid body dynamics
            if any(rigid_type in prim_name for rigid_type in rigid_body_types):
                print(f"Adding rigid body dynamics to {prim_name}")
                infinigen_utils.add_rigid_body_dynamics(prim)

# Main program
if __name__ == "__main__":
    try:

        usd_file_path = args.usd_path
        print(f"Using file path: {usd_file_path}")
        
        # Check if file exists
        if not os.path.exists(usd_file_path):
            print(f"File not found: {usd_file_path}")
            print("Please check the path and try again.")
            sys.exit(1)
        
        # Setup environment
        setup_infinigen_scene(
            usd_file_path=usd_file_path,
            coarse_approximation_type=args.coarse_approximation,
            hide_top_walls=args.hide_walls,
        )
    
    except Exception as e:
        print(f" Error occurred: {e}")
        print("Please check your arguments and try again.")
        sys.exit(1)
    
    last_print_time = time.time()
    print_interval = 2.0
    # Keep application running
    while simulation_app.is_running():
        current_time = time.time()
        # Print reminder messages every 2 seconds
        if current_time - last_print_time >= print_interval:
            print(f"{GREEN}=== PLEASE COLLECTED AND SAVE THE CURRENT SCENE BEFORE CLOSING THE WINDOW! ==={RESET}")
            print(f"{GREEN}=== PLEASE SELECT FLAT COLLECTION AND GROUP BY MDL ==={RESET}")
            print()
            last_print_time = current_time
        simulation_app.update()
    print("=== EXIT PROGRAM ===")

    simulation_app.close()
