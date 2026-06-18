import numpy as np
import time
import json

from .base_lidar_attack import BaseLidarAttack

class ObjectPatchAttack(BaseLidarAttack):

    def __init__(self, parameters):
        self.attack_patches = parameters["patches"] #TODO: error handling
        self._patch_anchors = {}

        self.NAME = parameters.get(
            "attack_name",
            "Lidar Attack: Object Side Patch"
        )

        self.front_filter = parameters.get(
            "front_filter",
            {
                "x_min": 3.0,
                "x_max": 40.0,
                "y_abs_max": 4.0,
                "z_min": -1.9,
                "z_max": 2.0,
            }
        )

    def apply(self, lidar_data):

        attack_applied = False

        # Attack logic - only apply attack if attack enabled 
        if len(self.attack_patches) > 0:

            #self._debug_print_front_points(lidar_data, interval_sec=30.0)
            #self._debug_estimate_vehicle_rectangle(lidar_data, interval_sec=30.0) 

            # 1. get the front points where we assume the target vehicle 
            front_points, front_mask = self._get_front_region(lidar_data)
            front_indices = np.where(front_mask)[0]

            # 2. filter the front points to remove points that likely do not belong to front vehicle 
            vehicle_points = self._extract_vehicle_cluster(front_points)

            if vehicle_points is not None:
                # 3. estimate the dimensions of the front vehicle by approximating it with a simple rectangle 
                rect = self._estimate_vehicle_rectangle(vehicle_points)

                # 4. estimate the point spacing in y and z direction - such that we generate a realistic number and spacing of points 
                y_vals = vehicle_points[:, 1]
                z_vals = vehicle_points[:, 2]

                sorted_y = np.sort(y_vals)
                dy = np.diff(sorted_y)
                dy = dy[dy > 0.02]
                mean_dy = np.median(dy) if len(dy) > 0 else 0.1

                sorted_z = np.sort(z_vals)
                dz = np.diff(sorted_z)
                dz = dz[dz > 0.02]
                mean_dz = np.median(dz) if len(dz) > 0 else 0.1

                for patch_cfg in self.attack_patches:

                    lidar_data = self._generate_adaptive_side_patch(
                        lidar_data,
                        rect,
                        front_indices,
                        mean_dy,
                        mean_dz,
                        x_offset=patch_cfg["x_offset"],
                        y_offset=patch_cfg["y_offset"],
                        z_offset=patch_cfg["z_offset"],
                        side=patch_cfg["side"],
                        patch_size=patch_cfg["patch_size"],
                        stabilize_anchor=patch_cfg["stabilize_anchor"],
                        print_interval=30.0
                    )

                    attack_applied = True

        return lidar_data, attack_applied

    ## Helper functions

    # Get the points in the front where our target vehicle is located 
    def _get_front_region(self, lidar_data):
        xyz = lidar_data[:, :3]

        #search points in front of ego: 5-40m away, within 4m left/right, and between -1,9 and 2m height (from sensor, exlude groung points)
        cfg = self.front_filter

        front_mask = (
            (xyz[:, 0] > cfg["x_min"]) &
            (xyz[:, 0] < cfg["x_max"]) &
            (np.abs(xyz[:, 1]) < cfg["y_abs_max"]) &
            (xyz[:, 2] > cfg["z_min"]) &
            (xyz[:, 2] < cfg["z_max"])
        )

        return xyz[front_mask], front_mask
    
    # Filters the front points to remove outliers that likely don't belong to front vehicle
    # If too few points discard, as then attack might not work too well or the object is much smaller than a vehicle
    def _extract_vehicle_cluster(self, front_points, min_points=10):
        if front_points.shape[0] < min_points:
            return None

        centroid_guess = np.mean(front_points, axis=0)
        distances = np.linalg.norm(front_points - centroid_guess, axis=1)

        vehicle_points = front_points[distances < 3.0]

        if vehicle_points.shape[0] < min_points:
            return None

        return vehicle_points
    

    # Estimates the dimensions of our target vehicle by approximating it with a simple rectangle --> returns the rectangle dimensions 
    # Very simple approach, might not work well when we are not aligned with front car, but it is easier to understand and debug than PCA
    def _estimate_vehicle_rectangle(self, vehicle_points):
        x_vals = vehicle_points[:, 0]
        y_vals = vehicle_points[:, 1]
        z_vals = vehicle_points[:, 2]

        x_min, x_max = x_vals.min(), x_vals.max()
        y_min, y_max = y_vals.min(), y_vals.max()
        z_min, z_max = z_vals.min(), z_vals.max()

        """# Rectangle center (visible part) - might be used instead of the centroid below
        center = np.array([
            (x_min + x_max) / 2.0,
            (y_min + y_max) / 2.0,
            (z_min + z_max) / 2.0
        ])"""

        return {
            "x_min": x_min,
            "x_max": x_max,
            "y_min": y_min,
            "y_max": y_max,
            "z_min": z_min,
            "z_max": z_max,
            "centroid": np.mean(vehicle_points, axis=0),
            "depth": x_max - x_min,
            "width": y_max - y_min,
            "height": z_max - z_min
        }
    
    # Generate patches next to front vehicle 
    def _generate_adaptive_side_patch(
            self,
            lidar_data,
            rect,
            front_indices,
            mean_dy,
            mean_dz,
            x_offset=-0.1,     # relative to vehicle x_min: if negative patch is in front of vehicle 
            y_offset=0.5,      # relative to vehicle y_max/y_min (depending on side) - basically how far away from vehicle side edges
            z_offset=0.0,      # relative to z_min - vertical placement of patch
            side="right",      # side of the car where patch is placed "right" or "left"
            patch_size=0.5,    # right now the patch is a square 
            stabilize_anchor=True,
            print_interval=30.0 # (debug) print interval in seconds 
    ):
        """
        Generates a LiDAR-realistic planar patch next to the front vehicle.
        Uses modular front-region, clustering and rectangle estimation functions.
        """

        x_min = rect["x_min"]
        y_max = rect["y_max"]
        y_min = rect["y_min"]
        z_min = rect["z_min"]
        z_max = rect["z_max"]


        # 5. compute the anchor of the patch - anchored to our car rectangular box  
        """current_anchor = np.array([
            x_min + x_offset,
            y_max + y_offset,
            (z_min + z_max) / 2.0
        ])"""

        if side == "right":
            anchor_y = y_max + y_offset
        elif side == "left":
            anchor_y = y_min - y_offset
        else:
            raise ValueError("side must be 'right' or 'left'")

        anchor_z = z_min + z_offset

        current_anchor = np.array([
            x_min + x_offset,
            anchor_y,
            anchor_z
        ])

        # either compute anchor once or recompute for each frame - once might be less inaccurate when far away but prevents the patch from moving in position 
        """if stabilize_anchor:
            if not hasattr(self, "_patch_anchor"):
                self._patch_anchor = current_anchor
            anchor = self._patch_anchor
        else:
            anchor = current_anchor"""
        
        #TODO: assign proper id to each patch 
        patch_id = f"{x_offset}_{y_offset}_{z_offset}_{side}_{patch_size}"

        if stabilize_anchor:
            if patch_id not in self._patch_anchors:
                self._patch_anchors[patch_id] = current_anchor
            anchor = self._patch_anchors[patch_id]
        else:
            anchor = current_anchor

        #Special case to introduce single atable points (mainly for probing)
        if patch_size == 0.0:
            patch = np.array([[anchor[0], anchor[1], anchor[2]]], dtype=np.float32)
        
        else:
            # 6. generate the lidar points for the patch based on anchor and the computed point spacing 
            y_coords = np.arange(
                anchor[1] - patch_size/2,
                anchor[1] + patch_size/2,
                mean_dy
            )

            """z_coords = np.arange(
                anchor[2] - patch_size/2,
                anchor[2] + patch_size/2,
                mean_dz
            )"""

            z_coords = np.arange(
                anchor[2],
                anchor[2] + patch_size,
                mean_dz
            )

            patch_points = [
                [anchor[0], y, z]
                for y in y_coords
                for z in z_coords
            ]

            patch = np.array(patch_points)

            if patch.shape[0] == 0:
                return lidar_data

            # 7. Simulate realistic sparsity - drop each point with 20% chance 
            # TODO: could also adapt this via param 
            keep_mask = np.random.rand(patch.shape[0]) > 0.2
            patch = patch[keep_mask]

        density = patch.shape[0]

        if density == 0:
            return lidar_data

        # 8. Add intensity and ring information to the patch points to generate correct points 
        # TODO: could make intensity also adaptive or based on car points
        # TODO: not sure if the ring information is really realistic, but not sure if it actually matters for the attack 
        new_points = np.zeros((density, 5), dtype=np.float32)
        new_points[:, 0:3] = patch
        new_points[:, 3] = 0.7 + 0.05 * np.random.randn(density) # assign a relatively high intensity + some small variations (in paper they mention reflective surfaces)

        chosen = np.random.choice(front_indices, density, replace=True) # Use ring information from nearby front points 
        new_points[:, 4] = lidar_data[chosen, 4]
    
        # Debug print - print the patch every print_interval seconds  
        now = time.time()

        if not hasattr(self, "_last_patch_print_time"):
            self._last_patch_print_time = 0.0

        do_print = False
        if now - self._last_patch_print_time > print_interval:
            self._last_patch_print_time = now
            do_print = True

        if do_print:
            print("\n=========== PATCH DEBUG ===========")
            print(f"\nVehicle distance: {x_min:.2f} m")
            print(f"Estimated horizontal spacing: {mean_dy:.3f} m")
            print(f"Estimated vertical spacing:   {mean_dz:.3f} m")
            print(f"Generated patch points: {density}")
            print(patch)
            print("=======================================\n")

        return np.vstack((lidar_data, new_points))
    
    # Debug function for the estimated vehicle rectangle 
    def _debug_estimate_vehicle_rectangle(self, lidar_data, interval_sec=30.0):
        now = time.time()
        if not hasattr(self, "_last_rect_debug_time"):
            self._last_rect_debug_time = 0.0
        if now - self._last_rect_debug_time < interval_sec:
            return
        self._last_rect_debug_time = now

        front_points, _ = self._get_front_region(lidar_data)
        vehicle_points = self._extract_vehicle_cluster(front_points)

        if vehicle_points is None:
            print("[RECT DEBUG] Not enough vehicle points.")
            return

        rect = self._estimate_vehicle_rectangle(vehicle_points)

        x_min = rect["x_min"]
        x_max = rect["x_max"]
        y_max = rect["y_max"]
        y_min = rect["y_min"]
        z_min = rect["z_min"]
        z_max = rect["z_max"]

        depth  = x_max - x_min
        width  = y_max - y_min
        height = z_max - z_min

        print("\n===== RECT DEBUG =====")
        print(f"\nVehicle distance: {x_min:.2f} m")
        print(f"Vehicle points: {vehicle_points.shape[0]}")
        print("\nEstimated visible dimensions:")
        print(f"Depth  (x span): {depth:.2f} m")
        print(f"Width  (y span): {width:.2f} m")
        print(f"Height (z span): {height:.2f} m")
        print("\nLateral edges:")
        print(f"Left edge  y ≈ {y_min:.3f}")
        print(f"Right edge y ≈ {y_max:.3f}")
        print(rect)
        print("======================\n")

    
    # Debug function for the detected front points 
    def _debug_print_front_points(self, lidar_data, interval_sec=30.0):
        now = time.time()
        if not hasattr(self, "_last_front_debug_time"):
            self._last_front_debug_time = 0.0
        if now - self._last_front_debug_time < interval_sec:
            return
        self._last_front_debug_time = now

        front_points, _ = self._get_front_region(lidar_data)

        print("\n================ FRONT POINT DEBUG ================")
        print(f"Total front points: {front_points.shape[0]}")

        if front_points.shape[0] == 0:
            print("No front points detected.")
            print("===================================================\n")
            return

        order = np.argsort(front_points[:, 0])
        front_points_sorted = front_points[order]

        print(front_points_sorted)

        print("===================================================\n")
