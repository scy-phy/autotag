import rclpy
import time
import carla
import numpy as np
import itertools
import ast
import csv
from collections import defaultdict



from patch_config_publisher import AttackPublisher
from perception_monitor import MultiPerceptionMonitor

INIT_FRAMES = 5
EVAL_FRAMES = 5

DISTANCES = [5, 10, 15, 20, 25, 30]

# P * D probings --> relatively cheap here
P = 1000 # probe sets  --> 1000 probings in first iteration
Q = 50  # points per set
M = 10  # number of sets to choose after probing

# N * M * D probings here --> most expensive step
N = 150 # removal iterations --> 80 * 10 * 5 = 4000 probings
K = 5 # number of points to remove per iteration

# M * Y * D probings --> can accumulate but still ok
Y = 40 # number of points to keep for later selection --> 750 probings

A = list(range(1, Y+1))   # number of points used in final selection
B = 3                     # number of best sets per A

SEARCH_X = (-1.0, 4.0)
SEARCH_Y = (-0.7, 1.0)
SEARCH_Z = (0.0, 2.3)
SIDES = ["left", "right"]

BASE_THRESH = 0.6 #0.65 # 0.6 for smaller cars, 0.65 for larger

#Test
#DISTANCES = [5, 10]

# P * D probings --> relatively cheap here
"""P = 20 # probe sets  --> 1000 probings
Q = 50  # points per set
M = 1  # number of sets to choose after probing

# N * M * D probings here --> most expensive step
N = 30 # removal iterations --> 80 * 10 * 5 = 4000 probings
K = 5 # number of points to remove per iteration

# M * Y * D probings --> can accumulate but still ok
Y = 30 # number of points to keep for later selection --> 750 probings

A = list(range(1, Y+1))   # number of points used in final selection
B = 3                     # number of best sets per A"""


## Location Probing ## --> find sets of locations that have adversarial impact
    # Define search space & params
    # Create P (probing iterations) sets of Q (probe points per set) points inside search space

    # Probe:
    # For d in D (distances):
        # Place ego at d
        # For p in P:
            # Insert Q points 
            # Calculate orig_adv_score: e^(2.0 - (centerp. det. + fusion det)) --> 2.0 here assuming perfect none attack setting det. 

    # Aggregate results:
    # For p in P:
        # Aggregate score over all d: Sum of them

    # (Store results in csv: for each p store set of q, per d store centerp. det., fusion det., score --> then also store the aggr. result)
    
    # Choose best sets:
    # Sort P and choose M best sets (highest score)


## Point Removal ## --> estimate which points are the most impactful per set
    # Define N (removal iterations) sets of K (number of removal points) points --> choose indices here
    # Initialize 2 sets:
        # imp_set: {x1: 0, x2: 0, ..., xq: 0}
        # un_imp_set: {x1: 0, x2: 0, ..., xq: 0}

    
    # For m in M:
        # Probe truncated sets
        # For d in D:
            # For n in N:
                # Remove K points from M --> (Q - K)
                # Calculate new_score (as above)
                # If trunc_adv_score << orig_adv_score:
                    # Update K points in imp_set --> increase by 1
                # Else:
                    # Update K points in un_imp_set --> increase by 1
                #--> Alternatively: delta = orig_score - trunc_score 
                                    # importance[x] += delta (automatically takes care of imp. and not imp.)
                                    # Then just need to sort after importance

        # Aggregate results:
        # For x in q:
            # agg_score = 0
            # For d in D:
                # agg_score += 2*count(imp_set(x)) - count(un_imp_set(x)) --> imp. is more important here so weigh higher
            # Put agg_score into upd_set
        # Sort upd_set (highest scores first)

        # Choose Y first points to build M_upd

    # (Store the upd_set (actual points) in csv)


## Point Selection ## --> actually choose the important points
    # For m in M_upd:
        # For y in Y:
            # Add y to location set 
            # Probe and calculate score
    # For each a in A (number of requested locations):
        # Choose B best sets (highest score for that a)

    # (Store for each a an m the scores + actual points)


class LocationSearchRunner:

    def __init__(self):

        # Connect to CARLA
        self.client = carla.Client("localhost", 1403)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        settings = self.world.get_settings()
        assert settings.synchronous_mode, "CARLA must be in synchronous mode"

        # ROS
        rclpy.init()

        self.attack_node = AttackPublisher()
        self.monitor_node = MultiPerceptionMonitor()
        self.benign_baselines = {}

    # Set / reset the ego vehicle 
    # If distance = None set to default position as in objects.json, if distance set calculate x roughly based on target
    def reset_ego(self, distance = None):
        actors = self.world.get_actors().filter('vehicle.*')

        ego = None
        for a in actors:
            if a.attributes.get('role_name') in ['ego_vehicle', 'hero']:
                ego = a
                break

        if ego is None:
            raise RuntimeError("Ego vehicle not found")

        ego.set_target_velocity(carla.Vector3D(0,0,0))
        ego.set_target_angular_velocity(carla.Vector3D(0,0,0))

        #TODO: also this seems to desynchronize the map in AW
        if distance:
            x = 19.35 - 2 - distance # only rough estimate
            transform = carla.Transform(
                #Target vehicle: x=19.35, y=137.46, z=0.6 yaw=0.32 (according to spawn point (75))
                carla.Location(x=x, y=137.96, z=0.6),
                carla.Rotation(yaw=0.352127)
            )
        else:
            transform = carla.Transform(
                carla.Location(x=-54.344658, y=137.050995, z=0.6),
                carla.Rotation(yaw=0.352127)
            )

        ego.set_transform(transform)

    # The world is ticked by the benign scenario script already - so wait for some ticks 
    def tick_n(self, n):
        for _ in range(n):
            self.world.wait_for_tick()


    #L is the maximum confidence score: the lower = better for attack 
    def init_and_calculate_L_score(self):
        self.tick_n(INIT_FRAMES)

        self.monitor_node.reset_buffers()

        start_count = self.monitor_node.frame_counters["centerpoint"]

        while (
            self.monitor_node.frame_counters["centerpoint"] - start_count
            < EVAL_FRAMES
        ):
            rclpy.spin_once(self.monitor_node, timeout_sec=0.1)

        (
            _,
            center_det,
            center_cls,
            center_class_conf,
            center_exist
        ) = self.monitor_node.compute_max_confidence("centerpoint")

        (
            _,
            fusion_det,
            fusion_cls,
            fusion_class_conf,
            fusion_exist
        ) = self.monitor_node.compute_max_confidence("fusion")

        return (center_exist, center_det, fusion_exist, fusion_det)
    

    # Helper to generate valid points
    def sample_probe_point(self):
        while True:

            x = np.random.uniform(*SEARCH_X)
            y = np.random.uniform(*SEARCH_Y)
            z = np.random.uniform(*SEARCH_Z)
            side = np.random.choice(SIDES)

            # avoid points inside / behind vehicle
            if (x >= 0 and z <= 1.4 and y < 0):
                continue

            return (x, y, z, side)
        
    # Helper to generate a probe set
    def generate_probe_sets(self):
        probe_sets = []

        for p in range(P):
            patches = []

            for _ in range(Q):
                x, y, z, side = self.sample_probe_point()

                patch = {
                    "x_offset": float(x),
                    "y_offset": float(y),
                    "z_offset": float(z),
                    "patch_size": 0.0,  # probing uses single points
                    "side": side,
                    "stabilize_anchor": False
                }

                patches.append(patch)

            probe_sets.append(patches)

        return probe_sets
    

    def collect_benign_baseline(self, distance):

        print(f"Collecting benign baseline for distance {distance}")

        self.attack_node.set_config([], enabled=False)
        rclpy.spin_once(self.attack_node)

        self.tick_n(INIT_FRAMES)
        self.monitor_node.reset_buffers()

        start_count = self.monitor_node.frame_counters["centerpoint"]

        while (
            self.monitor_node.frame_counters["centerpoint"] - start_count
            < (2 * EVAL_FRAMES)
        ):
            rclpy.spin_once(self.monitor_node, timeout_sec=0.1)

        (
            _,
            center_det,
            center_cls,
            center_conf,
            center_exist
        ) = self.monitor_node.compute_max_confidence("centerpoint")

        (
            _,
            fusion_det,
            fusion_cls,
            fusion_conf,
            fusion_exist
        ) = self.monitor_node.compute_max_confidence("fusion")

        self.benign_baselines[distance] = {
            "center_det": center_det,
            "center_cls": center_cls,
            "center_conf": center_conf,
            "center_exist": center_exist,
            "fusion_det": fusion_det,
            "fusion_cls": fusion_cls,
            "fusion_conf": fusion_conf,
            "fusion_exist": fusion_exist
        }
    

    def location_probing(self):

        # Init csv:
        csv_file = open("location_probing_results.csv", "w", newline="")
        writer = csv.writer(csv_file)

        writer.writerow([
            "mode",
            "probe_id",
            "distance",
            "center_det_rate",
            "fusion_det_rate",
            "center_exist_prob",
            "fusion_exist_prob",
            "points"
        ])
    
        #Create probe sets
        probe_sets = self.generate_probe_sets() 
        first_dist_id = 0

        aggregate_scores_center = {}
        aggregate_scores_fusion = {}
        
        #First distance (smallest) is used to prune P (i.e. only keep promising p)
        first_distance = True
        thresh = BASE_THRESH
 
        for d in DISTANCES:
            print(f"\n=== Distance {d}m ===")
            self.reset_ego(distance=d)
            time.sleep(5)
            self.tick_n(10)
            print("Ego vehicle set, starting probing")

            # collect benign metrics
            self.collect_benign_baseline(d)
            baseline = self.benign_baselines[d]

            writer.writerow([
                "benign",
                -1,
                d,
                baseline["center_det"],
                baseline["fusion_det"],
                baseline["center_exist"],
                baseline["fusion_exist"],
                []
            ])

            if first_distance:
                new_probe_sets = []

            #Start probing for that distance
            for probe_id, patches in enumerate(probe_sets):

                print(f"Probe set {probe_id}/{len(probe_sets)}")

                # enable attack
                self.attack_node.set_config(patches, enabled=True)
                rclpy.spin_once(self.attack_node)

                (
                    center_exist,
                    center_det,
                    fusion_exist,
                    fusion_det
                ) = self.init_and_calculate_L_score()
                
                if (not first_distance) or (center_exist <= thresh and fusion_exist < 0.1):

                    if not first_distance:
                        id = probe_id
                        aggregate_scores_center[id] += center_exist
                        aggregate_scores_fusion[id] += fusion_exist
                    else:
                        id = first_dist_id
                        first_dist_id += 1
                        aggregate_scores_center[id] = center_exist
                        aggregate_scores_fusion[id] = fusion_exist
                        new_probe_sets.append(patches)

                    writer.writerow([
                        "attack",
                        id,
                        d,
                        center_det,
                        fusion_det,
                        center_exist,
                        fusion_exist,
                        patches
                    ])

            if first_distance:

                probe_sets = new_probe_sets

                print(f"\nPruned probe sets: {len(probe_sets)} remaining")

                first_distance = False


        csv_file.close()

        return probe_sets, aggregate_scores_center, aggregate_scores_fusion
    

    def select_best_probe_sets(self, probe_sets, aggregate_scores_center, aggregate_scores_fusion):
        sorted_ids = sorted(
            aggregate_scores_center,
            key=lambda x: aggregate_scores_center[x],
            reverse=False
        )

        filtered_ids = [
            i for i in sorted_ids
            if aggregate_scores_fusion[i] < 0.1 # make sure fusion is also affected
        ]

        best_ids = filtered_ids[:M]

        best_sets = [probe_sets[i] for i in best_ids]

        print("\nBest probe sets:")
        for i in best_ids:
            print(f"Set {i} score = {aggregate_scores_center[i]}")

        return best_sets
    


    def point_adv_score(self, patch_set, removal_sets):

        results_per_distance = {}

        for d in DISTANCES:
            self.reset_ego(distance=d)
            time.sleep(5)
            self.tick_n(10)
            print(f"Ego vehicle set, starting removal at distance {d}")


            # Compute baseline score for that distance
            # enable attack
            self.attack_node.set_config(patch_set, enabled=True)
            rclpy.spin_once(self.attack_node)

            baseline = self.benign_baselines[d]

            orig_center_exist, _ , orig_fusion_exist, _ = self.init_and_calculate_L_score()

            center_insertion_score = np.exp(baseline["center_exist"] - orig_center_exist)
            fusion_insertion_score = np.exp(baseline["fusion_exist"] - orig_fusion_exist)

            w_insertion = {i: (center_insertion_score, fusion_insertion_score) for i in range(len(patch_set))} #Insertion score for all points in set 

            # Calculate L(X - removed)
            removal_scores = {}

            for n, remove_idx in enumerate(removal_sets):

                truncated = [
                    p for i, p in enumerate(patch_set)
                    if i not in remove_idx
                ]

                # enable attack for truncated
                self.attack_node.set_config(truncated, enabled=True)
                rclpy.spin_once(self.attack_node)

                trunc_center_exist, _ , trunc_fusion_exist, _ = self.init_and_calculate_L_score()

                trunc_score_n = (trunc_center_exist, trunc_fusion_exist)

                removal_scores[n] = trunc_score_n

            # Calculate removal score for each point
            w_removal = {}

            for i, x in enumerate(patch_set):

                # find all removal sets that include x
                relevant_removals = [
                    n for n, remove_idx in enumerate(removal_sets)
                    if i in remove_idx
                ]

                # calculate average removal score when x is removed
                avg_trunc_center_exist = np.mean([
                    np.exp(removal_scores[n][0] - orig_center_exist) for n in relevant_removals
                ])

                avg_trunc_fusion_exist = np.mean([
                    np.exp(removal_scores[n][1] - orig_fusion_exist) for n in relevant_removals
                ])

                w_removal[i] = (avg_trunc_center_exist, avg_trunc_fusion_exist)

            # Calculate final w 
            w_final = {}

            for i in range(len(patch_set)):

                w_final[i] = (
                    w_insertion[i][0] + w_removal[i][0],
                    w_insertion[i][1] + w_removal[i][1]
                )

            results_per_distance[d] = {
                "removal_scores": removal_scores,
                "w_insertion": w_insertion,
                "w_removal": w_removal,
                "w_final": w_final
            }

        return results_per_distance



    def point_selection_score(self, patches, sorted_point_list, distance):

        print(f"\n=== Point selection for distance {distance}m ===")

        self.reset_ego(distance=distance)
        time.sleep(5)
        self.tick_n(10)
        print(f"Ego vehicle set, starting removal at distance {distance}")

        history = []

        # Base - insert a single point
        cand_patches = [patches[sorted_point_list[0][0]]]
        self.attack_node.set_config(cand_patches, enabled=True)
        rclpy.spin_once(self.attack_node)

        curr_center_score, _, curr_fusion_score, _ = self.init_and_calculate_L_score()
        current_points = [sorted_point_list[0][0]]

        history.append({
            "step": len(current_points),
            "points": current_points.copy(),
            "center_score": curr_center_score,
            "fusion_score": curr_fusion_score
        })

        for point_idx, _, _ in sorted_point_list[1:]:
            
            curr_patches = cand_patches.copy()
            curr_patches.append(patches[point_idx])
            self.attack_node.set_config(curr_patches, enabled=True)
            rclpy.spin_once(self.attack_node)

            center_score, _, fusion_score, _ = self.init_and_calculate_L_score()

            # Only consider adding point if it improves center score (significantly) or fusion score is affected
            if ((curr_center_score - center_score) > 0.005 and fusion_score <= curr_fusion_score) or ((curr_center_score - center_score) >= 0.0 and fusion_score < curr_fusion_score):

                curr_center_score = center_score
                curr_fusion_score = fusion_score
                current_points.append(point_idx)
                cand_patches.append(patches[point_idx])

                history.append({
                    "step": len(current_points),
                    "points": current_points.copy(),
                    "center_score": curr_center_score,
                    "fusion_score": curr_fusion_score
                })

        return history


    def point_removal(self, best_sets):

        csv_file = open("point_removal_importance.csv", "w", newline="")
        writer = csv.writer(csv_file)

        writer.writerow([
            "set_id",
            "distance",
            "point_rank",
            "point_idx",
            "w_score_center",
            "w_score_fusion",
            "x_offset",
            "y_offset",
            "z_offset",
            "side"
        ])

        csv_file_removal = open("point_removal_set.csv", "w", newline="")
        writer_removal = csv.writer(csv_file_removal)

        writer_removal.writerow([
            "set_id",
            "distance",
            "removal_set_id",
            "score_center",
            "score_fusion",
            "orig_score_center",
            "orig_score_fusion",
            "removed_patches"
        ])

        csv_file_selection = open("point_selection_set.csv", "w", newline="")
        writer_selection = csv.writer(csv_file_selection)

        writer_selection.writerow([
            "set_id",
            "distance",
            "num_points",
            "score_center",
            "score_fusion",
            "point_idx",
            "points"
        ])

        csv_file_agg = open(f"point_aggregate_selection.csv", "w", newline="")
        writer_agg = csv.writer(csv_file_agg)

        writer_agg.writerow([
            "set_id",
            "point_idx",
            "x_offset",
            "y_offset",
            "z_offset",
            "side",
            "avg_w_center",
            "avg_w_fusion",
            "selection_frequency",
            "avg_rank",
            "ranks",
            "num_selected",
            "num_total"
        ])

        # Pre-generate removal index sets
        removal_sets = [
            np.random.choice(Q, K, replace=False)
            for _ in range(N)
        ]

        if not self.benign_baselines:
            for d in DISTANCES:
                print(f"\n=== Distance {d}m ===")
                self.reset_ego(distance=d)
                time.sleep(5)
                self.tick_n(10)
                print("Ego vehicle set, computing baselines")

                # collect benign metrics
                self.collect_benign_baseline(d)
                baseline = self.benign_baselines[d]

        for m_id, patches in enumerate(best_sets):

            print(f"\n===== Point removal for set {m_id} =====")

            results_per_distance = self.point_adv_score(patches, removal_sets)

            point_aggregate = defaultdict(lambda: {
                "point_idx": 0.0,
                "w_center_scores": [],
                "w_fusion_scores": [],
                "selected_count": 0,
                "total_count": 0,
                "importance_rank": 0,
                "ranks": []
            })

            for d, metrics in results_per_distance.items():
                w_final = metrics["w_final"]

                # Create a list of tuples: (point_idx, w_center, w_fusion)
                points_list = [
                    (i, w_final[i][0], w_final[i][1]) for i in w_final
                ]

                # Sort descending - only consider center score here for sorting
                points_list.sort(key=lambda x: x[1] if not np.isnan(x[1]) else -np.inf, reverse=True)
                

                # Write point ranks to csv
                for rank, (point_idx, w_center, w_fusion) in enumerate(points_list):

                    # Write to csv
                    patch = patches[point_idx]
                    writer.writerow([
                        m_id,      
                        d,            
                        rank,         # point rank
                        point_idx,    # point index in set
                        w_center,
                        w_fusion,
                        patch["x_offset"],     
                        patch["y_offset"],     
                        patch["z_offset"],     
                        patch["side"]      
                    ])

                
                orig_score = metrics["w_insertion"][0]
                removal_list = []

                # Write removal set scores to csv
                for n, remove_idx in enumerate(removal_sets):
                    
                    trunc_center_exist, trunc_fusion_exist = metrics["removal_scores"][n]

                    # Get actual removed patches
                    removed_patches = [patches[i] for i in remove_idx]

                    if trunc_fusion_exist < 0.1 and trunc_center_exist <= orig_score[0]:
                        removal_list.append((
                            n,
                            trunc_center_exist,
                            trunc_fusion_exist,
                            remove_idx
                        ))

                    writer_removal.writerow([
                        m_id,
                        d,
                        n, 
                        trunc_center_exist,
                        trunc_fusion_exist,
                        orig_score[0],
                        orig_score[1],
                        removed_patches
                    ])

                # Do selection 

                removed_points = []
                # First remove obviously bad points:
                if len(removal_list) > 0:
                    removal_list.sort(key=lambda x: x[1])

                    # only remove points from one set, otherwise would needd check first that points removing more points has the same effect
                    removed_points = removal_list[0][3]

                # Selection

                #Filter the sorted point list
                filtered_sorted_points = [
                    i for i in points_list if i[0] not in removed_points
                ]

                history = self.point_selection_score(patches, filtered_sorted_points, d)

                for entry in history:
                    points = [patches[idx] for idx in entry["points"]]
                    writer_selection.writerow([
                        m_id,
                        d,
                        entry["step"],
                        entry["center_score"],
                        entry["fusion_score"],
                        entry["points"],
                        points
                    ])

                selection = history[-1]["points"]

                # For aggregation:
                for i in range(len(patches)):

                    patch = patches[i]

                    # Create a unique key for the point
                    key = (
                        patch["x_offset"],
                        patch["y_offset"],
                        patch["z_offset"],
                        patch["side"]
                    )

                    point_aggregate[key]["point_idx"] = i
                    point_aggregate[key]["total_count"] += 1

                    if i in selection:
                        point_aggregate[key]["selected_count"] += 1
                        point_aggregate[key]["importance_rank"] += selection.index(i)  # index in sorted list --> lower = better
                        point_aggregate[key]["ranks"].append(selection.index(i))

                    point_aggregate[key]["w_center_scores"].append(metrics["w_final"][i][0])
                    point_aggregate[key]["w_fusion_scores"].append(metrics["w_final"][i][1])

            for key, stats in point_aggregate.items():

                x, y, z, side = key

                avg_center = (
                    np.mean(stats["w_center_scores"])
                    if stats["w_center_scores"] else np.nan
                )

                avg_fusion = (
                    np.mean(stats["w_fusion_scores"])
                    if stats["w_fusion_scores"] else np.nan
                )

                selection_freq = stats["selected_count"] / stats["total_count"]

                avg_rank = (
                    stats["importance_rank"] / stats["selected_count"]
                    if stats["selected_count"] > 0 else np.nan
                )

                writer_agg.writerow([
                    m_id,
                    stats["point_idx"],
                    x,
                    y,
                    z,
                    side,
                    avg_center,
                    avg_fusion,
                    selection_freq,
                    avg_rank,
                    stats["ranks"],
                    stats["selected_count"],
                    stats["total_count"]
                ])

        csv_file.close()
        csv_file_removal.close()
        csv_file_selection.close()
        csv_file_agg.close()


    def cluster_to_patch(self, cluster):

        # Support both cluster format and raw point format
        x = float(cluster.get("x", cluster.get("x_offset")))
        y = float(cluster.get("y", cluster.get("y_offset")))
        z = float(cluster.get("z", cluster.get("z_offset")))
        side = cluster["side"]

        # --- Avoid placing patch inside / behind vehicle ---
        # Rule: (x >= 0 and z <= 1.6 and y < 0.2) --> invalid
        if (x >= 0 and z <= 1.6 and np.abs(y) < 0.2):

            # Shift patch outward depending on side
            if side == "right":
                y = max(y, 0.2)
            else:
                y = min(y, -0.2)


        return {
            "x_offset": x,
            "y_offset": y,
            "z_offset": z,
            "patch_size": 0.4,
            "side": side,
            "stabilize_anchor": False
        }
    
    def check_overlap(self, existing_patches, new_patch, threshold=0.4):
        for p in existing_patches:
            if p["side"] != new_patch["side"]:
                continue
            
            dx = np.absolute(p["x_offset"] - new_patch["x_offset"])
            dy = np.absolute(p["y_offset"] - new_patch["y_offset"])
            dz = np.absolute(p["z_offset"] - new_patch["z_offset"])


            if dy < threshold or dz < threshold or dx < 0.1:
                return True

        return False
    

    def final_patch_selection(self, csv_path="point_aggregate_selection.csv", max_patches=5):

        # Group points by set_id
        sets = defaultdict(list)

        with open(csv_path) as f:
            reader = csv.DictReader(f)

            for row in reader:

                set_id = int(row["set_id"])

                sets[set_id].append({
                    "x_offset": float(row["x_offset"]),
                    "y_offset": float(row["y_offset"]),
                    "z_offset": float(row["z_offset"]),
                    "side": row["side"],
                    "selection_freq": float(row["selection_frequency"]),
                    "avg_rank": float(row["avg_rank"]) if row["avg_rank"] != "nan" else np.inf
                })

        # CSV logging
        csv_file = open("final_patch_results.csv", "w", newline="")
        writer = csv.writer(csv_file)

        writer.writerow([
            "set_id",
            "num_patches",
            "distance",
            "center_score",
            "fusion_score",
            "patches"
        ])

        all_results = []

        for set_id, points in sets.items():

            print(f"\n\n===== Processing set {set_id} =====")

            # Sort points by selection frequency and then by average rank
            points.sort(
                key=lambda x: (-x["selection_freq"], -x["avg_rank"])
            )

            # Generate configs for this set
            configs = []

            for a in range(1, max_patches + 1):

                patches = []

                for p in points:

                    cur_patch = self.cluster_to_patch(p)

                    if len(patches) == 0:
                        patches.append(cur_patch)
                        if len(patches) >= a:
                            break
                        continue

                    if not self.check_overlap(patches, cur_patch, threshold=0.4):
                        patches.append(cur_patch)

                    if len(patches) >= a:
                        break

                if len(patches) == a:
                    configs.append({
                        "set_id": set_id,
                        "num_patches": a,
                        "patches": patches
                    })
                else:
                    print(f"[WARN] set {set_id}: could only build {len(patches)}/{a} patches")

            # Evaluate configs

            for d in DISTANCES:

                print(f"\n===== Distance {d}m =====")

                # Reset world ONCE per distance
                self.reset_ego(distance=d)
                time.sleep(5)
                self.tick_n(10)

                for config in configs:

                    patches = config["patches"]
                    a = config["num_patches"]

                    print(f"Evaluating {a} patches @ {d}m")

                    # Apply attack
                    self.attack_node.set_config(patches, enabled=True)
                    rclpy.spin_once(self.attack_node)

                    center_exist, _, fusion_exist, _ = self.init_and_calculate_L_score()

                    writer.writerow([
                        set_id,
                        a,
                        d,
                        center_exist,
                        fusion_exist,
                        patches
                    ])

                    # Store per-distance result 
                    all_results.append({
                        "set_id": set_id,
                        "distance": d,
                        "num_patches": a,
                        "patches": patches,
                        "center_score": center_exist,
                        "fusion_score": fusion_exist
                    })

        csv_file.close()

        # Global best across ALL sets
        best = min(all_results, key=lambda x: x["center_score"])

        print("\n===== FINAL BEST CONFIG (GLOBAL) =====")
        print(f"Set ID: {best['set_id']}")
        print(f"Patches: {best['num_patches']}")
        print(f"Center score: {best['center_score']}")
        print(f"Fusion score: {best['fusion_score']}")

        return best, all_results
    
    
    def load_and_select_best_sets(self, csv_path, top_k_per_n=2):

        # (set_id, num_patches) → accumulate scores
        results = defaultdict(lambda: {
            "center": [],
            "fusion": [],
            "patches": None
        })

        # ---- Load CSV ----
        with open(csv_path, "r") as f:
            reader = csv.DictReader(f)

            for row in reader:

                set_id = int(row["set_id"])
                num_patches = int(row["num_patches"])

                key = (set_id, num_patches)

                center = float(row["center_score"])
                fusion = float(row["fusion_score"])

                results[key]["center"].append(center)
                results[key]["fusion"].append(fusion)

                if results[key]["patches"] is None:
                    try:
                        results[key]["patches"] = ast.literal_eval(row["patches"])
                    except Exception:
                        results[key]["patches"] = row["patches"]

        # ---- Aggregate per (set_id, num_patches) ----
        aggregated = []

        for (set_id, num_patches), vals in results.items():

            aggregated.append({
                "set_id": set_id,
                "num_patches": num_patches,
                "center_score": float(np.sum(vals["center"])),
                "fusion_score": float(np.sum(vals["fusion"])),
                "patches": vals["patches"]
            })

        # ---- Group by num_patches ----
        grouped = defaultdict(list)

        for entry in aggregated:
            grouped[entry["num_patches"]].append(entry)

        # ---- Select top-k per num_patches ----
        best_per_n = {}

        for n, entries in grouped.items():

            # lower is better 
            entries.sort(key=lambda x: (x["center_score"], x["fusion_score"]))

            best_per_n[n] = entries[:top_k_per_n]

        # ---- Print summary ----
        print("\n===== BEST SETS PER NUMBER OF PATCHES =====")

        for n in sorted(best_per_n.keys()):
            print(f"\n--- {n} patches ---")

            for rank, e in enumerate(best_per_n[n], 1):
                print(
                    f"#{rank} | set_id={e['set_id']} "
                    f"| center={e['center_score']:.4f} "
                    f"| fusion={e['fusion_score']:.4f}"
                )

        return best_per_n
    

    
    def run_location_probing(self):

        probe_sets, aggregate_scores_center, aggregate_scores_fusion = self.location_probing()

        best_sets = self.select_best_probe_sets(
            probe_sets,
            aggregate_scores_center,  # only look at center here
            aggregate_scores_fusion
        )

        return best_sets
    

    #Helper functions to resume individual pipeline steps from csv's
    def load_best_probe_sets_from_csv(self, csv_path="location_probing_results.csv"):

        probe_sets = {}
        aggregate_scores_center = {}
        aggregate_scores_fusion = {}

        with open(csv_path) as f:
            reader = csv.DictReader(f)

            for row in reader:

                probe_id = int(row["probe_id"])
                center_score = float(row["center_exist_prob"])
                fusion_score = float(row["fusion_exist_prob"])

                points = ast.literal_eval(row["points"])

                probe_sets[probe_id] = points

                aggregate_scores_center.setdefault(probe_id, 0.0)
                aggregate_scores_center[probe_id] += center_score

                aggregate_scores_fusion.setdefault(probe_id, 0.0)
                aggregate_scores_fusion[probe_id] += fusion_score

        best_sets = self.select_best_probe_sets(probe_sets, aggregate_scores_center, aggregate_scores_fusion) #focus on center here

        return best_sets
    
    def load_updated_sets_from_csv(self, csv_path="point_removal_updated_sets.csv"):

        sets = {}

        with open(csv_path) as f:
            reader = csv.DictReader(f)

            for row in reader:

                set_id = int(row["set_id"])

                patch = {
                    "x_offset": float(row["x_offset"]),
                    "y_offset": float(row["y_offset"]),
                    "z_offset": float(row["z_offset"]),
                    "patch_size": 0.0,
                    "side": row["side"],
                    "stabilize_anchor": False
                }

                sets.setdefault(set_id, []).append(patch)

        updated_sets = list(sets.values())

        print(f"\nLoaded {len(updated_sets)} updated sets from CSV")

        return updated_sets
    
        
def main():
    runner = LocationSearchRunner()

    #Location probing
    best_sets = runner.run_location_probing() #run step
    #best_sets = runner.load_best_probe_sets_from_csv() #reuse existing csv

    runner.point_removal(best_sets)
    
    final_best, all_results = runner.final_patch_selection(csv_path="point_aggregate_selection.csv", max_patches=5)

    runner.load_and_select_best_sets(csv_path="final_patch_results.csv", top_k_per_n=2)


if __name__ == "__main__":
    main()

