import open3d as o3d
import numpy as np
import rosbag
import sensor_msgs.point_cloud2 as pc2
import csv
from sensor_msgs.msg import PointCloud2

def get_all_pointclouds(bag_path, topic):
    bag = rosbag.Bag(bag_path)
    for _, msg, t in bag.read_messages(topics=[topic]):
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        yield np.array(points), t
    bag.close()

def visualize_and_pick(points, frame_idx, timestamp):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    print(f"\n--- Frame {frame_idx} | Time: {timestamp} ---")
    print("Shift + Left Click to pick a point. Then press [Q] to confirm and close window.\n")

    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)

    # Zoomed-in camera setup
    ctr = vis.get_view_control()
    params = ctr.convert_to_pinhole_camera_parameters()
    extrinsic = params.extrinsic.copy()
    extrinsic[0:3, 3] = [0, 0, -1]
    params.extrinsic = extrinsic
    ctr.convert_from_pinhole_camera_parameters(params)
    ctr.set_zoom(1)

    vis.run()
    vis.destroy_window()

    picked_ids = vis.get_picked_points()
    if picked_ids:
        picked_point = points[picked_ids[0]]
        if np.all(np.isfinite(picked_point)):
            xy = picked_point[:2]
            distance_xy = np.linalg.norm(xy)
            print(f"Picked Point (x, y only): {xy}, Distance from LiDAR (xy): {distance_xy:.3f} meters")
            return timestamp.to_sec(), distance_xy, xy
        else:
            print(f"Picked Point: {picked_point} contains invalid values.")
            return timestamp.to_sec(), None, None
    else:
        print("No point picked.")
        return timestamp.to_sec(), None, None

# def main():
#     bag_path = "/home/sridhar/Downloads/rosbagnew/ag2.bag"
#     topic = "/ouster/points"
#     target_frame_idx = int(input("Enter starting frame number: "))
#     num_frames = 5

#     results = []
#     for idx, (points, t) in enumerate(get_all_pointclouds(bag_path, topic)):
#         if idx < target_frame_idx:
#             continue
#         if idx >= target_frame_idx + num_frames:
#             break

#         print(f"Frame {idx} | Timestamp: {t} | Num points: {points.shape[0]}")
#         if points.size == 0:
#             print("Empty point cloud, skipping.")
#             continue

#         timestamp_sec, distance, xy = visualize_and_pick(points, idx, t)
#         if distance is not None:
#             results.append((timestamp_sec, distance,xy[0],xy[1]))

#     # Save results to CSV
#     csv_path = "src/mp2/src/dlidar.csv"
#     with open(csv_path, "w", newline="") as csvfile:
#         writer = csv.writer(csvfile)
#         writer.writerow(["timestamp", "dlidar", "rel_x","rel_y"])
#         for row in results:
#             writer.writerow(row)

#     print(f"\nSaved {len(results)} points to {csv_path}")


#### TO CHOOSE 15 POINTS OF CHOICE #####
def main():
    bag_path = "/home/sridhar/Downloads/rosbagnew/ag2.bag"
    topic = "/ouster/points"

    # Take 15 space-separated frame indices from user
    input_str = input("Enter 15 frame numbers (space-separated): ")
    frame_indices = list(map(int, input_str.strip().split()))
    
    if len(frame_indices) != 15:
        print("Error: You must enter exactly 15 frame numbers.")
        return

    results = []
    max_index = max(frame_indices)
    frame_indices_set = set(frame_indices)

    for idx, (points, t) in enumerate(get_all_pointclouds(bag_path, topic)):
        if idx > max_index:
            break

        if idx not in frame_indices_set:
            continue

        print(f"\nFrame {idx} | Timestamp: {t} | Num points: {points.shape[0]}")
        if points.size == 0:
            print("Empty point cloud, skipping.")
            continue

        timestamp_sec, distance, xy = visualize_and_pick(points, idx, t)
        if distance is not None:
            results.append((timestamp_sec, distance, xy[0], xy[1]))

        # Remove the frame index once processed
        frame_indices_set.remove(idx)

        if not frame_indices_set:
            break  # All 15 frames processed

    # Save results to CSV
    csv_path = "src/mp2/src/dlidar.csv"
    with open(csv_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp", "dlidar", "rel_x", "rel_y"])
        for row in results:
            writer.writerow(row)

    print(f"\nSaved {len(results)} points to {csv_path}")


if __name__ == "__main__":
    main()
