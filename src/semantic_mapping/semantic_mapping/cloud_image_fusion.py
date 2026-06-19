import numpy as np
import scipy.ndimage
from scipy.spatial.transform import Rotation
import cv2
import scipy
from scipy.stats import gaussian_kde, kurtosis, skew
from scipy.signal import find_peaks
from sklearn.mixture import GaussianMixture
from scipy.ndimage import minimum_filter
import os

import time

def scan2pixels(laserCloud, L2C_PARA, CAMERA_PARA, LIDAR_PARA):
    lidarX = L2C_PARA["x"] #   lidarXStack[imageIDPointer]
    lidarY = L2C_PARA["y"] # idarYStack[imageIDPointer]
    lidarZ = L2C_PARA["z"] # lidarZStack[imageIDPointer]
    lidarRoll = -L2C_PARA["roll"] #  lidarRollStack[imageIDPointer]
    lidarPitch = -L2C_PARA["pitch"] # lidarPitchStack[imageIDPointer]
    lidarYaw = -L2C_PARA["yaw"]# lidarYawStack[imageIDPointer]

    imageWidth = CAMERA_PARA["width"]
    imageHeight = CAMERA_PARA["height"]
    cameraOffsetZ = 0   #  additional pixel offset due to image cropping? 
    vertPixelOffset = 0  #  additional vertical pixel offset due to image cropping

    sinLidarRoll = np.sin(lidarRoll)
    cosLidarRoll = np.cos(lidarRoll)
    sinLidarPitch = np.sin(lidarPitch)
    cosLidarPitch = np.cos(lidarPitch)
    sinLidarYaw = np.sin(lidarYaw)
    cosLidarYaw = np.cos(lidarYaw)
    
    lidar_offset = np.array([lidarX, lidarY, lidarZ])
    camera_offset = np.array([0, 0, cameraOffsetZ])
    
    cloud = laserCloud[:, :3] - lidar_offset
    R_z = np.array([[cosLidarYaw, -sinLidarYaw, 0], [sinLidarYaw, cosLidarYaw, 0], [0, 0, 1]])
    R_y = np.array([[cosLidarPitch, 0, sinLidarPitch], [0, 1, 0], [-sinLidarPitch, 0, cosLidarPitch]])
    R_x = np.array([[1, 0, 0], [0, cosLidarRoll, -sinLidarRoll], [0, sinLidarRoll, cosLidarRoll]])
    cloud = cloud @ R_z @ R_y @ R_x
    cloud = cloud - camera_offset
    
    horiDis = np.sqrt(cloud[:, 0] ** 2 + cloud[:, 1] ** 2)
    horiPixelID = (-imageWidth / (2 * np.pi) * np.arctan2(cloud[:, 1], cloud[:, 0]) + imageWidth / 2 + 1).astype(int) - 1
    vertPixelID = (-imageWidth / (2 * np.pi) * np.arctan2(cloud[:, 2], horiDis) + imageHeight / 2 + 1 + vertPixelOffset).astype(int)
    PixelDepth = horiDis

    horiPixelID = np.clip(horiPixelID, 0, CAMERA_PARA["width"] - 1)
    vertPixelID = np.clip(vertPixelID, 0, CAMERA_PARA["height"] - 1)

    point_pixel_idx = np.array([horiPixelID, vertPixelID, PixelDepth]).T
    
    return point_pixel_idx.astype(int)

def scan2pixels_wheelchair(laserCloud):
    # project scan points to image pixels
    # https://github.com/jizhang-cmu/cmu_vla_challenge_unity/blob/noetic/src/semantic_scan_generation/src/semanticScanGeneration.cpp
    
    # Input: 
    # [#points, 3], x-y-z coordinates of lidar points
    
    # Output: 
    #    point_pixel_idx['horiPixelID'] : horizontal pixel index in the image coordinate
    #    point_pixel_idx['vertPixelID'] : vertical pixel index in the image coordinate

    # L2C_PARA= {"x": 0, "y": 0, "z": 0.235, "roll": -1.5707963, "pitch": 0, "yaw": -1.5707963} #  mapping from scan coordinate to camera coordinate(m) (degree), camera is  "z" higher than lidar
    L2C_PARA= {"x": 0, "y": 0, "z": 0.235, "roll": 0.0, "pitch": 0, "yaw": -0.0} #  mapping from scan coordinate to camera coordinate(m) (degree), camera is  "z" higher than lidar
    CAMERA_PARA= {"hfov": 360, "vfov": 120, "width": 1920, "height": 640}  # cropped 30 degree(160 pixels) in top and  30 degree(160 pixels) in bottom 
    LIDAR_PARA= {"hfov": 360, "vfov": 30}   
    
    return scan2pixels(laserCloud, L2C_PARA, CAMERA_PARA, LIDAR_PARA)

def scan2pixels_mecanum_sim(laserCloud):
    CAMERA_PARA= {"x": 0.0, "y": 0.0, "z": 0.1, "roll": -1.5707963, "pitch": 0, "yaw": -1.5707963, "hfov": 360, "vfov": 120, "width": 1920, "height": 640}  # cropped 30 degree(160 pixels) in top and  30 degree(160 pixels) in bottom 
    LIDAR_PARA= {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    lidar_offset = np.array([LIDAR_PARA["x"], LIDAR_PARA["y"], LIDAR_PARA["z"]])
    lidarRoll = LIDAR_PARA["roll"] #  lidarRollStack[imageIDPointer]
    lidarPitch = LIDAR_PARA["pitch"] # lidarPitchStack[imageIDPointer]
    lidarYaw = LIDAR_PARA["yaw"]# lidarYawStack[imageIDPointer]
    lidarR_z = np.array([[np.cos(lidarYaw), -np.sin(lidarYaw), 0], [np.sin(lidarYaw), np.cos(lidarYaw), 0], [0, 0, 1]])
    lidarR_y = np.array([[np.cos(lidarPitch), 0, np.sin(lidarPitch)], [0, 1, 0], [-np.sin(lidarPitch), 0, np.cos(lidarPitch)]])
    lidarR_x = np.array([[1, 0, 0], [0, np.cos(lidarRoll), -np.sin(lidarRoll)], [0, np.sin(lidarRoll), np.cos(lidarRoll)]])
    lidarR = lidarR_z @ lidarR_y @ lidarR_x

    cam_offset = np.array([CAMERA_PARA["x"], CAMERA_PARA["y"], CAMERA_PARA["z"]])
    camRoll = CAMERA_PARA["roll"]
    camPitch = CAMERA_PARA["pitch"]
    camYaw = CAMERA_PARA["yaw"]
    camR_z = np.array([[np.cos(camYaw), -np.sin(camYaw), 0], [np.sin(camYaw), np.cos(camYaw), 0], [0, 0, 1]])
    camR_y = np.array([[np.cos(camPitch), 0, np.sin(camPitch)], [0, 1, 0], [-np.sin(camPitch), 0, np.cos(camPitch)]])
    camR_x = np.array([[1, 0, 0], [0, np.cos(camRoll), -np.sin(camRoll)], [0, np.sin(camRoll), np.cos(camRoll)]])
    camR = camR_z @ camR_y @ camR_x

    xyz = laserCloud[:, :3] - lidar_offset
    xyz = xyz @ lidarR
    xyz = xyz - cam_offset
    xyz = xyz @ camR

    horiDis = np.sqrt(xyz[:, 0] ** 2 + xyz[:, 2] ** 2)
    horiPixelID = (CAMERA_PARA["width"] / (2 * np.pi) * np.arctan2(xyz[:, 0], xyz[:, 2]) + CAMERA_PARA["width"] / 2 + 1).astype(int)
    vertPixelID = (CAMERA_PARA["width"] / (2 * np.pi) * np.arctan(xyz[:, 1] / horiDis) + CAMERA_PARA["height"] / 2 + 1).astype(int)
    pixelDepth = horiDis

    horiPixelID = np.clip(horiPixelID, 0, CAMERA_PARA["width"] - 1)
    vertPixelID = np.clip(vertPixelID, 0, CAMERA_PARA["height"] - 1)

    # --- Step1: 构建 depth_map ---
    H = CAMERA_PARA["height"]
    W = CAMERA_PARA["width"]
    depth_map = np.full((H, W), np.inf)
    idx = vertPixelID * W + horiPixelID
    np.minimum.at(depth_map.ravel(), idx, pixelDepth)

    neighborhood = 3
    # --- Step2: 邻域 Z-buffer ---
    if neighborhood > 0:
        depth_map = minimum_filter(depth_map, size=(2*neighborhood+1), mode='nearest')

    # --- Step3: 保留最近点 ---
    remove_mask = pixelDepth >= depth_map[vertPixelID, horiPixelID] + 0.15

    # 过滤后的结果
    horiPixelID[remove_mask] = -1
    vertPixelID[remove_mask] = -1
    point_pixel_idx = np.stack([horiPixelID, vertPixelID, pixelDepth], axis=-1)

    # 根据pixelDepth对于点云进行排序，近的在前面
    sort_idx = np.argsort(point_pixel_idx[:, 2])
    point_pixel_idx = point_pixel_idx[sort_idx]
    laserCloud[:] = laserCloud[sort_idx]

    return point_pixel_idx

def scan2pixels_mecanum(laserCloud):
    CAMERA_PARA= {"x": -0.12, "y": -0.075, "z": 0.265, "roll": -1.5707963, "pitch": 0, "yaw": -1.5707963, "hfov": 360, "vfov": 120, "width": 1920, "height": 640}  # cropped 30 degree(160 pixels) in top and  30 degree(160 pixels) in bottom 
    # CAMERA_PARA= {"x": -0.12, "y": -0.075, "z": 0.265, "roll": -1.5707963, "pitch": 0, "yaw": -1.5707963, "hfov": 360, "vfov": 120, "width": 1920, "height": 480}  # cropped 30 degree(160 pixels) in top and  30 degree(160 pixels) in bottom 
    LIDAR_PARA= {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    lidar_offset = np.array([LIDAR_PARA["x"], LIDAR_PARA["y"], LIDAR_PARA["z"]])
    lidarRoll = LIDAR_PARA["roll"] #  lidarRollStack[imageIDPointer]
    lidarPitch = LIDAR_PARA["pitch"] # lidarPitchStack[imageIDPointer]
    lidarYaw = LIDAR_PARA["yaw"]# lidarYawStack[imageIDPointer]
    lidarR_z = np.array([[np.cos(lidarYaw), -np.sin(lidarYaw), 0], [np.sin(lidarYaw), np.cos(lidarYaw), 0], [0, 0, 1]])
    lidarR_y = np.array([[np.cos(lidarPitch), 0, np.sin(lidarPitch)], [0, 1, 0], [-np.sin(lidarPitch), 0, np.cos(lidarPitch)]])
    lidarR_x = np.array([[1, 0, 0], [0, np.cos(lidarRoll), -np.sin(lidarRoll)], [0, np.sin(lidarRoll), np.cos(lidarRoll)]])
    lidarR = lidarR_z @ lidarR_y @ lidarR_x

    cam_offset = np.array([CAMERA_PARA["x"], CAMERA_PARA["y"], CAMERA_PARA["z"]])
    camRoll = CAMERA_PARA["roll"]
    camPitch = CAMERA_PARA["pitch"]
    camYaw = CAMERA_PARA["yaw"]
    camR_z = np.array([[np.cos(camYaw), -np.sin(camYaw), 0], [np.sin(camYaw), np.cos(camYaw), 0], [0, 0, 1]])
    camR_y = np.array([[np.cos(camPitch), 0, np.sin(camPitch)], [0, 1, 0], [-np.sin(camPitch), 0, np.cos(camPitch)]])
    camR_x = np.array([[1, 0, 0], [0, np.cos(camRoll), -np.sin(camRoll)], [0, np.sin(camRoll), np.cos(camRoll)]])
    camR = camR_z @ camR_y @ camR_x

    xyz = laserCloud[:, :3] - lidar_offset
    xyz = xyz @ lidarR
    xyz = xyz - cam_offset
    xyz = xyz @ camR

    horiDis = np.sqrt(xyz[:, 0] ** 2 + xyz[:, 2] ** 2)
    horiPixelID = (CAMERA_PARA["width"] / (2 * np.pi) * np.arctan2(xyz[:, 0], xyz[:, 2]) + CAMERA_PARA["width"] / 2 + 1).astype(int)
    vertPixelID = (CAMERA_PARA["width"] / (2 * np.pi) * np.arctan(xyz[:, 1] / horiDis) + CAMERA_PARA["height"] / 2 + 1).astype(int)
    pixelDepth = horiDis

    horiPixelID = np.clip(horiPixelID, 0, CAMERA_PARA["width"] - 1)
    vertPixelID = np.clip(vertPixelID, 0, CAMERA_PARA["height"] - 1)

    # --- Step1: 构建 depth_map ---
    H = CAMERA_PARA["height"]
    W = CAMERA_PARA["width"]
    depth_map = np.full((H, W), np.inf)
    idx = vertPixelID * W + horiPixelID
    np.minimum.at(depth_map.ravel(), idx, pixelDepth)

    neighborhood = 3
    # --- Step2: 邻域 Z-buffer ---
    if neighborhood > 0:
        depth_map = minimum_filter(depth_map, size=(2*neighborhood+1), mode='nearest')

    # --- Step3: 保留最近点 ---
    remove_mask = pixelDepth >= depth_map[vertPixelID, horiPixelID] + 0.15

    # 过滤后的结果
    horiPixelID[remove_mask] = -1
    vertPixelID[remove_mask] = -1
    point_pixel_idx = np.stack([horiPixelID, vertPixelID, pixelDepth], axis=-1)

    # 根据pixelDepth对于点云进行排序，近的在前面
    sort_idx = np.argsort(point_pixel_idx[:, 2])
    point_pixel_idx = point_pixel_idx[sort_idx]
    laserCloud[:] = laserCloud[sort_idx]

    return point_pixel_idx

    # # --- Step3: 更新所有点的 depth ---
    # corrected_depth = depth_map[vertPixelID, horiPixelID]

    # # point_pixel_idx 对应的像素坐标 + 更新后的 depth
    # point_pixel_idx = np.stack([horiPixelID, vertPixelID, corrected_depth], axis=-1)

    # return point_pixel_idx

def scan2pixels_diablo(laserCloud):
    CAMERA_PARA= {"x": 0.0, "y": 0.0, "z": 0.185, "roll": -1.5707963, "pitch": 0, "yaw": -1.5707963, "hfov": 360, "vfov": 120, "width": 1920, "height": 640}  # cropped 30 degree(160 pixels) in top and  30 degree(160 pixels) in bottom 
    LIDAR_PARA= {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}

    lidar_offset = np.array([LIDAR_PARA["x"], LIDAR_PARA["y"], LIDAR_PARA["z"]])
    lidarRoll = LIDAR_PARA["roll"] #  lidarRollStack[imageIDPointer]
    lidarPitch = LIDAR_PARA["pitch"] # lidarPitchStack[imageIDPointer]
    lidarYaw = LIDAR_PARA["yaw"]# lidarYawStack[imageIDPointer]
    lidarR_z = np.array([[np.cos(lidarYaw), -np.sin(lidarYaw), 0], [np.sin(lidarYaw), np.cos(lidarYaw), 0], [0, 0, 1]])
    lidarR_y = np.array([[np.cos(lidarPitch), 0, np.sin(lidarPitch)], [0, 1, 0], [-np.sin(lidarPitch), 0, np.cos(lidarPitch)]])
    lidarR_x = np.array([[1, 0, 0], [0, np.cos(lidarRoll), -np.sin(lidarRoll)], [0, np.sin(lidarRoll), np.cos(lidarRoll)]])
    lidarR = lidarR_z @ lidarR_y @ lidarR_x

    cam_offset = np.array([CAMERA_PARA["x"], CAMERA_PARA["y"], CAMERA_PARA["z"]])
    camRoll = CAMERA_PARA["roll"]
    camPitch = CAMERA_PARA["pitch"]
    camYaw = CAMERA_PARA["yaw"]
    camR_z = np.array([[np.cos(camYaw), -np.sin(camYaw), 0], [np.sin(camYaw), np.cos(camYaw), 0], [0, 0, 1]])
    camR_y = np.array([[np.cos(camPitch), 0, np.sin(camPitch)], [0, 1, 0], [-np.sin(camPitch), 0, np.cos(camPitch)]])
    camR_x = np.array([[1, 0, 0], [0, np.cos(camRoll), -np.sin(camRoll)], [0, np.sin(camRoll), np.cos(camRoll)]])
    camR = camR_z @ camR_y @ camR_x

    xyz = laserCloud[:, :3] - lidar_offset
    xyz = xyz @ lidarR
    xyz = xyz - cam_offset
    xyz = xyz @ camR

    horiDis = np.sqrt(xyz[:, 0] ** 2 + xyz[:, 2] ** 2)
    horiPixelID = (CAMERA_PARA["width"] / (2 * np.pi) * np.arctan2(xyz[:, 0], xyz[:, 2]) + CAMERA_PARA["width"] / 2 + 1).astype(int)
    vertPixelID = (CAMERA_PARA["width"] / (2 * np.pi) * np.arctan(xyz[:, 1] / horiDis) + CAMERA_PARA["height"] / 2 + 1).astype(int)
    pixelDepth = horiDis

    horiPixelID = np.clip(horiPixelID, 0, CAMERA_PARA["width"] - 1)
    vertPixelID = np.clip(vertPixelID, 0, CAMERA_PARA["height"] - 1)
    point_pixel_idx = np.array([horiPixelID, vertPixelID, pixelDepth]).T
    
    return point_pixel_idx

def scan2pixels_go2w(laserCloud):
    # AlphaZ Go2-W: Mid-360 LiDAR + forward-facing pinhole camera.
    # Extrinsic from AlphaZ's static yaml at
    #   /home/all/AlphaZ/perception/src/perception_bringup/config/tf/go2w_006.yaml
    # That yaml publishes livox_frame -> front_cam_optical:
    #   t = (0.1705, 0.0262, -0.0628)
    #   q (xyzw) = (-0.4575, 0.4339, -0.5384, 0.5591)
    # Inverted to express the projection direction (livox -> camera):
    R_l2c = np.array([
        [ 0.0436, -0.9990,  0.0074],
        [ 0.2049,  0.0017, -0.9788],
        [ 0.9779,  0.0444,  0.2049],
    ])
    t_l2c = np.array([0.01921, -0.09647, -0.15504])

    # Rectified intrinsics from camera_info's P matrix (not K).
    fx = 789.61359
    fy = 797.78961
    cx = 612.8791
    cy = 358.62105
    W = 1280
    H = 720

    # Row-vector convention: p_cam = p_lidar @ R.T + t
    xyz_cam = laserCloud[:, :3] @ R_l2c.T + t_l2c
    z_cam = xyz_cam[:, 2]

    # Pinhole projection (use safe z; behind-camera points masked out below).
    behind = z_cam <= 1e-3
    z_safe = np.where(behind, 1.0, z_cam)
    u = fx * xyz_cam[:, 0] / z_safe + cx
    v = fy * xyz_cam[:, 1] / z_safe + cy

    horiPixelID = u.astype(int)
    vertPixelID = v.astype(int)
    pixelDepth = z_cam.astype(float).copy()

    out_of_image = (horiPixelID < 0) | (horiPixelID >= W) | (vertPixelID < 0) | (vertPixelID >= H)
    invalid = behind | out_of_image
    horiPixelID[invalid] = -1
    vertPixelID[invalid] = -1
    pixelDepth[invalid] = np.inf  # push invalid points to end of depth-sort

    # Z-buffer occlusion (same approach as scan2pixels_mecanum).
    valid_mask = ~invalid
    if np.any(valid_mask):
        depth_map = np.full((H, W), np.inf)
        idx = vertPixelID[valid_mask] * W + horiPixelID[valid_mask]
        np.minimum.at(depth_map.ravel(), idx, pixelDepth[valid_mask])

        neighborhood = 3
        depth_map = minimum_filter(depth_map, size=(2 * neighborhood + 1), mode='nearest')

        depth_at_pixel = np.full(pixelDepth.shape, np.inf)
        depth_at_pixel[valid_mask] = depth_map[vertPixelID[valid_mask], horiPixelID[valid_mask]]
        occluded = (pixelDepth >= depth_at_pixel + 0.15) & valid_mask
        horiPixelID[occluded] = -1
        vertPixelID[occluded] = -1

    point_pixel_idx = np.stack([horiPixelID, vertPixelID, pixelDepth], axis=-1)

    # Sort by depth ascending; mirror the permutation on laserCloud so caller's parallel-array indexing matches.
    sort_idx = np.argsort(point_pixel_idx[:, 2])
    point_pixel_idx = point_pixel_idx[sort_idx]
    laserCloud[:] = laserCloud[sort_idx]

    return point_pixel_idx

def scan2pixels_go2w_bag(laserCloud):
    # Direct LIO (no arise_slam). The cloud reaching generate_seg_cloud is in the
    # body frame of /state_estimation (the robot base), so the projection
    # extrinsic is base -> camera-optical. NOTE: superseded by the topic-driven
    # _scan2pixels_calibrated (camera_info + tf); kept for reference only.
    #
    # Extrinsic from the bag's /tf_static chain (go2w_005):
    #   base -> front_cam:    t=(0.3271, 0, 0.0430), q=identity
    #   front_cam -> front_cam_ar (optical): q(xyzw)=(-0.5, 0.4996, -0.5, 0.5004)
    # Composed and inverted to the projection direction p_cam = p_base @ R.T + t:
    R_l2c = np.array([
        [ 0.000800, -1.000000,  0.000000],
        [ 0.000800,  0.000000, -1.000000],
        [ 0.999999,  0.000800,  0.000800],
    ])
    t_l2c = np.array([-0.000262, 0.042738, -0.327134])

    # Intrinsics from /go2w_005/camera/camera_info (K == P; image is raw/distorted).
    fx = 806.0578
    fy = 805.5558
    cx = 632.4743
    cy = 346.8795
    W = 1280
    H = 720
    # plumb_bob distortion [k1, k2, p1, p2, k3]; NON-trivial, so it must be
    # applied — the image_raw the detector runs on is unrectified.
    k1, k2, p1, p2, k3 = -0.3899, 0.17881, 0.0002, -0.00068, -0.04416

    # Row-vector convention: p_cam = p_base @ R.T + t
    xyz_cam = laserCloud[:, :3] @ R_l2c.T + t_l2c
    z_cam = xyz_cam[:, 2]

    behind = z_cam <= 1e-3
    z_safe = np.where(behind, 1.0, z_cam)
    # Normalised image-plane coords, then plumb_bob (radtan) distortion.
    xn = xyz_cam[:, 0] / z_safe
    yn = xyz_cam[:, 1] / z_safe
    r2 = xn * xn + yn * yn
    radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
    x_d = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
    y_d = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
    u = fx * x_d + cx
    v = fy * y_d + cy

    horiPixelID = u.astype(int)
    vertPixelID = v.astype(int)
    pixelDepth = z_cam.astype(float).copy()

    out_of_image = (horiPixelID < 0) | (horiPixelID >= W) | (vertPixelID < 0) | (vertPixelID >= H)
    invalid = behind | out_of_image
    horiPixelID[invalid] = -1
    vertPixelID[invalid] = -1
    pixelDepth[invalid] = np.inf  # push invalid points to end of depth-sort

    # Z-buffer occlusion (same approach as scan2pixels_go2w).
    valid_mask = ~invalid
    if np.any(valid_mask):
        depth_map = np.full((H, W), np.inf)
        idx = vertPixelID[valid_mask] * W + horiPixelID[valid_mask]
        np.minimum.at(depth_map.ravel(), idx, pixelDepth[valid_mask])

        neighborhood = 3
        depth_map = minimum_filter(depth_map, size=(2 * neighborhood + 1), mode='nearest')

        depth_at_pixel = np.full(pixelDepth.shape, np.inf)
        depth_at_pixel[valid_mask] = depth_map[vertPixelID[valid_mask], horiPixelID[valid_mask]]
        occluded = (pixelDepth >= depth_at_pixel + 0.15) & valid_mask
        horiPixelID[occluded] = -1
        vertPixelID[occluded] = -1

    point_pixel_idx = np.stack([horiPixelID, vertPixelID, pixelDepth], axis=-1)

    sort_idx = np.argsort(point_pixel_idx[:, 2])
    point_pixel_idx = point_pixel_idx[sort_idx]
    laserCloud[:] = laserCloud[sort_idx]

    return point_pixel_idx

def scan2pixels_scannet(cloud):
    rgb_intrinsics = {
        'fx': 1169.621094,
        'fy': 1167.105103,
        'cx': 646.295044,
        'cy': 489.927032,
    }

    rgb_width = 1296
    rgb_height = 968

    x = cloud[:, 0]
    y = cloud[:, 1]
    x_rgb = x * rgb_intrinsics['fx'] / (cloud[:, 2] + 1e-6) + rgb_intrinsics['cx']
    y_rgb = y * rgb_intrinsics['fy'] / (cloud[:, 2] + 1e-6) + rgb_intrinsics['cy']

    point_pixel_idx = np.array([y_rgb, x_rgb, cloud[:, 2]]).T
    return point_pixel_idx

def grow_cluster_from_min(points, threshold=0.3):
    """
    从最小值点开始，区域生长聚类
    Args:
        points: numpy.ndarray, shape (N, D)，N个D维点
        threshold: float，最大距离阈值
    Returns:
        cluster_idx: numpy.ndarray, shape (N,) 的bool数组，True表示属于该簇
    """
    points = np.asarray(points)
    N = len(points)
    if N == 0:
        return np.array([], dtype=bool)

    # 找到最小值点（按第一维来找）
    min_idx = np.argmin(points[:, 0]) if points.ndim > 1 else np.argmin(points)
    cluster_idx = np.zeros(N, dtype=bool)
    cluster_idx[min_idx] = True

    changed = True
    while changed:
        changed = False
        # 当前簇里的点
        cluster_points = points[cluster_idx]
        # 簇外的点
        outside_idx = np.where(~cluster_idx)[0]
        if len(outside_idx) == 0:
            break

        # 计算簇外点到簇的最小距离
        dists = np.min(np.linalg.norm(points[outside_idx, None, :] - cluster_points[None, :, :], axis=2), axis=1)

        # 满足阈值的点加入簇
        new_points = outside_idx[dists < threshold]
        if len(new_points) > 0:
            cluster_idx[new_points] = True
            changed = True

    return cluster_idx

class CloudImageFusion:
    def __init__(self, platform):
        self.platform_list = ['wheelchair', 'mecanum', 'mecanum_bagfile', 'mecanum_sim', 'scannet', 'diablo', 'go2w', 'go2w_bag']

        if platform not in self.platform_list:
            raise ValueError(f"Invalid platform: {platform}. Available platforms: {self.platform_list}")
        else:
            self.platform = platform
            # self.scan2pixels = eval(f"scan2pixels_{platform}")

        # Runtime calibration (intrinsics + extrinsic) sourced from camera_info +
        # tf for platforms that use the topic-driven projection. Set via
        # set_calibration() before any projection; None until then.
        self.calib = None

        if platform == 'wheelchair':
            self.scan2pixels = scan2pixels_wheelchair
        elif platform == 'mecanum' or platform == 'mecanum_bagfile':
            self.scan2pixels = scan2pixels_mecanum
        elif platform == 'mecanum_sim':
            self.scan2pixels = scan2pixels_mecanum_sim
        elif platform == 'scannet':
            self.scan2pixels = scan2pixels_scannet
        elif platform == 'diablo':
            self.scan2pixels = scan2pixels_diablo
        elif platform == 'go2w':
            self.scan2pixels = scan2pixels_go2w
        elif platform == 'go2w_bag':
            # Calibration (rectified intrinsics + base->camera extrinsic) comes
            # from camera_info + tf at runtime; see set_calibration().
            self.scan2pixels = self._scan2pixels_calibrated
        else:
            print(f"Invalid platform: {platform}. Available platforms: [wheelchair, mecanum, mecanum_bagfile, mecanum_sim, scannet, diablo, go2w]")
            raise ValueError
    
    def set_calibration(self, R_l2c, t_l2c, fx, fy, cx, cy, width, height, dist=None):
        """Store the runtime camera calibration used by the topic-driven
        projection (_scan2pixels_calibrated). R_l2c/t_l2c map a point from the
        cloud's body frame to the camera-optical frame (p_cam = R_l2c @ p_body +
        t_l2c). fx/fy/cx/cy are the projection intrinsics (the P matrix for a
        rectified image). dist is the plumb_bob [k1,k2,p1,p2,k3] for a raw image,
        or None for a rectified image (no distortion applied)."""
        self.calib = {
            'R_l2c': np.asarray(R_l2c, dtype=float).reshape(3, 3),
            't_l2c': np.asarray(t_l2c, dtype=float).reshape(3),
            'fx': float(fx), 'fy': float(fy), 'cx': float(cx), 'cy': float(cy),
            'width': int(width), 'height': int(height),
            'dist': None if dist is None else np.asarray(dist, dtype=float).reshape(5),
        }

    def _scan2pixels_calibrated(self, laserCloud):
        """Pinhole projection using the runtime calibration set via
        set_calibration(). Mirrors scan2pixels_go2w (z-buffer occlusion + depth
        sort) but reads intrinsics/extrinsic from camera_info + tf instead of
        hardcoded constants. Applies plumb_bob distortion only when dist is set
        (raw image); rectified images pass dist=None."""
        c = self.calib
        if c is None:
            raise RuntimeError(
                "CloudImageFusion.set_calibration() must be called before "
                "projection (camera_info + tf not yet available?)")
        R_l2c, t_l2c = c['R_l2c'], c['t_l2c']
        fx, fy, cx, cy = c['fx'], c['fy'], c['cx'], c['cy']
        W, H, dist = c['width'], c['height'], c['dist']

        xyz_cam = laserCloud[:, :3] @ R_l2c.T + t_l2c
        z_cam = xyz_cam[:, 2]
        behind = z_cam <= 1e-3
        z_safe = np.where(behind, 1.0, z_cam)
        xn = xyz_cam[:, 0] / z_safe
        yn = xyz_cam[:, 1] / z_safe
        if dist is not None:
            k1, k2, p1, p2, k3 = dist
            r2 = xn * xn + yn * yn
            radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
            x_d = xn * radial + 2.0 * p1 * xn * yn + p2 * (r2 + 2.0 * xn * xn)
            y_d = yn * radial + p1 * (r2 + 2.0 * yn * yn) + 2.0 * p2 * xn * yn
        else:
            x_d, y_d = xn, yn
        u = fx * x_d + cx
        v = fy * y_d + cy

        horiPixelID = u.astype(int)
        vertPixelID = v.astype(int)
        pixelDepth = z_cam.astype(float).copy()

        out_of_image = (horiPixelID < 0) | (horiPixelID >= W) | (vertPixelID < 0) | (vertPixelID >= H)
        invalid = behind | out_of_image
        horiPixelID[invalid] = -1
        vertPixelID[invalid] = -1
        pixelDepth[invalid] = np.inf

        valid_mask = ~invalid
        if np.any(valid_mask):
            depth_map = np.full((H, W), np.inf)
            idx = vertPixelID[valid_mask] * W + horiPixelID[valid_mask]
            np.minimum.at(depth_map.ravel(), idx, pixelDepth[valid_mask])
            neighborhood = 3
            depth_map = minimum_filter(depth_map, size=(2 * neighborhood + 1), mode='nearest')
            depth_at_pixel = np.full(pixelDepth.shape, np.inf)
            depth_at_pixel[valid_mask] = depth_map[vertPixelID[valid_mask], horiPixelID[valid_mask]]
            occluded = (pixelDepth >= depth_at_pixel + 0.15) & valid_mask
            horiPixelID[occluded] = -1
            vertPixelID[occluded] = -1

        point_pixel_idx = np.stack([horiPixelID, vertPixelID, pixelDepth], axis=-1)
        sort_idx = np.argsort(point_pixel_idx[:, 2])
        point_pixel_idx = point_pixel_idx[sort_idx]
        laserCloud[:] = laserCloud[sort_idx]
        return point_pixel_idx

    def generate_seg_cloud(self, cloud: np.ndarray, masks, labels, confidences, R_b2w, t_b2w, image_src=None):
        # Project the cloud points to image pixels. `cloud` is in the body frame
        # of /state_estimation and is lifted to world via R_b2w/t_b2w below. No
        # gravity rotation: the stack is standardized on direct LIO, so the body
        # frame is already the bag's frame.
        point_pixel_idx = self.scan2pixels(cloud) # [N, 3] array of pixel coordinates (x, y, depth)

        if masks is None or len(masks) == 0:
            return None, None
        
        image_shape = masks[0].shape
        
        out_of_bound_filter = (point_pixel_idx[:, 0] >= 0) & \
                            (point_pixel_idx[:, 0] < image_shape[1]) & \
                            (point_pixel_idx[:, 1] >= 0) & \
                            (point_pixel_idx[:, 1] < image_shape[0])

        point_pixel_idx = point_pixel_idx[out_of_bound_filter]
        cloud = cloud[out_of_bound_filter]
        
        horDis = point_pixel_idx[:, 2] 
        point_pixel_idx = point_pixel_idx.astype(int)

        all_obj_cloud_mask = np.zeros(cloud.shape[0], dtype=bool)
        all_obj_cloud_mask_ori = np.zeros(cloud.shape[0], dtype=bool)
        obj_cloud_world_list = []
        for i in range(len(labels)):
            obj_mask = masks[i]
            cloud_mask = obj_mask[point_pixel_idx[:, 1], point_pixel_idx[:, 0]].astype(bool)

            obj_depth = horDis[cloud_mask].reshape(-1, 1)
            obj_cloud = cloud[cloud_mask]

            if obj_depth.shape[0] <=1:
                obj_cloud_world = obj_cloud[:, :3] @ R_b2w.T + t_b2w
                obj_cloud_world_list.append(obj_cloud_world)
                continue
            # 错位相减obj_depth
            obj_depth_diff = (obj_depth[1:] - obj_depth[:-1]).squeeze()
            obj_depth_max = np.max(obj_depth_diff)
            # print(f"{obj_depth_diff}")
            # print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! {obj_depth_max}")
            # min_depth = np.min(obj_depth)
            # max_depth = np.max(obj_depth)
            # count = len(obj_depth > (min_depth + max_depth) / 2)
            # if obj_depth_max > 0.5 and len(obj_depth) > 5:
            if obj_depth_max > 0.3:
                # print(f"Object {i} has large depth variation: {obj_depth_max}, len: {len(obj_depth)}")
                # from sklearn.cluster import DBSCAN
                # db = DBSCAN(eps=0.2, min_samples=5).fit(obj_depth)
                # labels = db.labels_

                # # 找到最小值点（按第一维比较）
                # min_idx = np.argmin(obj_depth[:, 0]) if obj_depth.ndim > 1 else np.argmin(obj_depth)
                # min_label = labels[min_idx]
                # cluster_mask = (labels == min_label)
                # obj_cloud = obj_cloud[cluster_mask]

                # i = 0            
                # for j in range(len(cloud_mask)):
                #     if cloud_mask[j] == False:
                #         continue
                #     else:
                #         if cluster_mask[i] == False:
                #             cloud_mask[j] = False
                #         i += 1
                # all_obj_cloud_mask = np.logical_or(all_obj_cloud_mask, cloud_mask)

                # # ----------------------------------------------------------------------------
                # idx_tmp = grow_cluster_from_min(obj_depth.reshape(-1, 1), threshold=0.3)
                # obj_cloud = obj_cloud[idx_tmp]
                # # ----------------------------------------------------------------------------

                # ----------------------------------------------------------------------------
                # only keep the idx before the largest jump
                idx_tmp = np.ones(len(obj_depth), dtype=bool)
                jump_idx = np.argmax(obj_depth_diff)
                idx_tmp[jump_idx+1:] = False
                obj_cloud = obj_cloud[idx_tmp]

                # 用 idx_tmp 过滤 cloud_mask
                filtered_mask = cloud_mask.copy()
                filtered_mask[np.where(cloud_mask)[0][~idx_tmp]] = False
                # i = 0            
                # for j in range(len(cloud_mask)):
                #     if cloud_mask[j] == False:
                #         continue
                #     else:
                #         if idx_tmp[i] == False:
                #             cloud_mask[j] = False
                #         i += 1
                all_obj_cloud_mask = np.logical_or(all_obj_cloud_mask, filtered_mask)
                all_obj_cloud_mask_ori = np.logical_or(all_obj_cloud_mask_ori, cloud_mask)
            else:
                all_obj_cloud_mask = np.logical_or(all_obj_cloud_mask, cloud_mask)
                all_obj_cloud_mask_ori = np.logical_or(all_obj_cloud_mask_ori, cloud_mask)
                # obj_cloud_list.append(obj_cloud)
        
            obj_cloud_world = obj_cloud[:, :3] @ R_b2w.T + t_b2w
            obj_cloud_world_list.append(obj_cloud_world)

        # if image_src is not None:
        #     # all_obj_cloud = cloud
        #     # all_obj_point_pixel_idx = point_pixel_idx
        #     # horDis = horDis
        #     all_obj_cloud_mask = np.ones_like(all_obj_cloud_mask, dtype=bool)
        #     time1 = int(round(time.time() * 1000))

        #     all_obj_cloud = cloud[all_obj_cloud_mask]
        #     all_obj_point_pixel_idx = point_pixel_idx[all_obj_cloud_mask]
        #     horDis_tmp = horDis[all_obj_cloud_mask]
        #     maxRange = 6.0
        #     pixelVal = np.clip(255 * horDis_tmp / maxRange, 0, 255).astype(np.uint8)
        #     image_src[all_obj_point_pixel_idx[:, 1], all_obj_point_pixel_idx[:, 0]] = np.array([pixelVal, 255-pixelVal, np.zeros_like(pixelVal)]).T # assume RGB
        #     # image_src[all_obj_point_pixel_idx[:, 1], all_obj_point_pixel_idx[:, 0]] = np.array([np.zeros_like(horDis_tmp), horDis_tmp*10, np.zeros_like(horDis_tmp)]).T # assume RGB, gray image
        #     cv2.imwrite(f"debug_obj/debug_all_obj_points_{time1}_1.png", image_src)


        if image_src is not None:
            # Visualize ALL point cloud points projected on image (not just object points)
            all_obj_cloud = cloud
            all_obj_point_pixel_idx = point_pixel_idx
            horDis = horDis
            maxRange = 8.0
            pixelVal = np.clip(255 * horDis / maxRange, 0, 255).astype(np.uint8)
            
            # Create color map: close points = red, far points = blue
            colors = np.stack([pixelVal, 255 - pixelVal, np.zeros_like(pixelVal)], axis=1)
            
            # Draw each point as a small circle
            for coords, color in zip(point_pixel_idx, colors):
                x, y = coords[:2]
                cv2.circle(image_src, (int(x), int(y)), radius=1, 
                          color=tuple(int(c) for c in color), thickness=-1)
            time1 = int(round(time.time() * 1000))
            os.makedirs("debug/img_lidar", exist_ok=True)
            out_path = f"debug/img_lidar/debug_all_obj_points_{time1}_1.png"
            ok = cv2.imwrite(out_path, image_src)
            print(f"[generate_seg_cloud] imwrite ok={ok} path={os.path.abspath(out_path)} "
                  f"img shape={None if image_src is None else image_src.shape} "
                  f"dtype={None if image_src is None else image_src.dtype}")
        
        return obj_cloud_world_list

    # @profile
    def generate_seg_cloud_v2(self, cloud: np.ndarray, masks, labels, confidences, R_b2w, t_b2w, image_src=None):
        point_pixel_idx = self.scan2pixels(cloud)

        if masks is None:
            return None, None
        
        image_shape = masks[0].shape
        
        out_of_bound_filter = (point_pixel_idx[:, 0] >= 0) & \
                            (point_pixel_idx[:, 0] < image_shape[1]) & \
                            (point_pixel_idx[:, 1] >= 0) & \
                            (point_pixel_idx[:, 1] < image_shape[0])

        point_pixel_idx = point_pixel_idx[out_of_bound_filter]
        cloud = cloud[out_of_bound_filter]
        
        depths = point_pixel_idx[:, 2]
        point_pixel_idx = point_pixel_idx.astype(int)

        depth_image = np.full(image_shape, np.inf, dtype=np.float32)

        import time
        start_time = time.time()

        # pixel_indices, depths = min_depth_per_pixel(point_pixel_idx[:, :2], horDis)
        # pixel_indices = np.array(pixel_indices, dtype=int)
        # pixel_indices = pixel_indices[pixel_indices[:, 0] >= 0]
        # depths = np.array(depths)

        np.minimum.at(depth_image, (point_pixel_idx[:, 1], point_pixel_idx[:, 0]), depths)
        structure = np.array([[0, 1, 0],
                            [1, 1, 1],
                            [0, 1, 0]], dtype=np.uint8)
        inflated_depth_image = scipy.ndimage.grey_dilation(depth_image, footprint=structure, mode='nearest')

        inflated_depth_image = np.minimum(inflated_depth_image, depth_image)

        print(f'pixel conversion: {time.time() - start_time} for {point_pixel_idx.shape[0]} points')
        # for i, pixel_idx in enumerate(pixel_indices):
        #     depth_image[*pixel_idx[[1, 0]].tolist()] = depths[i]
            
        # depth_image[pixel_indices[:, 1], pixel_indices[:, 0]] = depths

        valid_mask = ~np.isinf(inflated_depth_image)  # Mask for valid depth values
        if valid_mask.any():
            min_depth = inflated_depth_image[valid_mask].min()
            max_depth = inflated_depth_image[valid_mask].max()

            print(f"Min depth: {min_depth}, Max depth: {max_depth}")

            # Normalize only valid depth values
            normalized_depth = np.zeros_like(inflated_depth_image, dtype=np.uint8)
            normalized_depth[valid_mask] = 255 * (1 - (inflated_depth_image[valid_mask] - min_depth) / (max_depth - min_depth + 1e-6))
        else:
            normalized_depth = np.zeros_like(inflated_depth_image, dtype=np.uint8)  # If all values are inf, return a blank image
        
        # cv2.imshow("Depth Image", normalized_depth)
        # cv2.waitKey(1)  # Wait for a key press to close the window

        all_obj_cloud_mask = np.zeros(cloud.shape[0], dtype=bool)
        obj_cloud_world_list = []
        for i in range(len(labels)):
            obj_mask = masks[i]
            cloud_mask = obj_mask[point_pixel_idx[:, 1], point_pixel_idx[:, 0]].astype(bool)
            all_obj_cloud_mask = np.logical_or(all_obj_cloud_mask, cloud_mask)
            obj_cloud = cloud[cloud_mask]
                    
            # obj_cloud_list.append(obj_cloud)
            
            obj_cloud_world = obj_cloud[:, :3] @ R_b2w.T + t_b2w
            obj_cloud_world_list.append(obj_cloud_world)

        if image_src is not None:
            all_obj_cloud = cloud
            all_obj_point_pixel_idx = point_pixel_idx
            horDis = horDis
            # all_obj_cloud = cloud[all_obj_cloud_mask]
            # all_obj_point_pixel_idx = point_pixel_idx[all_obj_cloud_mask]
            # horDis = horDis[all_obj_cloud_mask]
            maxRange = 6.0
            pixelVal = np.clip(255 * horDis / maxRange, 0, 255).astype(np.uint8)
            image_src[all_obj_point_pixel_idx[:, 1], all_obj_point_pixel_idx[:, 0]] = np.array([pixelVal, 255-pixelVal, np.zeros_like(pixelVal)]).T # assume RGB
        
        return obj_cloud_world_list, normalized_depth
