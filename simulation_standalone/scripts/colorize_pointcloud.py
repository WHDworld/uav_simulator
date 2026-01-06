#!/usr/bin/env python3
"""
Point Cloud Colorization Tool
Add colors to a PCD file based on height, normals, or custom rules.

Usage:
    python3 colorize_pointcloud.py input.pcd output.pcd --mode height
    python3 colorize_pointcloud.py input.pcd output.pcd --mode indoor
    python3 colorize_pointcloud.py input.pcd output.pcd --mode outdoor
"""

import numpy as np
import argparse
import struct
import os

def read_pcd(filename):
    """Read PCD file and return points as numpy array"""
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    # Parse header
    header = {}
    data_idx = 0
    for i, line in enumerate(lines):
        if line.startswith('DATA'):
            data_idx = i + 1
            header['data_type'] = line.split()[1]
            break
        parts = line.split()
        if len(parts) >= 2:
            header[parts[0]] = parts[1:]
    
    # Get point count and fields
    num_points = int(header['POINTS'][0])
    fields = header['FIELDS']
    
    # Read points
    if header['data_type'] == 'ascii':
        points = []
        for line in lines[data_idx:data_idx + num_points]:
            values = [float(v) for v in line.split()]
            points.append(values[:3])  # Only take XYZ
        points = np.array(points)
    else:
        raise ValueError("Binary PCD not supported. Convert to ASCII first using pcl_convert_pcd_ascii_binary")
    
    return points, header

def write_pcd_rgb(filename, points, colors):
    """Write PCD file with RGB colors"""
    num_points = len(points)
    
    with open(filename, 'w') as f:
        # Write header
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z rgb\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        
        # Write points with RGB packed into float
        for i in range(num_points):
            x, y, z = points[i]
            r, g, b = colors[i]
            # Pack RGB into a single float (PCL format)
            rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
            rgb_float = struct.unpack('f', struct.pack('I', rgb_int))[0]
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {rgb_float}\n")
    
    print(f"Saved {num_points} colored points to {filename}")

def colorize_by_height(points, z_min=None, z_max=None, colormap='viridis'):
    """Color points based on height (Z coordinate)"""
    z = points[:, 2]
    if z_min is None:
        z_min = np.min(z)
    if z_max is None:
        z_max = np.max(z)
    
    # Normalize height to [0, 1]
    z_norm = (z - z_min) / (z_max - z_min + 1e-6)
    z_norm = np.clip(z_norm, 0, 1)
    
    # Apply colormap
    if colormap == 'viridis':
        # Viridis-like colormap
        colors = np.zeros((len(points), 3))
        colors[:, 0] = 68 + 180 * z_norm  # R: purple to yellow
        colors[:, 1] = 1 + 220 * z_norm   # G: dark to bright
        colors[:, 2] = 84 - 60 * z_norm   # B: purple to yellow
    elif colormap == 'jet':
        # Jet colormap
        colors = np.zeros((len(points), 3))
        colors[:, 0] = np.clip(1.5 - np.abs(4 * z_norm - 3), 0, 1) * 255
        colors[:, 1] = np.clip(1.5 - np.abs(4 * z_norm - 2), 0, 1) * 255
        colors[:, 2] = np.clip(1.5 - np.abs(4 * z_norm - 1), 0, 1) * 255
    elif colormap == 'gray':
        intensity = (z_norm * 200 + 55).astype(np.uint8)
        colors = np.stack([intensity, intensity, intensity], axis=1)
    
    return colors.astype(np.uint8)

def colorize_indoor(points):
    """
    Color points for indoor environment:
    - Floor (z ~ 0): brown/gray
    - Walls: light gray
    - Ceiling (high z): white
    - Objects: various colors based on height
    """
    z = points[:, 2]
    z_min, z_max = np.min(z), np.max(z)
    z_range = z_max - z_min
    
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    
    # Floor (bottom 10%)
    floor_mask = z < z_min + 0.1 * z_range
    colors[floor_mask] = [139, 119, 101]  # Brown/gray floor
    
    # Ceiling (top 10%)
    ceiling_mask = z > z_max - 0.1 * z_range
    colors[ceiling_mask] = [240, 240, 245]  # Off-white ceiling
    
    # Walls and objects (middle)
    mid_mask = ~floor_mask & ~ceiling_mask
    z_norm = (z[mid_mask] - z_min) / z_range
    
    # Walls are typically gray with slight variation
    wall_colors = np.zeros((np.sum(mid_mask), 3), dtype=np.uint8)
    wall_colors[:, 0] = 180 + (z_norm * 40).astype(np.uint8)  # R
    wall_colors[:, 1] = 175 + (z_norm * 45).astype(np.uint8)  # G
    wall_colors[:, 2] = 170 + (z_norm * 50).astype(np.uint8)  # B
    colors[mid_mask] = wall_colors
    
    return colors

def colorize_outdoor(points):
    """
    Color points for outdoor environment:
    - Ground: green/brown
    - Buildings: gray
    - Vegetation: green
    - Sky facing: blue tint
    """
    z = points[:, 2]
    z_min, z_max = np.min(z), np.max(z)
    z_range = z_max - z_min
    
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    
    # Ground (bottom 5%)
    ground_mask = z < z_min + 0.05 * z_range
    # Mix of green and brown for ground
    ground_colors = np.zeros((np.sum(ground_mask), 3), dtype=np.uint8)
    rand = np.random.random(np.sum(ground_mask))
    green_mask = rand > 0.5
    ground_colors[green_mask] = [34, 139, 34]  # Forest green
    ground_colors[~green_mask] = [139, 119, 101]  # Brown
    colors[ground_mask] = ground_colors
    
    # Buildings/structures (above ground)
    struct_mask = ~ground_mask
    z_norm = (z[struct_mask] - z_min) / z_range
    
    # Gray buildings with height variation
    struct_colors = np.zeros((np.sum(struct_mask), 3), dtype=np.uint8)
    base_gray = 140 + (z_norm * 60)
    struct_colors[:, 0] = base_gray.astype(np.uint8)
    struct_colors[:, 1] = base_gray.astype(np.uint8)
    struct_colors[:, 2] = (base_gray + 10).astype(np.uint8)  # Slight blue tint
    colors[struct_mask] = struct_colors
    
    return colors

def colorize_tunnel(points):
    """
    Color points for tunnel/underground environment:
    - Dark overall with subtle variations
    """
    z = points[:, 2]
    z_min, z_max = np.min(z), np.max(z)
    z_norm = (z - z_min) / (z_max - z_min + 1e-6)
    
    colors = np.zeros((len(points), 3), dtype=np.uint8)
    
    # Dark gray/brown with height variation
    base = 80 + z_norm * 60
    colors[:, 0] = base.astype(np.uint8)
    colors[:, 1] = (base - 10).astype(np.uint8)
    colors[:, 2] = (base - 20).astype(np.uint8)
    
    return colors

def colorize_random(points, seed=42):
    """Random colors for each point (for debugging)"""
    np.random.seed(seed)
    colors = np.random.randint(50, 255, size=(len(points), 3), dtype=np.uint8)
    return colors

def main():
    parser = argparse.ArgumentParser(description='Colorize point cloud')
    parser.add_argument('input', help='Input PCD file')
    parser.add_argument('output', help='Output PCD file with colors')
    parser.add_argument('--mode', choices=['height', 'indoor', 'outdoor', 'tunnel', 'random', 'gray'],
                        default='indoor', help='Colorization mode')
    parser.add_argument('--colormap', choices=['viridis', 'jet', 'gray'],
                        default='viridis', help='Colormap for height mode')
    args = parser.parse_args()
    
    print(f"Reading {args.input}...")
    points, header = read_pcd(args.input)
    print(f"Loaded {len(points)} points")
    print(f"Z range: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
    
    print(f"Colorizing with mode: {args.mode}")
    if args.mode == 'height':
        colors = colorize_by_height(points, colormap=args.colormap)
    elif args.mode == 'indoor':
        colors = colorize_indoor(points)
    elif args.mode == 'outdoor':
        colors = colorize_outdoor(points)
    elif args.mode == 'tunnel':
        colors = colorize_tunnel(points)
    elif args.mode == 'random':
        colors = colorize_random(points)
    elif args.mode == 'gray':
        colors = colorize_by_height(points, colormap='gray')
    
    write_pcd_rgb(args.output, points, colors)
    print("Done!")

if __name__ == '__main__':
    main()

