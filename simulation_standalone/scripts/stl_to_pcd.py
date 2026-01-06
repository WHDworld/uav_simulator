#!/usr/bin/env python3
"""
STL to PCD Converter
Converts STL mesh files to PCD point cloud files by sampling points on the mesh surface.

Usage:
    python3 stl_to_pcd.py input.stl output.pcd --points 500000
    python3 stl_to_pcd.py input.stl output.pcd --density 1000  # points per m^2
"""

import numpy as np
import argparse
import struct
import os

def read_stl_ascii(filename):
    """Read ASCII STL file"""
    vertices = []
    normals = []
    
    with open(filename, 'r') as f:
        current_normal = None
        for line in f:
            line = line.strip()
            if line.startswith('facet normal'):
                parts = line.split()
                current_normal = [float(parts[2]), float(parts[3]), float(parts[4])]
            elif line.startswith('vertex'):
                parts = line.split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
                if current_normal:
                    normals.append(current_normal)
    
    vertices = np.array(vertices).reshape(-1, 3, 3)  # [num_triangles, 3 vertices, 3 coords]
    return vertices

def read_stl_binary(filename):
    """Read binary STL file"""
    with open(filename, 'rb') as f:
        # Skip header (80 bytes)
        f.read(80)
        # Read number of triangles
        num_triangles = struct.unpack('I', f.read(4))[0]
        
        vertices = []
        for _ in range(num_triangles):
            # Normal (12 bytes) - skip
            f.read(12)
            # 3 vertices (36 bytes)
            v1 = struct.unpack('fff', f.read(12))
            v2 = struct.unpack('fff', f.read(12))
            v3 = struct.unpack('fff', f.read(12))
            vertices.append([v1, v2, v3])
            # Attribute byte count (2 bytes) - skip
            f.read(2)
        
        return np.array(vertices)

def read_stl(filename):
    """Read STL file (auto-detect ASCII or binary)"""
    with open(filename, 'rb') as f:
        header = f.read(80)
    
    # Check if ASCII (starts with 'solid')
    try:
        header_str = header.decode('ascii').strip()
        if header_str.startswith('solid'):
            # Could be ASCII, try reading
            try:
                return read_stl_ascii(filename)
            except:
                pass
    except:
        pass
    
    # Binary
    return read_stl_binary(filename)

def sample_triangle(v0, v1, v2, num_samples):
    """Sample random points uniformly on a triangle"""
    # Generate random barycentric coordinates
    r1 = np.random.random(num_samples)
    r2 = np.random.random(num_samples)
    
    # Ensure uniform distribution
    sqrt_r1 = np.sqrt(r1)
    
    # Barycentric coordinates
    u = 1 - sqrt_r1
    v = sqrt_r1 * (1 - r2)
    w = sqrt_r1 * r2
    
    # Convert to Cartesian
    points = (u[:, np.newaxis] * v0 + 
              v[:, np.newaxis] * v1 + 
              w[:, np.newaxis] * v2)
    
    return points

def triangle_area(v0, v1, v2):
    """Calculate triangle area"""
    edge1 = v1 - v0
    edge2 = v2 - v0
    cross = np.cross(edge1, edge2)
    return 0.5 * np.linalg.norm(cross)

def sample_mesh(triangles, num_points):
    """Sample points uniformly on mesh surface"""
    # Calculate area of each triangle
    areas = np.array([triangle_area(t[0], t[1], t[2]) for t in triangles])
    total_area = np.sum(areas)
    
    # Number of samples per triangle (proportional to area)
    probs = areas / total_area
    samples_per_triangle = np.random.multinomial(num_points, probs)
    
    # Sample points
    all_points = []
    for i, (triangle, num_samples) in enumerate(zip(triangles, samples_per_triangle)):
        if num_samples > 0:
            points = sample_triangle(triangle[0], triangle[1], triangle[2], num_samples)
            all_points.append(points)
    
    return np.vstack(all_points)

def write_pcd(filename, points):
    """Write PCD file (ASCII format)"""
    num_points = len(points)
    
    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")
        
        for p in points:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
    
    print(f"Saved {num_points} points to {filename}")

def apply_transform(points, T):
    """Apply 4x4 transformation matrix to points"""
    # Add homogeneous coordinate
    ones = np.ones((len(points), 1))
    points_h = np.hstack([points, ones])
    # Apply transform
    points_transformed = (T @ points_h.T).T
    return points_transformed[:, :3]

def main():
    parser = argparse.ArgumentParser(description='Convert STL mesh to PCD point cloud')
    parser.add_argument('input', help='Input STL file')
    parser.add_argument('output', help='Output PCD file')
    parser.add_argument('--points', type=int, default=500000, 
                        help='Number of points to sample (default: 500000)')
    parser.add_argument('--density', type=float, default=None,
                        help='Point density (points per m^2, overrides --points)')
    parser.add_argument('--transform', type=str, default=None,
                        help='Transformation: "y_up" (Y-up to Z-up), or T_m_w matrix as comma-separated values')
    parser.add_argument('--center', action='store_true',
                        help='Center the point cloud at origin (XY plane)')
    args = parser.parse_args()
    
    print(f"Reading {args.input}...")
    triangles = read_stl(args.input)
    print(f"Loaded {len(triangles)} triangles")
    
    # Calculate total surface area
    areas = np.array([triangle_area(t[0], t[1], t[2]) for t in triangles])
    total_area = np.sum(areas)
    print(f"Total surface area: {total_area:.2f} m^2")
    
    # Determine number of points
    if args.density:
        num_points = int(total_area * args.density)
        print(f"Using density {args.density} points/m^2 -> {num_points} points")
    else:
        num_points = args.points
        print(f"Sampling {num_points} points")
    
    # Sample points
    print("Sampling points from mesh surface...")
    points = sample_mesh(triangles, num_points)
    
    # Print original bounds
    print(f"Original point cloud bounds:")
    print(f"  X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
    print(f"  Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
    print(f"  Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
    
    # Apply transformation if specified
    if args.transform:
        if args.transform == "y_up":
            # Y-up to Z-up: swap Y and Z, negate new Y
            # (x, y, z) -> (x, -z, y)  or similar based on convention
            # Common convention: (x, y, z)_yup -> (x, z, -y)_zup
            print("Applying Y-up to Z-up transformation...")
            T = np.array([
                [1, 0, 0, 0],
                [0, 0, -1, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 1]
            ], dtype=np.float64)
            points = apply_transform(points, T)
        else:
            # Parse custom T_m_w matrix (16 comma-separated values)
            try:
                values = [float(v) for v in args.transform.split(',')]
                if len(values) == 16:
                    T = np.array(values).reshape(4, 4)
                    print(f"Applying custom transformation matrix...")
                    points = apply_transform(points, T)
                else:
                    print(f"Warning: Invalid transform (expected 16 values, got {len(values)})")
            except:
                print(f"Warning: Could not parse transform '{args.transform}'")
    
    # Center if requested
    if args.center:
        print("Centering point cloud...")
        x_center = (points[:, 0].min() + points[:, 0].max()) / 2
        y_center = (points[:, 1].min() + points[:, 1].max()) / 2
        points[:, 0] -= x_center
        points[:, 1] -= y_center
    
    # Print final bounds
    print(f"Final point cloud bounds:")
    print(f"  X: [{points[:, 0].min():.2f}, {points[:, 0].max():.2f}]")
    print(f"  Y: [{points[:, 1].min():.2f}, {points[:, 1].max():.2f}]")
    print(f"  Z: [{points[:, 2].min():.2f}, {points[:, 2].max():.2f}]")
    
    # Write PCD
    write_pcd(args.output, points)
    print("Done!")

if __name__ == '__main__':
    main()

