"""Geometry helpers for the convex polygon used as the flight geofence."""

import numpy as np


DEFAULT_GEOFENCE = [3.0, -0.3, 1.0, -3.2, -5.2, 0.3, -3.0, 3.8]


def vertices_from_flat(values):
    """Validate a flat x1,y1,... list and return counter-clockwise vertices."""
    vertices = np.asarray(values, dtype=float)
    if vertices.size < 6 or vertices.size % 2:
        raise ValueError('geofence must contain at least three x,y coordinate pairs')
    vertices = vertices.reshape((-1, 2))
    area2 = np.sum(
        vertices[:, 0] * np.roll(vertices[:, 1], -1)
        - vertices[:, 1] * np.roll(vertices[:, 0], -1)
    )
    if abs(area2) < 1e-9:
        raise ValueError('geofence vertices enclose no area')
    if area2 < 0:
        vertices = vertices[::-1].copy()

    # A single half-plane test is sufficient only for a convex polygon.
    edges = np.roll(vertices, -1, axis=0) - vertices
    turns = np.cross(edges, np.roll(edges, -1, axis=0))
    if np.any(turns < -1e-9):
        raise ValueError('geofence vertices must describe a convex polygon in order')
    return vertices


def inward_normals(vertices):
    edges = np.roll(vertices, -1, axis=0) - vertices
    lengths = np.linalg.norm(edges, axis=1)
    if np.any(lengths < 1e-9):
        raise ValueError('geofence contains duplicate adjacent vertices')
    return np.column_stack((-edges[:, 1], edges[:, 0])) / lengths[:, None]


def signed_edge_distances(point, vertices):
    """Distance to each edge; non-negative means inside that edge."""
    return np.sum((np.asarray(point) - vertices) * inward_normals(vertices), axis=1)


def contains(point, vertices, margin=0.0):
    return bool(np.all(signed_edge_distances(point, vertices) >= -margin))


def project_inside(point, vertices):
    """Return the closest point in/on a convex polygon and whether it changed."""
    point = np.asarray(point, dtype=float)
    if contains(point, vertices):
        return point.copy(), False

    ends = np.roll(vertices, -1, axis=0)
    edges = ends - vertices
    t = np.sum((point - vertices) * edges, axis=1) / np.sum(edges * edges, axis=1)
    t = np.clip(t, 0.0, 1.0)
    candidates = vertices + t[:, None] * edges
    closest = candidates[np.argmin(np.sum((candidates - point) ** 2, axis=1))]
    return closest, True


def offset_vertices(vertices, margin):
    """Vertices of a convex polygon whose edges are shifted outward by margin."""
    normals = inward_normals(vertices)
    shifted = vertices - margin * normals
    result = []
    for i in range(len(vertices)):
        prev = (i - 1) % len(vertices)
        matrix = np.vstack((normals[prev], normals[i]))
        rhs = np.array((
            np.dot(normals[prev], shifted[prev]),
            np.dot(normals[i], shifted[i]),
        ))
        result.append(np.linalg.solve(matrix, rhs))
    return np.asarray(result)
