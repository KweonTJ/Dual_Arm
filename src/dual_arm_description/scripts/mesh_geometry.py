"""Closed, outward-facing triangle surfaces for RViz's single-sided rendering."""
import numpy as np


def edge_topology(mesh):
    """Weld STL coordinates at nanometre precision and index directed edges."""
    mesh = np.asarray(mesh, dtype=float)
    if mesh.ndim != 3 or mesh.shape[1:] != (3, 3) or not np.isfinite(mesh).all():
        raise ValueError('Expected finite triangles with shape (N, 3, 3)')
    _, indices = np.unique(np.round(mesh.reshape(-1, 3), 9), axis=0, return_inverse=True)
    indices = indices.reshape(-1, 3)
    edges = np.stack([indices[:, [0, 1]], indices[:, [1, 2]], indices[:, [2, 0]]], axis=1).reshape(-1, 2)
    _, inverse, counts = np.unique(np.sort(edges, axis=1), axis=0, return_inverse=True, return_counts=True)
    directions = np.where(edges[:, 0] < edges[:, 1], 1, -1)
    return inverse, counts, directions


def signed_volume(mesh):
    # A nearby origin avoids cancellation for thin, offset mounting plates.
    relative = mesh - mesh.reshape(-1, 3).mean(axis=0)
    return float(np.einsum('ij,ij->i', relative[:, 0],
                          np.cross(relative[:, 1], relative[:, 2])).sum() / 6)


def orient_solid(mesh):
    """Require closed shells, make neighbours consistent, then orient outward.

    Each input is a solid primitive; assembled pieces may touch or overlap.
    Deliberate through holes have closed bore walls and remain open passages.
    """
    mesh = np.array(mesh, dtype=float, copy=True)
    area = np.linalg.norm(np.cross(mesh[:, 1] - mesh[:, 0], mesh[:, 2] - mesh[:, 0]), axis=1)
    mesh = mesh[area > 1e-12]
    inverse, counts, directions = edge_topology(mesh)
    if not len(mesh) or np.any(counts != 2):
        raise ValueError(f'Open/non-manifold primitive: {np.count_nonzero(counts != 2)} edges')
    pairs = np.argsort(inverse, kind='stable').reshape(-1, 2)
    neighbours = [[] for _ in mesh]
    for edge_a, edge_b in pairs:
        a, b = edge_a // 3, edge_b // 3
        relative_sign = -int(directions[edge_a] * directions[edge_b])
        neighbours[a].append((b, relative_sign))
        neighbours[b].append((a, relative_sign))
    signs = np.zeros(len(mesh), dtype=np.int8)
    for start in range(len(mesh)):
        if signs[start]:
            continue
        signs[start] = 1
        pending, component = [start], []
        while pending:
            a = pending.pop()
            component.append(a)
            for b, relative_sign in neighbours[a]:
                expected = signs[a] * relative_sign
                if signs[b] and signs[b] != expected:
                    raise ValueError('Surface cannot be consistently oriented')
                if not signs[b]:
                    signs[b] = expected
                    pending.append(b)
        component = np.array(component)
        flip = component[signs[component] < 0]
        mesh[flip] = mesh[flip, ::-1]
        volume = signed_volume(mesh[component])
        if abs(volume) < 1e-20:
            raise ValueError('Closed primitive has no enclosed volume')
        if volume < 0:
            mesh[component] = mesh[component, ::-1]
    return mesh


def surface_report(mesh):
    """Touching closed solids may share edges; their edge directions must balance."""
    inverse, counts, directions = edge_topology(mesh)
    balance = np.bincount(inverse, weights=directions)
    return {
        'triangles': len(mesh),
        'boundary_edges': int(np.count_nonzero(counts == 1)),
        'unbalanced_edges': int(np.count_nonzero(balance)),
        'contact_edges': int(np.count_nonzero(counts > 2)),
        'signed_volume_m3': signed_volume(np.asarray(mesh)),
    }
