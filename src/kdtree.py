import numpy as np
import heapq

class KDNode:
    def __init__(self, point, split_dim, left=None, right=None):
        self.point = point
        self.split_dim = split_dim
        self.left = left
        self.right = right

class KDTree:
    def __init__(self, points):
        self.root = self._build_kdtree(points)

    def _build_kdtree(self, points, depth=0):
        if len(points) == 0:
            return None

        # choose the axis to split on (alternates between dimensions)
        split_dim = depth % len(points[0])

        # sort the points by the split dimension
        points = points[points[:,split_dim].argsort()]

        # find the median point and split the points into two halves
        median_idx = len(points) // 2
        median_point = points[median_idx]
        left_points = points[:median_idx]
        right_points = points[median_idx+1:]

        # recursively build the left and right subtrees
        left = self._build_kdtree(left_points, depth+1)
        right = self._build_kdtree(right_points, depth+1)

        return KDNode(median_point, split_dim, left, right)

    def _search_kdtree(self, node, point,k,queue, depth=0):
        if node is None:
            return None
        dist = np.linalg.norm(node.point-point)
        if len(queue) < k or dist < -queue[0][0]:
            heapq.heappush(queue, (-dist, node.point))
            if len(queue) > k:
                heapq.heappop(queue)

        # choose the axis to search on (same as during construction)
        split_dim = node.split_dim

        if point[split_dim] < node.point[split_dim]:
            # search the left subtree if the target point is less than the current node's point
            return self._search_kdtree(node.left, point, k, queue, depth+1)
        else:
            # search the right subtree otherwise
            return self._search_kdtree(node.right, point, k, queue, depth+1)

    def query(self, point, k=1):
        queue = []
        self._search_kdtree(self.root, point, k, queue)
        return [x[1] for x in sorted(queue, reverse=True)]
