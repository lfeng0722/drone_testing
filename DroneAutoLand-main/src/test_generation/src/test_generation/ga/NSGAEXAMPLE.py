# from typing import List, Set
#
#
# def non_dominated_sorting_with_weights(solutions: List[List[float]]) -> List[List[float]]:
#     def dominates(sol1: List[float], sol2: List[float]) -> bool:
#         """Check if sol1 dominates sol2."""
#         return all(x <= y for x, y in zip(sol1, sol2)) and any(x < y for x, y in zip(sol1, sol2))
#
#     def calculate_crowding_distance(front_solutions: List[List[float]]) -> List[float]:
#         """Calculate the crowding distance of each solution in the front."""
#         if not front_solutions:
#             return []
#
#         size = len(front_solutions)
#         distances = [0.0 for _ in range(size)]
#         for m in range(len(front_solutions[0])):
#             sorted_indices = sorted(range(size), key=lambda x: front_solutions[x][m])
#             distances[sorted_indices[0]] = float('inf')
#             distances[sorted_indices[-1]] = float('inf')
#             for i in range(1, size - 1):
#                 distances[sorted_indices[i]] += (
#                             front_solutions[sorted_indices[i + 1]][m] - front_solutions[sorted_indices[i - 1]][m])
#
#         return distances
#
#     # Non-dominated sorting
#     n = len(solutions)
#     dominated_by = [set() for _ in range(n)]
#     dominates_count = [0 for _ in range(n)]
#     fronts = [[]]
#
#     for i in range(n):
#         for j in range(n):
#             if i != j:
#                 if dominates(solutions[i], solutions[j]):
#                     dominated_by[i].add(j)
#                 elif dominates(solutions[j], solutions[i]):
#                     dominates_count[i] += 1
#         if dominates_count[i] == 0:
#             fronts[0].append(i)
#
#     i = 0
#     while fronts[i]:
#         next_front = []
#         for sol in fronts[i]:
#             for dominated_sol in dominated_by[sol]:
#                 dominates_count[dominated_sol] -= 1
#                 if dominates_count[dominated_sol] == 0:
#                     next_front.append(dominated_sol)
#         i += 1
#         fronts.append(next_front)
#
#     fronts = [set(front) for front in fronts if front]
#
#     # Sorting within each front based on crowding distance
#     sorted_population_indices = []
#     for front in fronts:
#         front_list = list(front)
#         front_solutions = [solutions[i] for i in front_list]
#         crowding_distances = calculate_crowding_distance(front_solutions)
#         # Sort the front based on crowding distance (descending order)
#         sorted_front_indices = sorted(front_list, key=lambda i: crowding_distances[front_list.index(i)], reverse=True)
#         sorted_population_indices.extend(sorted_front_indices)
#
#     sorted_population = [solutions[i] for i in sorted_population_indices]
#
#     return sorted_population
#
# # Example usage
# solutions = [
#     [-1, -2,-3],
#     [-2, -1,-2],
#     [1.5, -1.5,-5],
#     [-3, -3,-3],
#     [2.5, 2,3],
#     [2, 2.5,3]
# ]
# sorted_population = non_dominated_sorting_with_weights(solutions)
#
# print(sorted_population)


print([m for m in range(30)])
import numpy as np


# def compute_distances(arr):
#     n = arr.shape[0]
#     distances_sum = np.zeros(n)
#
#     for i in range(n):
#         distances = np.sqrt(np.sum((arr - arr[i]) ** 2, axis=1))
#         print(distances)
#         distances_sum[i] = np.sum(distances)
#
#     return distances_sum
#
#
# # 示例
# arr = np.array([[1, 2], [3, 4], [5, 6]])  # 一个3x2的多维数组
# result = compute_distances(arr)
# print(result)