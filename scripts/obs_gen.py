import numpy as np
from fire import Fire
obstacles = [
    [0.75, 0.75], [1.8, 1.2], [2.25, 2.25], [1.05, 1.95], [2.55, 0.45],
    [3.3, 2.7], [1.5, 0.05], [3.75, 0.75], [3.0, 4.5], [2.95, 1.5],
    [0.05, 1.5], [1.5, 3.0], [4.8, 1.2], [0.75, 3.75], [4.05, 1.95],
    [4.8, 4.2], [3.75, 3.75], [4.5, 3.0], [2.55, 3.45], [1.8, 4.2]
]

def scatter_obstacles(obstacles, min_distance=0.75, max_iterations=1000):
    scattered = np.array(obstacles)
    for _ in range(max_iterations):
        moved = False
        for i in range(len(scattered)):
            for j in range(i + 1, len(scattered)):
                distance = np.linalg.norm(scattered[i] - scattered[j])
                if distance < min_distance:
                    direction = scattered[j] - scattered[i]
                    direction /= np.linalg.norm(direction)
                    move_distance = (min_distance - distance) / 2
                    scattered[i] -= direction * move_distance
                    scattered[j] += direction * move_distance
                    moved = True
        if not moved:
            break
    return scattered.tolist()

def main(min_distance=0.75):
    scattered_obstacles = scatter_obstacles(obstacles, min_distance)

    print("Scattered obstacles:")
    for obstacle in scattered_obstacles:
        print(f"[{obstacle[0]:.2f}, {obstacle[1]:.2f}],")

    # Verify minimum distance
    min_distance = float('inf')
    for i in range(len(scattered_obstacles)):
        for j in range(i + 1, len(scattered_obstacles)):
            distance = np.linalg.norm(np.array(scattered_obstacles[i]) - np.array(scattered_obstacles[j]))
            min_distance = min(min_distance, distance)

    print(f"\nMinimum distance between obstacles: {min_distance:.2f} m")

if __name__ == "__main__":
    Fire(main)
