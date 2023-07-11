from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from PRM import PRM

def load_map(file_path, resolution_scale):
    # Load the image with grayscale
    img = Image.open(file_path).convert('L')

    # Rescale the image
    size_x, size_y = img.size
    new_x, new_y = int(size_x * resolution_scale), int(size_y * resolution_scale)
    img = img.resize((new_x, new_y), Image.BILINEAR)
    map_array = np.asarray(img, dtype='uint8')

    # Convert the grayscale image to binary image
    threshold = 150
    map_array = 1 * (map_array > threshold)

    # Return the 2D numpy array representation of the map
    return map_array

if __name__ == "__main__":
    # Load the map
    start = (250, 30)
    goal = (20, 200)
    map_array = load_map("ASU_Map.jpg", 0.3)

    # Create PRM object with the loaded map
    prm_obj = PRM(map_array)

    # Sample and search with PRM using different sampling methods

    # Uniform Sampling
    connectivity_threshold = prm_obj.sample(n_pts=1000, sampling_method="uniform")
    prm_obj.search(start, goal, connectivity_threshold, "Uniformly Sampled Probabilistic Roadmap (PRM)")

    # Random Sampling
    connectivity_threshold = prm_obj.sample(n_pts=1000, sampling_method="random")
    prm_obj.search(start, goal, connectivity_threshold, "Randomly Sampled Probabilistic Roadmap (PRM)")

    # Gaussian Sampling
    connectivity_threshold = prm_obj.sample(n_pts=1500, sampling_method="gaussian")
    prm_obj.search(start, goal, connectivity_threshold, "Gaussian-Sampled Probabilistic Roadmap(PRM)")

    # Bridge Sampling
    connectivity_threshold = prm_obj.sample(n_pts=40000, sampling_method="bridge")
    prm_obj.search(start, goal, connectivity_threshold, "Bridge-Sampled Probabilistic Roadmap(PRM)")