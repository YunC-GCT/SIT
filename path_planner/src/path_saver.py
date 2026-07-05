import numpy as np

def save_final_path(path, output_path):
    np.save(output_path, np.array(path))