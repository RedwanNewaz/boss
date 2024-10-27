#!/Users/redwan/miniconda3/envs/limbo/bin/python
import matplotlib.pyplot as plt 
import numpy as np 
import pandas as pd 
import yaml
from matplotlib.patches import Circle
from fire import Fire

def main(csv_file="output.csv", env_file="test/env1.yaml"):
    with open(env_file) as file:
        env = yaml.safe_load(file)
    path = np.array(pd.read_csv(csv_file))
    plt.plot(path[:, 0], path[:, 1])
    ax = plt.gca()
    for ob in env['obstacles']:
        c = Circle(ob, 0.456)
        ax.add_patch(c)
    plt.show()


if __name__ == "__main__":
    Fire(main)
