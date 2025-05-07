import imageio.v2 as imageio
import os

png_dir = "/home/jhkeselman/WPI_SwarmIntelligence/swarm-final-zkm/frames/noalgo"
files = sorted([f for f in os.listdir(png_dir) if f.endswith(".png")])

with imageio.get_writer("output.mp4", fps=100) as writer:
    for file in files:
        image = imageio.imread(os.path.join(png_dir, file))
        writer.append_data(image)
