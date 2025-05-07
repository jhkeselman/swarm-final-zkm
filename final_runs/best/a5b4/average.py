import pandas as pd
import numpy as np
import glob

# Automatically find all .txt files in the current directory
filenames = glob.glob("*.txt")

# Initialize a list to hold DataFrames
dfs = []

# Read and process each file
for fname in filenames:
    df = pd.read_csv(fname, sep='\t', comment='#')

    # Clip to clock <= 1000
    df = df[df['clock'] <= 1000]

    # Ensure clock is integer
    df['clock'] = df['clock'].astype(int)

    dfs.append(df)

# Merge all DataFrames by clock and compute mean
merged = pd.concat(dfs).groupby('clock').mean().reset_index()

# Reorder columns
merged = merged[['clock', 'tasks', 'taskCompletionRatio', 'totalReward']]

# Save data
with open("a5_b4.txt", "w") as f:
    f.write("clock\ttasks\ttaskCompletionRatio\ttotalReward\n")
    for _, row in merged.iterrows():
        f.write(f"{int(row['clock'])}\t{row['tasks']:.2f}\t{row['taskCompletionRatio']:.2f}\t{row['totalReward']:.2f}\n")
