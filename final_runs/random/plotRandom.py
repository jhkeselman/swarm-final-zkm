import pandas as pd
import matplotlib.pyplot as plt
import glob

files = glob.glob("*.txt")
metric_names = ['tasks', 'taskCompletionRatio', 'totalReward']

# Loop over each metric and create a separate plot
for i, metric in enumerate(metric_names):
    plt.figure(figsize=(12, 6))  # Create a new figure for each metric

    # Loop through each file and plot the data for the current metric
    for file in files:
        try:
            df = pd.read_csv(file, delim_whitespace=True, header=None, comment='#')
            df.columns = ['clock', 'tasks', 'taskCompletionRatio', 'totalReward']

            # Convert all columns to numeric values
            for col in df.columns:
                df[col] = pd.to_numeric(df[col], errors='coerce')
            df.dropna(inplace=True)

            # Cut the data to the first 1000 time steps (or rows)
            df = df.head(1000)

            label = file.split('/')[-1].replace('.txt', '')
            plt.plot(df['clock'], df[metric], label=label)

        except Exception as e:
            print(f"Skipping file {file} due to error: {e}")

    # Set labels, title, and legend for each plot
    plt.xlabel('Clock')
    plt.ylabel(metric)
    plt.title(f'{metric} Over Time Using Random Task Selection')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    # Show the plot
    plt.show()

