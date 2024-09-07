"""Script to read in npz and csv file to compare sinusoidal position test data."""

import argparse
import csv
from typing import List

import matplotlib.pyplot as plt
import numpy as np


def read_csv(file_path: str) -> List[float]:
    """Reads a CSV file and returns the data."""
    data = {"step": [], "position": []}
    with open(file_path, "r") as f:
        reader = csv.reader(f)
        # Skip header
        next(reader)
        for row in reader:
            data["step"].append(int(row[0]))
            data["position"].append(float(row[2]))
    return data


def main(npz_file: str, csv_file: str) -> None:
    """Reads in npz and csv file to compare sinusoidal position test data."""
    npz_data = np.load(npz_file)
    csv_data = read_csv(csv_file)
    """
    npz data is in form of:
    {
        "step": np.array([...]),
        "position": np.array([...]),
        "velocity": np.array([...]),
    }

    csv data is in form of:
    {
        "step": [...],
        "position": [...],
    }
    """

    print(csv_data)

    fig, ax = plt.subplots()
    ax.plot(csv_data["step"], csv_data["position"], label="real data")
    ax.plot(npz_data["step"], npz_data["position"], label="sim data")
    ax.set_xlabel("Step")
    ax.set_ylabel("Position")
    ax.legend()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read in npz and csv file to compare sinusoidal position test data.")
    parser.add_argument("npz_file", type=str, help="The npz file.")
    parser.add_argument("csv_file", type=str, help="The csv file.")
    args = parser.parse_args()
    main(args.npz_file, args.csv_file)
