import csv
import matplotlib.pyplot as plt
import numpy as np

def plot_results(csv_file):
    force = []
    time = []

    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            time.append(float(row[0]))
            force.append(float(row[2]))

    total_force = sum(force)

    fig, ax = plt.subplots()
    ax.plot(time, force, label='Force')
    ax.set_title(f'Robot force vs time\nSum of forces: {total_force:.2f}')
    ax.set_xlabel('Time')
    ax.set_ylabel('Force')
    ax.grid(True)
    ax.legend()

    plt.show()

if __name__ == "__main__":
    csv_file = 'Fase3_Ana_Martinez.csv'
    plot_results(csv_file)
