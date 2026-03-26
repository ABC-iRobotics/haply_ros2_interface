import matplotlib.pyplot as plt
import tikzplotlib

# -----------------------
# Daten vorbereiten
# -----------------------
data = {
    "NumberContacts": {
        0: [52, 35, 40, 27, 5, 26, 78, 19, 24, 25, 21, 30, 41, 44, 29, 21, 40],
        1: [11, 35, 11, 23, 5, 9, 50, 24, 32, 17, 10, 39, 39, 5, 26, 21, 15],
        2: [15, 18, 47, 44, 16, 9, 14, 14, 17, 11, 15, 64, 46, 24, 35, 15, 40],
    },
    "TimeNeeded": {
        0: [108.82, 93.14, 133.66, 172.76, 57.24, 66.92, 130.92, 59.55, 125.31, 48.01, 83.77, 143.65, 149.69, 103.26, 72.1, 64.49, 94.89],
        1: [56.80, 147.82, 85.69, 115.31, 56.60, 29.67, 113.26, 44.16, 83.45, 44.99, 68.24, 129.23, 110.90, 41.02, 74.58, 56.09, 81.97],
        2: [68.86, 109.17, 105.38, 161.44, 66.81, 79.89, 54.50, 53.51, 59.44, 35.37, 114.82, 149.77, 133.53, 53.63, 93.92, 52.3, 83.9],
    }
}

modes = [0,1,2]
colors = ['skyblue','lightgreen','salmon']

# -----------------------
# Boxplots erstellen
# -----------------------
fig, axes = plt.subplots(1, 2, figsize=(12,5))

positions = [1, 1.6, 2.2]

# Number of Contacts
axes[0].boxplot([data["NumberContacts"][m] for m in modes],
                positions=positions,
                widths=0.25,
                patch_artist=True,
                labels=[f"Mode {m}" for m in modes],
                boxprops=dict(facecolor='skyblue', color='blue'),
                medianprops=dict(color='red'))
axes[0].set_title("Number of Contacts")
#axes[0].set_ylabel("Number of Contacts")
axes[0].grid(True)

# Time Needed
axes[1].boxplot([data["TimeNeeded"][m] for m in modes],
                positions=positions,
                widths=0.25,
                patch_artist=True,
                labels=[f"Mode {m}" for m in modes],
                boxprops=dict(facecolor='lightgreen', color='green'),
                medianprops=dict(color='red'))
axes[1].set_title("Time Needed (s)")
#axes[1].set_ylabel("Time Needed [s]")
axes[1].grid(True)

plt.tight_layout()
fig.subplots_adjust(wspace=0.1)
tikzplotlib.save("/mnt/c/Users/hiwi_student/Documents/Haply_daVinci_system/tikz_files/boxplot_data_visualization.tex")
plt.show()