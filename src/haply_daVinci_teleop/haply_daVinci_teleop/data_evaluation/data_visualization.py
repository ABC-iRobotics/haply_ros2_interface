import matplotlib.pyplot as plt
import tikzplotlib

# -----------------------
# Daten vorbereiten
# -----------------------
data = {
    "NumberContacts": {
        0: [52, 35, 40, 27, 5, 26],
        1: [11, 35, 11, 23, 5, 9],
        2: [15, 18, 47, 44, 16, 9],
    },
    "TimeNeeded": {
        0: [108.82, 93.14, 133.66, 172.76, 57.24, 66.92],
        1: [56.80, 147.82, 85.69, 115.31, 56.60, 29.67],
        2: [68.86, 109.17, 105.38, 161.44, 66.81, 79.89],
    }
}

modes = [0,1,2]
colors = ['skyblue','lightgreen','salmon']

# -----------------------
# Boxplots erstellen
# -----------------------
fig, axes = plt.subplots(1, 2, figsize=(14,6))

# Number of Contacts
axes[0].boxplot([data["NumberContacts"][m] for m in modes], patch_artist=True,
                labels=[f"Mode {m}" for m in modes],
                boxprops=dict(facecolor='skyblue', color='blue'),
                medianprops=dict(color='red'))
axes[0].set_title("Number of Contacts")
axes[0].set_ylabel("Number of Contacts")

# Time Needed
axes[1].boxplot([data["TimeNeeded"][m] for m in modes], patch_artist=True,
                labels=[f"Mode {m}" for m in modes],
                boxprops=dict(facecolor='lightgreen', color='green'),
                medianprops=dict(color='red'))
axes[1].set_title("Time Needed (s)")
axes[1].set_ylabel("Time Needed [s]")

plt.tight_layout()
tikzplotlib.save("/mnt/c/Users/hiwi_student/Documents/Haply_daVinci_system/tikz_files/boxplot_data_visualization.tex")
plt.show()