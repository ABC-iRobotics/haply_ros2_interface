import pandas as pd
import matplotlib.pyplot as plt
import plot_likert
import tikzplotlib

# Teilnehmerantworten
data_question1 = [
    ["+2","0","+1"],
    ["+1","+1","-1"],
    ["+2","0","+1"],
    ["+2","0","+1"],
    ["+3","+3","+3"],
    ["+2","+2","+1"],
    ["+1","+1","+2"],
    ["+2","+1","0"],
    ["+3","-2","-1"],
    ["0","+2","0"],
    ["+3","+2","+1"],
    ["+2","+1","+2"],
    ["+1","+2","+3"],
    ["+2","+1","+1"],
    ["+3","+3","+3"],
    ["+2","+2","+2"],
    ["+3","0","0"]
]
data_question2 = [
    ["-1","+3","+2"],
    ["-2","-1","-2"],
    ["+1","+2","+1"],
    ["-1","+1","0"],
    ["-2","-1","-2"],
    ["-1","-1","-1"],
    ["-1","+1","-2"],
    ["+2","+1","+2"],
    ["-2","-2","-1"],
    ["-1","-2","+1"],
    ["-1","-1","+1"],
    ["-2","+1","-1"],
    ["+2","+2","+2"],
    ["-2","-3","-2"],
    ["+2","+3","+1"],
    ["+2","+1","+2"],
    ["-1","+1","0"]
]
data_question3 = [
    ["0","+2","+1"],
    ["-1","-1","+1"],
    ["+1","+2","+2"],
    ["-2","+2","-1"],
    ["-3","-2","-3"],
    ["-1","-1","-1"],
    ["-2","-1","-2"],
    ["0","+2","+3"],
    ["-3","-2","-2"],
    ["-1","0","+1"],
    ["-2","0","+1"],
    ["-1","+2","+1"],
    ["+1","-2","+1"],
    ["-1","-2","-1"],
    ["0","+3","-2"],
    ["+2","0","0"],
    ["-3","+1","0"]
]

datafile1 = pd.DataFrame(data_question1, columns=['M0', 'M1', 'M2'])
datafile2 = pd.DataFrame(data_question2, columns=['M0', 'M1', 'M2'])
datafile3 = pd.DataFrame(data_question3, columns=['M0', 'M1', 'M2'])

# Farben passend zur Skala

my_color_scheme =[
    #plot_likert.colors.TRANSPARENT,
    "white",      # transparent 
    "darkred", # -3
    "red",     # -2
    "orange",  # -1
    "lightgray", # 0
    "lightblue", # +1
    "blue",    # +2
    "darkblue" # +3
]

# Eigene Skala
my_scale = ["-3", "-2", "-1", "0", "+1", "+2", "+3"]

# Likert-Plot
fig, ax = plt.subplots(figsize=(10,3))
plot_likert.plot_likert(
    datafile1,
    my_scale,          
    plot_percentage=False,
    colors=my_color_scheme,
    ax=ax,
    #bar_labels=True,
    #bar_labels_color="snow",
    figsize=(10,3)
)
ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1))

tikzplotlib.save("/mnt/c/Users/hiwi_student/Documents/Haply_daVinci_system/tikz_files/likert_plot_question1.tex",
    axis_width='12cm',
    axis_height='4cm',
    )
plt.tight_layout()
plt.show()