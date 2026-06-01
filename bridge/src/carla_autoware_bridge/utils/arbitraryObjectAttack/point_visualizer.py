import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import ast

# -----------------------------
# CONFIG
# -----------------------------
CAR_LENGTH = 4.0
CAR_WIDTH = 1.6

TOP_K_POINTS = 10
TOP_K_SETS = 3

# -----------------------------
# Load data
# -----------------------------
df = pd.read_csv("point_removal_importance.csv")

# -----------------------------
# Convert side-relative y -> global y
# -----------------------------
def compute_y(row):
    if row["side"] == "right":
        return CAR_WIDTH/2 + row["y_offset"]
    else:
        return -CAR_WIDTH/2 - row["y_offset"]

df["y_global"] = df.apply(compute_y, axis=1)

# Use center score as importance
df["importance"] = df["w_score_center"]

# -----------------------------
# PART 1: Per-set Top-K points
# -----------------------------
sets = sorted(df["set_id"].unique())

fig, axes = plt.subplots(
    1,
    len(sets),
    figsize=(6 * len(sets), 6),
    sharex=True,
    sharey=True
)

if len(sets) == 1:
    axes = [axes]

for ax, s in zip(axes, sets):

    subset = df[df["set_id"] == s]

    # take best K points (lowest rank)
    subset = subset.sort_values("point_rank").head(TOP_K_POINTS)

    xs = subset["x_offset"]
    ys = subset["y_global"]
    imp = subset["importance"]

    sc = ax.scatter(
        xs,
        ys,
        c=imp,
        cmap="plasma",
        s=200 * (imp / (imp.max() + 1e-6)),
        edgecolors="black"
    )

    # draw car
    rect = plt.Rectangle(
        (0, -CAR_WIDTH/2),
        CAR_LENGTH,
        CAR_WIDTH,
        fill=False,
        linewidth=2,
        color="black"
    )
    ax.add_patch(rect)

    # rank labels
    for _, r in subset.iterrows():
        ax.text(
            r["x_offset"],
            r["y_global"],
            int(r["point_rank"]),
            ha="center",
            va="center",
            fontsize=8
        )

    ax.set_title(f"Set {s} (Top {TOP_K_POINTS})")
    ax.set_aspect("equal")
    ax.grid(True)

fig.colorbar(sc, ax=axes, label="Importance (center score)")
plt.tight_layout()
plt.show()

# -----------------------------
# Helper: convert y
# -----------------------------
def compute_y_point(p):
    if p["side"] == "right":
        return CAR_WIDTH/2 + p["y_offset"]
    else:
        return -CAR_WIDTH/2 - p["y_offset"]

# -----------------------------
# Load data
# -----------------------------
df_sel = pd.read_csv("point_selection_set.csv")

# -----------------------------
# Per distance visualization
# -----------------------------
for d in sorted(df_sel["distance"].unique()):

    df_d = df_sel[df_sel["distance"] == d]

    # take final step per set
    final_steps = df_d.sort_values("num_points").groupby("set_id").tail(1)

    # select best sets (lowest score)
    best_sets = final_steps.nsmallest(TOP_K_SETS, "score_center")

    fig, axes = plt.subplots(
        1,
        len(best_sets),
        figsize=(6 * len(best_sets), 6),
        sharex=True,
        sharey=True
    )

    if len(best_sets) == 1:
        axes = [axes]

    for ax, (_, row) in zip(axes, best_sets.iterrows()):

        set_id = row["set_id"]

        # ✅ parse actual points
        points = ast.literal_eval(row["points"])

        xs = []
        ys = []

        for p in points:
            xs.append(p["x_offset"])
            ys.append(compute_y_point(p))

        ax.scatter(
            xs,
            ys,
            color="red",
            s=120,
            edgecolors="black"
        )

        # draw car
        rect = plt.Rectangle(
            (0, -CAR_WIDTH/2),
            CAR_LENGTH,
            CAR_WIDTH,
            fill=False,
            linewidth=2,
            color="blue"
        )
        ax.add_patch(rect)

        ax.set_title(f"Set {set_id}\nScore={row['score_center']:.3f}")
        ax.set_aspect("equal")
        ax.grid(True)

    fig.suptitle(f"Top {TOP_K_SETS} Attack Sets @ Distance {d}m")

    plt.tight_layout()
    plt.show()
