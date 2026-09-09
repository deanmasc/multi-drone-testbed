import matplotlib.pyplot as plt
import sys
import glob

# Get filename from argument, or automatically find latest log
if len(sys.argv) > 1:
    filename = sys.argv[1]
else:
    files = glob.glob("square_log_*.txt")

    if not files:
        print("No square log files found.")
        sys.exit(1)

    # Filenames contain YYYYMMDD_HHMMSS, so alphabetical sorting works
    filename = max(files)

print(f"Using log file: {filename}")

x = []
y = []
target_x = []
target_y = []

with open(filename, "r") as file:
    for line in file:
        # Skip comments and empty lines
        if line.startswith("#") or not line.strip():
            continue

        data = line.split()

        x.append(float(data[2]))
        y.append(float(data[3]))
        target_x.append(float(data[4]))
        target_y.append(float(data[5]))


# ==========================================
# Graph 1 - Actual drone path only
# ==========================================

plt.figure()

points = plt.scatter(
    x,
    y,
    c=range(len(x)),
    cmap="viridis",
    s=10,
    label="Actual drone path"
)

plt.scatter(x[0], y[0], s=100, marker="o", label="Start")
plt.scatter(x[-1], y[-1], s=100, marker="X", label="Finish")

cbar = plt.colorbar(points)
cbar.set_label("Flight progression (start → finish)")

plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("Crazyflie Flight Path")
plt.legend()
plt.grid()
plt.axis("equal")


# ==========================================
# Graph 2 - Actual drone path + target square
# ==========================================

plt.figure()

plt.plot(
    target_x,
    target_y,
    "--",
    label="Target path"
)

points = plt.scatter(
    x,
    y,
    c=range(len(x)),
    cmap="viridis",
    s=10,
    label="Actual drone path"
)

plt.scatter(x[0], y[0], s=100, marker="o", label="Start")
plt.scatter(x[-1], y[-1], s=100, marker="X", label="Finish")

cbar = plt.colorbar(points)
cbar.set_label("Flight progression (start → finish)")

plt.xlabel("X position (m)")
plt.ylabel("Y position (m)")
plt.title("Crazyflie Flight Path vs Target Path")
plt.legend()
plt.grid()
plt.axis("equal")


# Show both graphs
plt.show()
