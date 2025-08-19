
import matplotlib.pyplot as plt

# Dosyayı oku
data = {"Gaussian": [], "Actual": [], "Prediction": [], "Update": [], "Landmark": []}

with open("poses.txt") as f:
    for line in f:
        parts = line.strip().split(",")
        label = parts[0]
        x, y, theta = map(float, parts[1:])
        data[label].append((x, y, theta))

# Grafiğe dök
plt.figure(figsize=(6,6))

for label, values in data.items():
    if not values:  # boşsa atla
        continue
    xs, ys, _ = zip(*values)
    plt.scatter(xs, ys, marker="o", label=label)

plt.legend()
plt.xlabel("X")
plt.ylabel("Y")
plt.title("EKF Logging")
plt.grid(True)
plt.show()
