import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from matplotlib.patches import Ellipse

def draw_cov_ellipse(ax, mean, cov, n_sigma=2.0, **kwargs):
    """
    mean: (x, y)
    cov: 2x2 kovaryans matrisi [[sx2, sxy],[sxy, sy2]]
    n_sigma: kaç sigma (1, 2, 3 ...)
    kwargs: Ellipse için ek stil argümanları (edgecolor, lw, alpha, facecolor vs.)
    """
    # Özdeğer/özvektör
    vals, vecs = np.linalg.eigh(cov)  # vals artan sırada gelir
    # Büyük eksen = en büyük özdeğerin karekökü
    order = np.argsort(vals)[::-1]
    vals, vecs = vals[order], vecs[:, order]

    # Yarı eksen uzunlukları (sigma ölçekli)
    width  = 2 * n_sigma * np.sqrt(vals[0])  # major
    height = 2 * n_sigma * np.sqrt(vals[1])  # minor

    # Açı (derece): major eksenin yön vektöründen
    angle = np.degrees(np.arctan2(vecs[1,0], vecs[0,0]))

    e = Ellipse(xy=mean, width=width, height=height, angle=angle,
                facecolor='none', **kwargs)
    ax.add_patch(e)
    return e


data = {"Gaussian": [], "Actual": [], "Prediction": [], "Update": [], "Landmark": [], "Landmark EKF": []}

with open("poses.txt") as f:
    for line in f:
        parts = line.strip().split(",")
        if not parts or len(parts) < 8:   # label + 7 sayı yoksa atla
            continue  
        label = parts[0]
        try:
            x, y, theta, t, sx2, sy2, sxy = map(float, parts[1:])
        except ValueError:
            print("Hatalı satır:", parts)
            continue
        data[label].append((x, y, theta, t, sx2, sy2, sxy))

# Maksimum zaman değeri bul
max_t = int(max(v[3] for values in data.values() for v in values))

# Grafik ayarları
fig, ax = plt.subplots(figsize=(6,6))
ax.set_xlim(min(v[0] for values in data.values() for v in values)-1,
            max(v[0] for values in data.values() for v in values)+1)
ax.set_ylim(min(v[1] for values in data.values() for v in values)-1,
            max(v[1] for values in data.values() for v in values)+1)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_title("EKF Logging - Animation")
ax.grid(True)

scatters = {label: ax.scatter([], [], marker="o", label=label) for label in data}
texts = []  # artık temizlemiyoruz, her frame’de yenisi ekleniyor
ellipses = []   # ellipse objeleri

def update(frame_t):
    for label, scatter in scatters.items():
        # o label’a ait frame_t’ye kadar olan verileri al
        values = [(x,y,th,t,sx2,sy2,sxy) for (x,y,th,t,sx2,sy2,sxy) in data[label] if int(t) <= frame_t]
        if values:
            xs, ys, ths, ts, sx2s, sy2s, sxys = zip(*values)
            scatter.set_offsets(np.column_stack((xs, ys)))

            for i, tt in enumerate(ts):
                if int(tt) == frame_t:
                    # Noktanın yanına zaman yaz
                    texts.append(ax.text(xs[i], ys[i], f"{tt:.0f}", fontsize=8, ha="right", va="bottom"))
                    
                    # Kovaryans matrisi
                    cov = np.array([[sx2s[i], sxys[i]],
                                    [sxys[i], sy2s[i]]], dtype=float)
                    
                    # Elips çiz (her label farklı renk olabilir istersen)
                    e = draw_cov_ellipse(ax, (xs[i], ys[i]), cov, n_sigma=2.0,
                                         edgecolor=scatter.get_facecolor()[0], lw=1.5, alpha=0.6)
                    ellipses.append(e)

    ax.legend()
    return list(scatters.values()) + texts + ellipses


ani = animation.FuncAnimation(fig, update, frames=range(max_t+1), interval=1000, blit=False, repeat=False)

plt.show()
