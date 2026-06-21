#!/usr/bin/env python3
"""
UWB 2D Relative Localization — Real-time MDS Visualization

Usage:
    python visualize.py /dev/ttyACM0 115200          (4 nodes, default)
    python visualize.py /dev/ttyACM0 115200 --n 3    (3 nodes)
    python visualize.py --test                       (fake square, no hardware)
    python visualize.py --test --n 3

WAZNE: --n musi byc rowne N_NODES we firmware (src/localization/node.cpp).
MDS wymaga WSZYSTKICH par niezerowych -> tyle modulow ile wezlow.
"""

import sys
import serial
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# N ustawiane z CLI (--n), domyslnie 4
N          = 4      # nodes (nadpisane w main z argumentu --n)
BOARD      = 4.0    # fixed plot size [m]
EMA_ALPHA  = 0.25   # smoothing: 0=frozen, 1=raw (0.25 = heavy smooth)
HISTORY    = 30     # trail length

# -------------------------------------------------------------------------
# MDS
# -------------------------------------------------------------------------

def dist_vec_to_matrix(dists):
    """[d01,d02,d03,d12,d13,d23] → 4×4 symmetric distance matrix."""
    D = np.zeros((N, N))
    k = 0
    for i in range(N):
        for j in range(i + 1, N):
            D[i, j] = dists[k]
            D[j, i] = dists[k]
            k += 1
    return D


def mds_2d(D):
    """Classical MDS. Returns Nx2 positions or None on bad input."""
    n = D.shape[0]
    off = D[~np.eye(n, dtype=bool)]
    if np.any(off <= 0) or np.any(~np.isfinite(D)):
        return None
    D2 = D ** 2
    H  = np.eye(n) - np.ones((n, n)) / n
    B  = -0.5 * H @ D2 @ H
    vals, vecs = np.linalg.eigh(B)
    order = np.argsort(vals)[::-1]
    vals, vecs = vals[order], vecs[:, order]
    X = vecs[:, :2] * np.sqrt(np.maximum(vals[:2], 0))
    return X


def target_layout(n):
    """
    Docelowe rozmieszczenie wezlow (do przypiecia orientacji).
    Etykiety na wykresie sa 1-indeksowane: index 0 = "Node 1", itd.

    Zyczenie uzytkownika (idac w kolko):
        Node 1 = lewy dolny  (BL)
        Node 2 = prawy dolny (BR)
        Node 3 = prawy gorny (TR)
        Node 4 = lewy gorny  (TL)
    """
    c = BOARD / 2
    if n == 4:
        return np.array([[c-1, c-1],   # Node 1 -> BL
                         [c+1, c-1],   # Node 2 -> BR
                         [c+1, c+1],   # Node 3 -> TR
                         [c-1, c+1]])  # Node 4 -> TL
    if n == 3:
        return np.array([[c-1, c-1],   # Node 1 -> BL
                         [c+1, c-1],   # Node 2 -> BR
                         [c,   c+1]])  # Node 3 -> gora-srodek
    if n == 2:
        return np.array([[c-1, c-1], [c+1, c-1]])
    ang = np.linspace(0, 2*np.pi, n, endpoint=False)
    return c + 1.3 * np.column_stack([np.cos(ang), np.sin(ang)])


def fix_orientation(X):
    """
    Przypina orientacje deterministycznie: dopasowuje ksztalt MDS (obrot +
    odbicie + skala + przesuniecie = Procrustes) do docelowego ukladu naroznikow,
    tak by Node i zawsze ladowal w swoim rogu. Skalowanie wypelnia widok i
    sprawia ze realne odleglosci nadal sa proporcjonalne (dokladne wartosci
    pokazuje tabelka). Zalezy tylko od biezacej klatki -> nie skacze/nie obraca sie.
    """
    Y  = target_layout(X.shape[0])
    Xc = X - X.mean(axis=0)
    Yc = Y - Y.mean(axis=0)

    # Ortogonalny Procrustes z odbiciem: R = U @ Vt z SVD(Xc^T Yc)
    M = Xc.T @ Yc
    U, sv, Vt = np.linalg.svd(M)
    R = U @ Vt
    aligned = Xc @ R

    # Skala dopasowujaca rozmiar do ukladu docelowego (wypelnia widok)
    denom = (Xc ** 2).sum()
    scale = (sv.sum() / denom) if denom > 1e-9 else 1.0
    aligned = scale * (Xc @ R)

    return aligned + Y.mean(axis=0)


# -------------------------------------------------------------------------
# Serial helpers
# -------------------------------------------------------------------------

def open_serial(port, baud):
    try:
        s = serial.Serial(port, baud, timeout=0.1)
        print(f"[OK] {port} @ {baud}")
        return s
    except serial.SerialException as e:
        print(f"[ERR] {e}")
        sys.exit(1)


def parse_matrix_line(line):
    line = line.strip()
    if not line.startswith("MATRIX,"):
        return None
    parts = line.split(",")
    if len(parts) != 1 + N * (N - 1) // 2:    # MATRIX + C(N,2) wartosci
        return None
    try:
        return np.array([float(p) for p in parts[1:]])
    except ValueError:
        return None


# -------------------------------------------------------------------------
# Plot
# -------------------------------------------------------------------------

COLORS = ["#e74c3c", "#3498db", "#2ecc71", "#f39c12",
          "#9b59b6", "#1abc9c", "#e67e22", "#95a5a6"]


def make_test_dists(n):
    """Syntetyczne dystanse z tego samego ukladu co docelowy (test = sanity check)."""
    pts = target_layout(n)
    return np.array([np.linalg.norm(pts[i] - pts[j])
                     for i in range(n) for j in range(i + 1, n)])


def main():
    global N
    test_mode = "--test" in sys.argv
    if "--n" in sys.argv:
        N = int(sys.argv[sys.argv.index("--n") + 1])

    # Pozycyjne argumenty to port i baud (pomijamy flagi --x i ich wartosci)
    pos = []
    skip = False
    for i, a in enumerate(sys.argv[1:]):
        if skip:
            skip = False
            continue
        if a == "--n":
            skip = True
            continue
        if a.startswith("--"):
            continue
        pos.append(a)
    port = pos[0] if pos else "/dev/ttyACM0"
    baud = int(pos[1]) if len(pos) > 1 else 115200

    labels = [f"Node {i+1}" for i in range(N)]   # 1-indeksowane etykiety
    test_dists = make_test_dists(N)

    ser = None
    if not test_mode:
        ser = open_serial(port, baud)
    else:
        print(f"[TEST] fake {N}-node geometry")

    trails   = [deque(maxlen=HISTORY) for _ in range(N)]
    smooth_X = None      # EMA state

    fig, ax = plt.subplots(figsize=(7, 7))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#16213e")
    ax.set_title("UWB Relative Localization (MDS)", color="white", fontsize=13)
    ax.tick_params(colors="white")
    ax.xaxis.label.set_color("white")
    ax.yaxis.label.set_color("white")
    for spine in ax.spines.values():
        spine.set_edgecolor("#444")

    # Fixed axes — never rescale
    pad = 0.4
    ax.set_xlim(-pad, BOARD + pad)
    ax.set_ylim(-pad, BOARD + pad)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    # Grid
    ax.grid(True, color="#2a2a4a", linewidth=0.5, linestyle="--")

    scat        = [ax.plot([], [], 'o',  color=COLORS[i % len(COLORS)], markersize=13,
                           label=labels[i], zorder=5)[0] for i in range(N)]
    trails_plot = [ax.plot([], [], '-',  color=COLORS[i % len(COLORS)], alpha=0.25,
                           linewidth=1.5)[0]              for i in range(N)]
    node_texts  = [ax.text(0, -99, labels[i], color="white", fontsize=9,
                           ha="center", va="bottom", zorder=6) for i in range(N)]
    dist_text   = ax.text(0.02, 0.98, "", transform=ax.transAxes,
                          color="#aaa", fontsize=8, va="top", family="monospace")
    ax.legend(loc="upper right", facecolor="#222", labelcolor="white", edgecolor="#444")

    buf = ""

    def update(_frame):
        nonlocal buf, smooth_X

        # --- get distances ---
        if test_mode:
            noise     = np.random.normal(0, 0.015, len(test_dists))
            dists_vec = test_dists + noise
        else:
            try:
                raw  = ser.read(ser.in_waiting or 1)
                buf += raw.decode("utf-8", errors="replace")
            except Exception:
                return
            dists_vec = None
            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                parsed = parse_matrix_line(line)
                if parsed is not None:
                    dists_vec = parsed
                    print(f"[RX] {line}")
            if dists_vec is None:
                return

        # --- MDS ---
        D = dist_vec_to_matrix(dists_vec)
        X = mds_2d(D)
        if X is None:
            print(f"[SKIP] bad distances: {np.round(dists_vec, 3)}")
            return

        # --- fix orientation (Node 1 always SW, consistent chirality) ---
        X = fix_orientation(X)

        # --- EMA smoothing: damps frame-to-frame jitter ---
        if smooth_X is None:
            smooth_X = X.copy()
        else:
            smooth_X = EMA_ALPHA * X + (1.0 - EMA_ALPHA) * smooth_X

        # --- draw ---
        txt_offset = BOARD * 0.04   # fixed 4% of board height

        for i in range(N):
            p = smooth_X[i]
            scat[i].set_data([p[0]], [p[1]])
            node_texts[i].set_position((p[0], p[1] + txt_offset))
            trails[i].append(p.copy())
            if len(trails[i]) > 1:
                t = np.array(trails[i])
                trails_plot[i].set_data(t[:, 0], t[:, 1])

        # distance table (2 wiersze)
        pair_labels, k = [], 0
        for i in range(N):
            for j in range(i + 1, N):
                pair_labels.append(f"d{i+1}{j+1}={dists_vec[k]:.2f}m")
                k += 1
        half = (len(pair_labels) + 1) // 2
        dist_text.set_text("  ".join(pair_labels[:half]) + "\n" +
                           "  ".join(pair_labels[half:]))

    ani = animation.FuncAnimation(fig, update, interval=80, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
    if ser:
        ser.close()


if __name__ == "__main__":
    main()
