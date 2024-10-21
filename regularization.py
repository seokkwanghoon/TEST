import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse, Polygon

# L1 제약과 L2 제약을 시각화하기 위한 함수
def plot_l1_l2_constraints():
    fig, ax = plt.subplots(figsize=(8, 8))

    # 원형 등고선 (손실 함수)
    x = np.linspace(-1.5, 1.5, 400)
    y = np.linspace(-1.5, 1.5, 400)
    X, Y = np.meshgrid(x, y)
    Z = X**2 + Y**2  # L2 손실 함수 (원형 등고선)
    
    # L1 패널티 (다이아몬드 모양)
    l1_penalty = Polygon([(-1, 0), (0, 1), (1, 0), (0, -1)], closed=True, fill=False, edgecolor='r', lw=2, label="L1 Penalty")
    
    # L2 패널티 (원형)
    ellipse = Ellipse(xy=(0, 0), width=2, height=2, edgecolor='b', fill=False, lw=2, label="L2 Penalty")

    # 등고선 그리기 (L2 손실 함수)
    ax.contour(X, Y, Z, levels=[0.5, 1, 1.5, 2], colors='black', alpha=0.6)

    # L1, L2 제약 그리기
    ax.add_patch(l1_penalty)
    ax.add_patch(ellipse)

    # 축 설정
    ax.axhline(0, color='black',linewidth=1)
    ax.axvline(0, color='black',linewidth=1)
    
    # 범례와 제목
    plt.legend(handles=[l1_penalty, ellipse], loc='upper right')
    plt.xlim([-1.5, 1.5])
    plt.ylim([-1.5, 1.5])
    plt.title("L1 Penalty (Diamond) vs L2 Penalty (Circle)")
    plt.gca().set_aspect('equal', adjustable='box')
    
    plt.show()

# 함수 실행
plot_l1_l2_constraints()
