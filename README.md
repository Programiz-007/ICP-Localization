# ICP-Localization

A from-scratch, notebook-based derivation and implementation of 2D **Iterative Closest Point (ICP)** scan matching — the algorithm behind most LiDAR/point-cloud localization and scan-to-scan registration pipelines (Cartographer, PCL, KISS-ICP, etc.). Everything is built with `numpy`/`sympy`/`matplotlib` only, no point-cloud libraries, so every step of the math is visible and animatable.

## Contents

| Notebook | Covers |
|---|---|
| [`IterativeClosestPoint.ipynb`](./IterativeClosestPoint.ipynb) | SVD-based ICP, non-linear least-squares (Gauss-Newton) ICP, point-to-plane ICP, robust-kernel outlier rejection |
| [`svd_eigen.ipynb`](./svd_eigen.ipynb) | Why SVD of the cross-covariance matrix gives the optimal rotation — relates eigendecomposition of the covariance matrix to SVD of the data matrix |

## Problem setup

Given two 2D scans (point sets) $P = \{p_i\}$ and $Q = \{q_i\}$, find the rigid transform $(R, t)$ that best aligns $P$ onto $Q$:

$$
E(R, t) = \sum_i \| R\,p_i + t - q_i \|^2 \rightarrow \min
$$

Each ICP iteration alternates between **correspondence estimation** (nearest-neighbor matching, brute-force here) and **transform estimation** (closed-form via SVD, or a linearized least-squares step).

## Methods implemented

### 1. SVD-based ICP (closed-form, per iteration)
- Center both point sets on their centroids.
- Compute the cross-covariance matrix $K = \sum (q_i - \mu_Q)(p_i - \mu_P)^T$.
- Decompose $K = USV^T$; the optimal rotation is $R = UV^T$, translation is recovered from the centroids.
- This is the orthogonal Procrustes solution — exact for a given correspondence set, no linearization needed.

### 2. Non-linear least-squares ICP (Gauss-Newton)
- State $x = [x, y, \theta]^T$, error $e_{i,j}(x) = R_\theta p_i + t - q_j$.
- Builds the Jacobian $J = [I \;\; R'_\theta p_i]$, accumulates $H = \sum J^TJ$, $g = \sum J^Te$, solves $H\Delta x = -g$ each iteration.
- Point-to-point metric; converges slower than SVD-ICP for this reason — motivates method 3.

### 3. Point-to-plane ICP
- Computes surface normals along $Q$ (2D normal of a tangent segment: $n_v = [-v_y, v_x]$).
- Error is the correspondence residual projected onto the normal: $E = \sum_i (n_i \cdot (R_\theta p_i + t - q_j))^2$.
- Jacobian collapses from $2\times3$ to $1\times3$ (verified symbolically with `sympy`). Converges in noticeably fewer iterations than point-to-point.

### 4. Outlier handling via robust kernels
- Injects synthetic outliers into $P$ and shows that all three methods above (SVD, least-squares, point-to-plane) drift/fail without correction.
- Introduces a weighting kernel $w(e) = \mathbb{1}[\|e\| < \text{threshold}]$ applied to each correspondence's contribution to $H$/$g$ (or to the cross-covariance in the SVD case).
- For SVD-ICP specifically, discusses the two places outliers leak in — the rotation step (fixable by down-weighting the cross-covariance sum) and the centering/translation step (fixable only by iteratively re-estimating the centroid while excluding marked outliers).

### 5. SVD ↔ eigendecomposition equivalence (`svd_eigen.ipynb`)
- For a covariance matrix $\Sigma = X^TX / (N-1)$, shows $\Sigma = VLV^T$ (eigendecomposition) and $X = USV^T$ (SVD of the data) give the *same* $V$, with $L = S^2/(N-1)$.
- Visual sanity check: fits confidence ellipses to a synthetic Gaussian point cloud two ways (from `np.linalg.eig` on the covariance vs. `np.linalg.svd` on the centered data) and confirms they coincide.
- This is the piece of linear algebra that justifies why SVD of the cross-covariance matrix in method 1 is doing something geometrically meaningful.

## Repo structure
```
ICP-Localization/
├── IterativeClosestPoint.ipynb   # main notebook — all 4 ICP variants + outlier handling
├── svd_eigen.ipynb               # SVD vs eigendecomposition theory + visualization
└── README.md
```

## Running locally

```bash
git clone https://github.com/Programiz-007/ICP-Localization.git
cd ICP-Localization
pip install numpy matplotlib sympy jupyter
jupyter notebook
```

No external datasets — all point clouds are synthetically generated in-notebook, so each cell is reproducible standalone. The animation cells use `matplotlib.animation`; if `%matplotlib inline` playback is choppy, switch to `%matplotlib notebook` or export the animation to GIF.

## References
- Besl & McKay, *A Method for Registration of 3-D Shapes*, IEEE PAMI 1992 (original ICP)
- Chen & Medioni, *Object Modeling by Registration of Multiple Range Images*, 1992 (point-to-plane metric)
- Low, *Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration*, 2004

## Roadmap / possible next steps
- [ ] Extend to 3D point clouds (rotation via quaternion/SO(3), same SVD machinery)
- [ ] KD-tree correspondence search instead of brute-force $O(NM)$
- [ ] Generalized-ICP (plane-to-plane) formulation
- [ ] Benchmark against a real LiDAR scan pair (e.g. KITTI or a TurtleBot3 Gazebo scan)
