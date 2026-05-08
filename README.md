# GNC-for-In-Orbit-Servicing-Proximity-Operations
A curated technical reference on **Guidance, Navigation, and Control (GNC)** for autonomous rendezvous, proximity operations, and docking/capture — the foundational technologies for **in-orbit servicing** and **active debris removal**.



---

## 1. Relative Navigation

> Determining the 6-DOF pose (position and orientation) of a chaser relative to a **non-cooperative target**.

### 1.1 Sensor Modalities

| Sensor | Principle | Strengths | Limitations | Best Use Case |
|--------|----------|-----------|-------------|---------------|
| **Monocular Camera** | Bearing‑only angles from 2D images | Simple, passive, flight‑proven | No direct range; struggles with extreme lighting | Mid‑range bearing estimation |
| **Stereo Camera** | Triangulation from two cameras | Direct 3D, well‑understood | Limited baseline on small spacecraft; calibration drift | Short‑range (<100 m) |
| **Flash Lidar** | Single‑pulse time‑of‑flight point cloud | Instantaneous 3D, lighting‑invariant | Higher power; point‑cloud processing complexity | Final approach & capture (<50 m) |
| **Scanning Lidar** | Rotating mirror time‑of‑flight | High resolution | Slow refresh rate; unsuitable for tumbling targets | Meter‑scale mapping only |
| **Infrared Camera** | Thermal signature | Works in eclipse | Lower resolution; target must have thermal contrast | Redundant sensor in darkness |

### 1.2 Navigation Filter Architecture

| Filter | Linearity Handling | Computational Cost | When to Use |
|--------|-------------------|-------------------|-------------|
| **Extended Kalman Filter (EKF)** | Linearizes dynamics & measurements | Low | Moderately nonlinear systems; flight‑proven |
| **Unscented Kalman Filter (UKF)** | Sigma‑point sampling avoids linearization | Medium | Highly nonlinear dynamics (tumbling target) |
| **Particle Filter** | Full non‑parametric distribution | High | Highly non‑Gaussian problems; rare in real‑time flight |
| **Iterated EKF** | Re‑linearizes measurement at updated state | Medium | When measurement nonlinearity dominates |

**Key design questions:**
- Are you augmenting the state to estimate the target's **inertia tensor** online?
- How do you handle target **mode switching** (stable spin → tumble)? Consider an **Interacting Multiple Model (IMM)** filter.
- What is your **covariance realism** strategy to prevent filter smugness?

### 1.3 Angles‑Only Navigation

A particularly challenging problem when range is unobservable. Key approaches:
- **Orbital mechanics prior:** Exploit known relative dynamics to infer range from bearing rate over time.
- **Manoeuvre‑based observability:** Perform a small, known translation to create parallax.
- **Biased CRLB analysis:** Understand the fundamental limits of range estimation accuracy.

---

## 2. Image Processing for Space

> Extracting actionable navigation information from camera and lidar data under extreme conditions.

### 2.1 Vision‑Based Pose Estimation Pipeline

| Stage | Classical Approach | Learning‑Based Approach |
|-------|-------------------|------------------------|
| **Preprocessing** | Histogram equalization, HDR fusion | Learned tone‑mapping (e.g., HDR‑Net) |
| **Feature Detection** | SIFT, SURF, ORB, Harris corner | SuperPoint, D2‑Net, R2D2 |
| **Feature Matching** | RANSAC + nearest‑neighbour | SuperGlue, LoFTR (dense matching) |
| **Pose Estimation** | PnP (Perspective‑n‑Point) | Direct 6‑DOF regression (PoseNet, EfficientPose) |
| **Temporal Filtering** | Kalman filter on pose | Learned odometry (DeepVO, TartanVO) |

### 2.2 Point‑Cloud Pose Estimation (Lidar)

| Architecture | Key Idea | Reference |
|-------------|----------|-----------|
| **PointNet++** | Hierarchical feature learning on raw point clouds | Qi et al., 2017 |
| **DCP (Deep Closest Point)** | Learned soft matching + SVD for rigid transform | Wang & Solomon, 2019 |
| **PCRNet** | Iterative point cloud registration network | Sarode et al., 2019 |
| **TEASER++** | Certifiably robust registration (outlier‑tolerant) | Yang et al., 2021 |

### 2.3 Space‑Specific Challenges

| Challenge | Mitigation Strategy |
|-----------|-------------------|
| **Extreme dynamic range** (bright sun, deep shadow) | HDR sensors; adaptive exposure; learned normalization |
| **Target surface degradation** (peeling MLI, specular reflections) | Multi‑modal fusion; robust loss functions in learning models |
| **Sim‑to‑real domain gap** | Domain randomization during training; fine‑tuning on orbital data |
| **Limited onboard compute** | Model quantization (INT8); pruning; dedicated inference accelerators (Google Coral TPU, LS1046) |
| **Verification of AI components** | See Section 5 — V&V Strategies |

---

## 3. Formation Flying & Proximity Operations

> The operational framework for safe, autonomous multi‑body motion in close proximity.

### 3.1 Relative Motion Models

| Model | Dynamics | Assumptions | Application |
|-------|----------|-------------|-------------|
| **Clohessy‑Wiltshire (CW)** | Linear, time‑invariant | Circular reference orbit; close range | Rapid trajectory design, safety analysis |
| **Lawden / Tschauner‑Hempel (TH)** | Linear, time‑varying | Elliptical reference orbit | Higher‑eccentricity missions |
| **Nonlinear relative dynamics** | Full two‑body difference | None | High‑fidelity simulation, truth model |

### 3.2 Safety Architecture for Proximity Operations

| Concept | Purpose | Implementation |
|---------|---------|----------------|
| **Passive Safety** | Guarantee no collision after thruster failure | Design relative trajectory whose natural drift does not intersect target |
| **Active Collision Avoidance** | Planned evasive manoeuvre | FDIR‑triggered burn; closed‑loop guidance to safe state |
| **Keep‑Out Zone (KOZ)** | Hard geometric safety boundary | Spherical or ellipsoidal constraint in flight software |
| **Traveling Safety Ellipse** | Fuel‑efficient, inherently safe station‑keeping | CW‑based 2×1 ellipse; natural periodic relative motion |
| **Abort Manoeuvre** | Pre‑planned deterministic escape sequence | Pre‑loaded burn sequence → safe parking orbit |

### 3.3 Guidance Law Taxonomy

| Approach | Principle | Strengths | Limitations |
|----------|-----------|-----------|-------------|
| **Glideslope Guidance** | Linear reduction of approach velocity with range | Simple, deterministic, verifiable | Sub‑optimal fuel use |
| **CW‑Based Guidance** | Analytic solution of relative motion model | Explicit passive safety design | Circular orbit assumption |
| **Convex Optimization (e.g., SCvx)** | Real‑time trajectory optimization with hard constraints | Guarantees KOZ satisfaction | Computational cost; need for warm‑start |
| **Artificial Potential Fields (APF)** | Repulsive force from obstacles + attractive force to target | Intuitive, reactive | Local minima; difficult to verify |
| **Control Barrier Functions (CBF)** | Quadratic‑program safety filter on nominal controller | Formal safety guarantee; minimal intervention | Requires valid CBF candidate |
| **Reinforcement Learning** | Learned policy from simulation | Handles uncertain dynamics | Safety verification challenge (see Section 5) |

---

## 4. Fault Detection, Isolation & Recovery (FDIR)

> Ensuring graceful degradation and deterministic safety in the presence of failures.

### 4.1 FDIR Architecture Levels

| Level | Scope | Response Time | Example |
|-------|-------|---------------|---------|
| **Sensor‑level** | Individual sensor fault | Sub‑second | IMU bias spike → switch to redundant IMU |
| **Navigation‑level** | Divergence of navigation filter | Seconds | Covariance inflation → trigger UKF reset |
| **Guidance‑level** | Trajectory violation | Seconds | KOZ violation → abort manoeuvre |
| **System‑level** | Catastrophic failure | Immediate | Total power loss → passive safety design saves mission |

### 4.2 Common Failure Modes in Proximity Operations

| Failure | Detection Method | Recovery Strategy |
|---------|-----------------|-------------------|
| Thruster stuck‑on | Unexpected ΔV from accelerometer | Switch to redundant thruster branch; abort |
| Thruster stuck‑off | Commanded ΔV not achieved | Degraded control allocation; abort if KOZ threatened |
| Sensor dropout | Stale data flags; filter innovation test | Rely on propagation; trigger sensor switch |
| Navigation filter divergence | Covariance test; innovation magnitude | Reset filter with fresh initialization |
| Processing overload | Watchdog timer; frame drops | Graceful degradation (reduce sensor rate); safe mode |

---

## 5. Verification & Validation (V&V) for Safety‑Critical GNC

> How to trust an autonomous system — especially one containing learned components.

### 5.1 V&V Strategy Framework

| Stage | Method | Goal |
|-------|--------|------|
| **Unit Testing** | Software‑in‑the‑Loop (SIL) | Verify individual GNC functions |
| **Monte Carlo Simulation** | High‑fidelity 6‑DOF simulator | Statistical performance bounds |
| **Processor‑in‑the‑Loop (PIL)** | Code running on flight‑like processor | Timing, memory, numerical precision |
| **Hardware‑in‑the‑Loop (HIL)** | Real sensors + emulated dynamics | Sensor latency, noise, fault injection |
| **Formal Methods** | Reachability analysis, proof assistants | Mathematical safety guarantees |

### 5.2 V&V for AI‑Based GNC Components

| Method | Description | Maturity |
|--------|-------------|----------|
| **Adversarial Robustness Testing** | Search input space for worst‑case failures | Medium |
| **Runtime Monitoring / Safety Shields** | CBF filter or anomaly detector independent of the AI | High (flight‑proven concept) |
| **Formal Verification of Neural Networks** | SMT solvers, abstract interpretation (e.g., Marabou, α‑β‑CROWN) | Low (scalability challenges) |
| **Sim‑to‑Real Gap Quantification** | Statistical validation with domain‑randomized simulators | Medium |
| **Explainability & Interpretability** | Saliency maps, concept bottlenecks | Low (qualitative, not certifiable) |

**Key principle:** Never trust a learned component with sole authority over a safety‑critical decision. Architect a **safety monitor** (classical algorithm or CBF‑based) that can override the AI if necessary.

---

## 6. Key Papers

| Year | Authors | Title | Relevance |
|------|---------|-------|-----------|
| 2017 | Qi et al. | PointNet++: Deep Hierarchical Feature Learning on Point Sets in a Metric Space | Foundation for lidar‑based perception |
| 2013 | Açıkmeşe et al. | Lossless Convexification of Non‑Convex Control Bound and Pointing Constraints | Foundation for convex GNC |
| 2019 | Szmuk et al. | Successive Convexification for 6‑DoF Powered Descent Guidance | Real‑time trajectory optimization |
| 2021 | Yang et al. | TEASER++: Fast & Certifiable Point Cloud Registration | Robust lidar‑based relative nav |
| 2022 | ESA/NASA | On‑Orbit Servicing GNC Standards (various) | Regulatory & safety framework |
| 2020 | Sharma et al. | Pose Estimation for Non‑Cooperative Spacecraft Rendezvous | Survey of vision‑based methods |

---

## 7. Open‑Source Tools

| Tool | Domain | Language | Link |
|------|--------|----------|------|
| **GMAT** | Mission design, trajectory optimization | C++/Python | NASA |
| **Orekit** | Orbit propagation, frames | Java | CS GROUP |
| **Nyx** | High‑fidelity astrodynamics | Rust/Python | nyxspace.com |
| **Basilisk** | 6‑DOF spacecraft simulation | C++/Python | AVS Lab (CU Boulder) |
| **42** | Spacecraft attitude & orbit simulation | C | NASA |
| **OpenCV** | Classical computer vision | C++/Python | opencv.org |
| **Kornia** | Differentiable computer vision | Python | kornia.org |
| **PyTorch3D** | 3D deep learning, rendering | Python | Meta |

---

## 8. Future Directions

1. **Certifiable Learned GNC** — Bridging the gap between neural network performance and formal safety guarantees (CBF + RL hybrids).
2. **Multi‑Agent In‑Orbit Servicing** — Coordinated multi‑chaser architectures for complex debris removal.
3. **On‑Orbit AI Updates** — Safe mechanisms for updating neural network weights post‑launch.
4. **Standardized V&V Benchmarks** — Community‑driven datasets and test protocols for space AI (analogous to ImageNet for terrestrial vision).

---

*Maintained by [@mayhammad23773]. Contributions and paper suggestions welcome via PR.*
