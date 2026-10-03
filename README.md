# 🚚 VeRoLog Logistics Scheduling & Inventory Optimization Solver

[![Python](https://img.shields.io/badge/Python-3.9+-blue.svg)](https://www.python.org/)
[![OR-Tools](https://img.shields.io/badge/Google%20OR--Tools-CP--SAT-orange.svg)](https://developers.google.com/optimization)
[![Optimization](https://img.shields.io/badge/VRP-Local%20Search-brightgreen.svg)]()

This project is dedicated to solving a complex **Multi-day Vehicle Routing Problem (VRP) with Tool Distribution and Collection**, derived from real-world industrial scenarios (VeRoLog Solver Challenge). By decoupling "temporal allocation" from "spatial routing," and combining Google OR-Tools Constraint Programming with a custom-built Local Search optimizer, we successfully tackled the business pain point of "dispatch deadlocks" under severely restricted global inventory.

---

## 💡 Business Background & Core Pain Points

Within a finite planning horizon, the system must fulfill a massive volume of tool rental requests from customers.
* **Global Inventory Mutex (Tool Locking):** The total number of tools is limited. Once delivered, they must remain at the customer's location for a given number of consecutive days ($D_r$), during which they are deducted from the depot's available inventory.
* **Strict Spatiotemporal Binding:** Customers have strict delivery time windows. Furthermore, the pickup day is strictly bound to the delivery day: $d_{pick} = d_{del} + D_r + 1$.
* **Harsh Physical Constraints:** Vehicles dispatched daily are strictly limited by maximum weight capacity and maximum travel distance, and they must start and end at the single depot.
* **Dispatch Deadlock Risk:** If daily dispatching is strictly "greedy" (short-sighted), it easily leads to critical tools being locked at distant customer sites and unable to be collected, causing subsequent orders to fail and the entire schedule to deadlock.

---

## 🏗️ Algorithm Architecture: Three-Phase Decoupled Approach

To combat the combinatorial explosion in ultra-large-scale instances (up to 30~70 days, 1000+ requests), we intentionally discarded the highly inefficient fully-coupled MILP approach in favor of a **heuristic three-phase decoupled architecture**:

### 📍 Phase 1: Macro Tool Distribution Planning (Temporal Decision)
We built the foundational temporal allocation model using **Google OR-Tools (CP-SAT)**.
* Transformed time windows and maximum inventory into hard constraints to decide the exact delivery date for each request.
* Innovatively introduced an **Automatic Diagnostic & Relaxation Mechanism (Diagnostic Run)**: When inventory is critically depleted and infeasible, the model automatically introduces slack variables to locate "overload bottlenecks," guiding business adjustments and ensuring the feasibility of the daily schedule from the source.

### 📍 Phase 2: Basic Route Construction (Spatial Initial Solution)
Reduced the dimensionality of the problem to independent single-day VRP scheduling.
* Utilized a greedy algorithm to rapidly generate initial routes satisfying distance and capacity constraints.
* Pioneered the **"Critical Pickups" evaluation metric & Bonus incentive mechanism** (assigning a high-priority weight of 50,000). This triggers an "Emergency Mode" forcing vehicles to prioritize collecting scarce tools, ensuring spatial planning strictly adheres to the temporal schedule.

### 📍 Phase 3: Custom Route Optimizer (VND Local Search)
To address the massive "single-trip" and redundant deadheading issues generated during the greedy construction phase, we developed a custom local search algorithm based on **Variable Neighborhood Descent (VND)**. We reconstructed a composite cost function encompassing travel distance and fixed vehicle costs, designing 6 intelligent operators:
1. `Move Inside Route` (Relocate a single node within the same route)
2. `Swap Inside Route` (Swap two nodes within the same route)
3. `Move Block` (Relocate a contiguous block of nodes within the same route, preserving local sub-tours)
4. `Reverse Block / 2-Opt` (Reverse a block to untangle crossed routes)
5. **`Inter-Route Relocate` (Move a node between routes - The ultimate cost-killer!)**
6. `Inter-Route Swap` (Swap nodes between different routes)

Through localized Delta Evaluation, we achieved rapid intelligent merging of discrete nodes and "Cross-docking on trucks."

---

## 📊 Core Optimization Achievements

Extensive benchmarking and stress testing were conducted across the `Early`, `Hidden`, and `Late` complexity tiers:

* 🚀 **Extreme Vehicle Compression Rate**: During the review of the core high-pressure test set (Day 2), the new optimizer successfully merged massive inefficient scattered orders, slashing the single-day dispatch volume from the baseline's **84 vehicles** down to just **13 vehicles** (a compression rate of **84.5%**), drastically reducing redundant mileage and global fixed asset overhead.
* 🛡️ **Outstanding Robustness**: In the `Late` datasets with extremely tight inventory constraints, the Phase 1 allocation strategy effectively prevented Tool Starvation, guaranteeing a 100% feasibility rate.

---

## 📂 Project File Structure

```text
.
├── distribution_model.py        # Phase 1 & 2: CP-SAT underlying tool allocation & Greedy basic route generation
├── route_optimiser.py           # Phase 3: VND local search optimizer core algorithm (6 major operators)
├── probalkozasok.ipynb          # Jupyter Main Dashboard: Data loading, batch testing, pipeline orchestration
├── SolutionCVRPTWUI.py          # Official solution validator & cost calculator
├── /ORTEC-early                 # Dataset (Early public standard set)
├── /ORTEC-hidden                # Dataset (Large-scale hidden validation set)
└── /ORTEC-late                  # Dataset (Tight time windows & extreme inventory challenge set)
