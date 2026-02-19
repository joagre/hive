# Copter Arena - Adversarial Quadrotor Games

Core Wars, but with quadcopters. Programmable drones competing in a
bounded arena under formal rules.

Each drone runs a Forth script that composes native maneuver primitives
into a strategy. The script is the strategy layer (when to attack, evade,
feint). The ROM words are the control layer (how to flip, orbit, pursue).
See [acrobatics_pilot_design.md](acrobatics_pilot_design.md) for the ROM
contract and [forth_actor_spec.md](forth_actor_spec.md)
for the VM.

## 1. Agile Flight Control

Quadcopters are **differentially flat systems** - the key enabler.

- Outputs: position (x, y, z) and yaw
- Inputs: total thrust + body torques
- Result: plan aggressive trajectories in output space and exactly
  reconstruct required motor commands

This is why flips, rolls, knife-edge flight, and snap maneuvers are
tractable. Not tricks - locally optimal solutions under tight constraints.

Foundations:
- Mellinger & Kumar (2011): "Minimum snap trajectory generation and
  control for quadrotors"

Recent advances:
- Foehn et al. (2023): "Agilicious" - open-source agile quadrotor,
  5g at 70 km/h, vision-based acrobatics
- Romero et al. (2024): "Towards MPC for Acrobatic Quadrotor Flights"
- Torrente et al. (2023): "Learning quadrotor dynamics for precise,
  safe, and agile flight control"

## 2. Optimal Control for Acrobatics

Beyond smooth trajectories, combat-style maneuvers require nonlinear
optimal control:

- Pontryagin's Minimum Principle
- Direct collocation methods
- Model Predictive Control, especially NMPC

Used for rapid attitude reversals, energy-efficient evasive maneuvers,
and tight constraint handling (thrust limits, angular rate limits).
Already applied in autonomous drone racing, vision-based gap traversal,
and high-speed obstacle avoidance.

## 3. Hybrid Systems and Mode Switching

Acrobatic flight is not one controller. It is a hybrid system with
discrete modes:

- Hover / cruise
- Aggressive translation
- Pure attitude control (zero velocity, high angular rate)
- Ballistic / thrust-limited phases

Theory: hybrid automata, switched systems, guard conditions on angular
velocity, thrust margin, energy state.

This maps directly to the ROM's injection levels. A Forth script switches
between levels as the maneuver demands - Level 1 for cruise, Level 3 for
flips, back to Level 1 for recovery. The override mechanism is the mode
switch. The cascade actors are the guard conditions (they resume if the
override goes stale).

## 4. Adversarial Control and Differential Games

This is where the idea becomes genuinely interesting.

Foundational theory:
- Isaacs (1965): "Differential Games" - pursuit-evasion as formal
  mathematics

Modern developments:
- Yan et al. (2023): "Multiplayer reach-avoid differential games"
- Chen et al. (2024): "Multi-UAV Pursuit-Evasion with Online Planning
  in Unknown Environments" (Deep RL)
- Li et al. (2024): "Multi-UAV pursuit-evasion based on PSO-M3DDPG"

Key techniques:
- Hamilton-Jacobi reachability analysis
- Multi-agent reinforcement learning (MARL)
- Transfer learning for obstacle environments

Each drone is a controlled dynamical system with partial observability,
competing for spatial dominance or kill conditions. Studied in missile
guidance, autonomous dogfighting (fixed-wing), and multi-agent robotics.
The missing step is bringing it to agile quadrotors in close proximity.

## 5. Multi-Agent Extension

With more than two drones:

- Distributed control and decentralized MPC
- Game-theoretic equilibria
- Collision avoidance as hard constraints

This enables feints, area denial, and sacrificial blocking - embodied
strategy, not just flying.

Recent work:
- Liu et al. (2024): "Game of Drones" - intelligent online decision
  making for multi-UAV confrontation
- Hu et al. (2024): "Transfer RL for multi-agent pursuit-evasion"
- Zhang et al. (2023): "Collaborative pursuit-evasion based on
  Apollonius circle"

## 6. Learning vs Control

Many people jump to reinforcement learning. The best results are hybrid:

- Model-based control for execution
- Learning for high-level policy selection

Discrete action selection (attack, evade, block, feint) with continuous
control underneath. This mirrors how humans fly aerobatics - and it is
exactly the Forth/ROM split. The Forth script selects strategy. The ROM
words execute physics.

## 7. Arena Constraints

The arena is not a gimmick. It is a formal boundary condition.

Define:
- Energy budgets
- Forbidden zones (the ROM's FENCE word)
- Kill conditions (tagging, line-of-sight lock, proximity)
- Reset rules

This transforms the problem into a bounded differential game with
reproducible evaluation metrics. Exactly what Core Wars did for
instruction-level competition.

## Next Steps

- Formal model for a 1-vs-1 quad duel (state space, payoff function,
  terminal conditions)
- Arena rule set that makes this tractable without killing people
- Extend the ROM with pursuit-evasion primitives (TRACK, EVADE,
  INTERCEPT)

## References

### Foundations
- Mellinger & Kumar (2011): [Minimum snap trajectory generation and control for quadrotors](https://ieeexplore.ieee.org/document/5980409/) - IEEE ICRA
- Isaacs (1965): [Differential Games: A Mathematical Theory with Applications to Warfare and Pursuit](https://www.amazon.com/Differential-Games-Mathematical-Applications-Optimization/dp/0486406822) - Dover

### Agile Flight (2023-2024)
- Foehn et al. (2023): [Agilicious: Open-source and open-hardware agile quadrotor](https://arxiv.org/abs/2307.06100) - Science Robotics
- Romero et al. (2024): [Towards Model Predictive Control for Acrobatic Quadrotor Flights](https://arxiv.org/abs/2401.17418) - arXiv
- Torrente et al. (2023): [Learning quadrotor dynamics for precise, safe, and agile flight control](https://www.sciencedirect.com/science/article/abs/pii/S1367578823000135) - Annual Reviews in Control

### Pursuit-Evasion & Multi-Agent (2023-2025)
- Chen et al. (2024): [Multi-UAV Pursuit-Evasion with Online Planning in Unknown Environments](https://arxiv.org/abs/2409.15866) - arXiv
- Li et al. (2024): [Multi-UAV pursuit-evasion gaming based on PSO-M3DDPG](https://link.springer.com/article/10.1007/s40747-024-01504-1) - Complex & Intelligent Systems
- Hu et al. (2024): [Transfer RL for multi-agent pursuit-evasion differential game](https://onlinelibrary.wiley.com/doi/abs/10.1002/asjc.3328) - Asian Journal of Control
- Survey (2025): [Recent Advances in Pursuit-Evasion Games with Unmanned Vehicles](https://www.mdpi.com/2077-1312/13/3/458) - MDPI
