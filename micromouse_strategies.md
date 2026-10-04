- Remember to reference your research pls (IEEE)? — reference list at the bottom, cite as [1], [2], ...

# Micromouse Strategies & Research Notes

## 1. The brief (what we are actually being asked to do)

Design and build a small autonomous robot that finds its way to the **centre of a walled maze in the fastest possible time** [1]. Assessment is about **successful stages leading towards the final outcome**, and the problems we hit and how we handled them must be documented [1].

**Hard constraints** [1]:

| Constraint | Value |
| :---- | :---- |
| Controller | Raspberry Pi Pico (W), programmed in **MicroPython** — module requirement (Thonny IDE recommended) |
| Budget | Total bill of materials **under £100** |
| Size | Must fit in a **16 cm x 16 cm** footprint (no height limit). Cells are 18 cm with 1.2 cm walls → 16.8 cm navigable |
| Autonomy | Fully autonomous, no outside communication, no feeding maze info in after it's revealed |
| Maze integrity | Must not damage, mark or climb walls, or leave debris |
| Time | 10 min total per contestant in the rules appendix (the problem statement says 15 min — **check with supervisor**) |

**Maze facts** [1]:
- Up to 16 x 16 cells of 18 cm; walls 5 cm high, 1.2 cm thick.
- Walls white-sided with red tops, floor black — but **don't rely on colours or lighting**; floor may be slippery and have seams.
- Start in a corner, walled on 3 sides. Goal is the **2 x 2 centre** with **one entrance**, deliberately placed so a **wall-follower can never reach it**.
- At least one wall touches every lattice post. Multiple routes to the goal are expected.
- Robot should drive itself back to the start for repeat speed runs (being touched = run aborted, +30 s penalty for re-placing).

## 2. Development plan (staged approach)

The course material strongly pushes a **staged approach** [1], [2]:

1. **Weeks 1–2: 3pi evaluation robot** — study its hardware and get line following and line-maze solving running. (We only have it for the first few weeks.)
2. **Pico basics** — blink an LED, PWM a motor, read a sensor in MicroPython.
3. **Algorithm in simulation** — write the flood-fill solver in normal Python on a laptop against a text maze, then port to the Pico.
4. **Our own robot** — chassis (3D print / laser cut), motors + driver, IR wall sensors, encoders.
5. **Drive control** — drive exactly one cell straight, turn exactly 90°, stay centred between walls (PID).
6. **Exploration run** → **speed run** → **return to start** → repeat.

Parts beyond what's in the lab must be negotiated with the supervisor, and **ordering has a lead time** — decide on parts early.

## 3. Maze-solving algorithms

### Follow the left (or right) wall
Pros: very simple, needs no memory or mapping.  
Cons: **will not work in a competition maze** — the centre is deliberately placed so wall-huggers can't reach it (rule change after a wall-follower won the 1979 contest) [1]. Only useful as an early test behaviour.

### Depth-first search
Pros: always finds *a* route [1].  
Cons: not necessarily the shortest, and wastes time exploring the whole maze [1].

### Flood fill (Bellman's algorithm) — **recommended**
Each cell stores its distance (in cells) to the centre, starting from an empty maze. At every cell the mouse [1]:
1. Checks for new walls and updates the wall map.
2. Re-floods the maze to update the distances.
3. Moves to the neighbour with the lowest distance.

Pros: works in an **unknown** maze, always finds the shortest path, built for exactly this problem.  
Cons: needs more memory (a 16 x 16 distance array + wall map — trivial for the Pico's 264 KB RAM).  
Note: shortest ≠ fastest — a path with fewer turns can be quicker [1].

**Modified flood fill**: only updates the cells whose values need to change rather than re-flooding the whole maze → faster per step [1]. Good upgrade once basic flood fill works.

### A\*
Pros: fast, efficient path search.  
Cons: assumes the map is already known, so on its own it's not an exploration strategy. Could be used for the speed run after mapping, but flood fill already gives that for free. Keep as a comparison in the report.

### Pledge algorithm
Designed to escape a maze to an **outside exit**, not reach a goal in the middle — **not suitable**. Worth one line in the report to show we considered and rejected it.

## 4. Lessons from the 3pi robot [2]

- **PID control**: turns a sensor error (distance off-centre) into a smooth motor speed difference. The 3pi example uses `P/20 + I/10000 + D*3/2`. Tune at low speed first, then raise speed and re-tune. We'll need the same idea for staying centred between walls.
- **Differential drive + PWM**: two independently driven wheels; speed set by PWM duty cycle through an H-bridge driver (TB6612FNG on the 3pi). Spin on the spot by driving wheels in opposite directions.
- **Regulated motor voltage**: the 3pi boosts to 9.25 V so motor speed doesn't drop as the battery drains — which makes timed turns repeatable. Without this, **use encoders** instead of timing.
- **Battery monitoring**: a voltage divider into an ADC lets the robot report battery level — cheap to copy on the Pico.
- **Path simplification**: the 3pi line-maze solver records turns (L/S/R/B) and collapses dead-end detours, e.g. `LBL → S`, `LBS → R`. Only works for mazes without loops, so not enough for us on its own.
- **Speed runs**: record segment lengths during exploration, then drive long straights fast and slow down only before turns.
- **Traction**: at speed, dusty tyres cause fishtailing — clean them with rubbing alcohol every few runs.
- Line following needs a different approach from wall following, but it's the best place to learn sensors, PID and motor control [1].

## 5. Hardware research

**Sensors** [1]:
- **IR (recommended)**: cheap, not affected by electrical noise. Sensitive to ambient light → take one reading with the emitter on and one with it off, then subtract. Sharp GP2D120 (4–30 cm, analogue) scored best in the background doc's comparison. Distance sensing (not just on/off proximity) is needed to stay centred.
- **Ultrasonic**: colour-independent, but suffers from electrical noise and transducer ringing.
- **Touch/bump**: too late to react on their own; maybe as a backup.
- Keep the sensor system as simple as possible — more sensors ≠ more useful data.

**Odometry** [1]: wheel encoders (photo-interrupter, photoreflector or Hall-effect). **Quadrature** encoders also give direction and double the resolution. Needed for driving exactly one cell and turning exactly 90°.

**Chassis** [1]: light but rigid; fits in 16 x 16 cm. Low parts may snag on floor seams.

**Power** [1], [2]: LiPo is compact and powerful but needs a **fuse** and short-proof connectors. NiMH is the safer choice. Regulated supply for the Pico.

## 6. Questions to answer early [1]

- What is a PWM timer and how does it work?
- What is a PID controller and how does it work?
- How does a voltage regulator differ from PID? Is it beneficial to use both?
- What design software will we use (Fusion 360)? Can we build a box in it?
- Can we make an LED flash on the Pico?
- Can we read and create hardware diagrams?
- What sensor functionality do we need? How will the robot detect a junction? How will it turn a corner?
- How does load (and its distribution) affect the motors and manoeuvrability?
- How will the robot solve the maze?

## 7. Safety & ethics [1]

- Soldering: goggles, ventilation, wash hands after handling solder/leaded parts.
- Never touch a powered circuit; know the lab power cut-off and fire extinguisher locations.
- LiPo batteries: fuse and shorting-proof connectors.
- Prefer lead-free components.
- **Cite all code and ideas** from others. No copying between groups.

## 8. Open questions for the supervisor

- Contest time: 10 min or 15 min?
- What size is the lab maze (full 16 x 16 or smaller)? Build the solver with the size as a parameter either way.
- What parts are available in the lab vs. need ordering, and the ordering process/lead time?
- What exactly is submitted, and when (reports via VLE)?

## 9. References

[1] University of York, School of Physics, Engineering and Technology, "Stage 2 Engineering Project: Maze Solving Robot — Background Information," 2026–27.

[2] Pololu Corporation, "Pololu 3pi Robot User's Guide," 2022. [Online]. Available: https://www.pololu.com/docs/0J21

[3] Pololu Corporation, "Pololu AVR C/C++ Library User's Guide." [Online]. Available: https://www.pololu.com/docs/0J20

[4] Pololu Corporation, "Pololu AVR Library Command Reference." [Online]. Available: https://www.pololu.com/docs/0J18

[5] Pololu Corporation, "Pololu USB AVR Programmer v2 User's Guide." [Online]. Available: https://www.pololu.com/docs/0J67

## 10. Weekly log

Columns follow the course's example log book template (everyone should also keep their own).

| Week | Group Member | Working on | How I'll do it | Decisions made | Reasoning | Problems | Solutions |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| 1 | Bea | Sensor research on the Pololu 3pi Robot |  |  |  |  |  |
|  | Jules |  |  |  |  |  |  |
|  | Rosie |  |  |  |  |  |  |
|  | Will |  |  |  |  |  |  |
|  | Ares | Python coding |  |  |  |  |  |
|  | Joe | 
