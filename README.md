# Autonomous-Flight-Simulation-with-LLM-A*-Algorithm
# LLM-Based Strategic Mission Planning and Autonomous Flight Simulation

## Project Overview
I developed an autonomous flight simulation system that integrates a Large Language Model (LLM) as a strategic decision-maker. The system analyzes mission briefings written in natural language to determine optimal flight parameters, which are then used by an A* path planning algorithm and a JSBSim-based flight dynamics engine. The goal was to create a bridge between high-level human intent and low-level autonomous execution.

## System Architecture
The project is built on three main layers:
* **Strategic AI Layer:** I used the Ollama framework to process mission descriptions. The AI evaluates whether a mission is "urgent" or "calm" and generates strategic parameters such as safety distances and maneuverability factors.
* **Path Planning Layer:** I implemented an A* algorithm that works with geodesic coordinates. This planner calculates the most efficient route while avoiding defined dangerous zones and accounting for turn penalties to maintain flight stability.
* **Simulation Layer:** I integrated JSBSim as the flight dynamics model to execute the planned route. The system handles different flight phases including ground, takeoff, waypoint navigation, and descent.

## Technical Implementation
* **llm.py:** Handles the connection to the LLM and parses strategic decisions into JSON format.
* **path_planner.py:** Contains the A* implementation, including bearing calculations and penalty logic for sharp turns.
* **flight_simulation.py:** Manages the interface with JSBSim, controlling aircraft properties and transitioning between flight phases.
* **data.py:** Responsible for logging flight states and converting LLA (Latitude, Longitude, Altitude) coordinates to ENU (East-North-Up) for 3D visualization.

## Limitations and Approximations
I believe in maintaining a realistic perspective on engineering projects. This system contains several approximations and areas for improvement:
* **Dynamic Threats:** The current path planner treats dangerous zones as static circles. Handling moving threats or complex 3D obstacles is not yet implemented.
* **Flight Dynamics Tuning:** While JSBSim provides a robust physics engine, the transition logic between flight phases and the autopilot's response to extreme maneuvers can be inconsistent depending on the aircraft model used.
* **Computational Latency:** The decision-making process depends on the response time of the local LLM. In real-world time-critical scenarios, this latency would need to be optimized or handled with fallback logic.
* **Calculation Precision:** The conversion between geographic and Cartesian coordinates assumes certain local linearities which might introduce small errors over very long-distance flights.
