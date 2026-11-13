# Third Place Solution — ROAR S1/S2 Series

**Author:** Vivian Zhu

This repository contains my third-place solution in the **Berkeley Robot Open Autonomous Racing (ROAR) S1/S2 Series**. It is a fork of the official [ROAR baseline](https://github.com/augcog/ROAR) with an improved PID controller tuned for the Berkeley Major map.

- 📄 Full write-up on the ROAR website: [Third Place Solution in ROAR S1/S2 Series](https://roar.berkeley.edu/third-place-solution-in-summer-2022-roar-s1-s2-series/)
- 🎥 Test run video: [youtube.com/watch?v=F2xxvhO6YVw](https://www.youtube.com/watch?v=F2xxvhO6YVw)

## Where the code is

| File | What it does |
| --- | --- |
| [`ROAR/control_module/pid_v_controller.py`](ROAR/control_module/pid_v_controller.py) | My controller: throttle tuning for risky areas, gradual speed-up after sharp throttle drops, and the "boxing" of known crash zones |
| [`ROAR/agent_module/pid_v_agent.py`](ROAR/agent_module/pid_v_agent.py) | Agent (`PIDVAgent`) that uses the controller above |
| [`runner_competition_evaluator.py`](runner_competition_evaluator.py) | Competition evaluator, set to run `PIDVAgent` |

To run it, set up the ROAR simulator by following the [ROAR quick start](https://augcog.github.io/ROAR/quickstart/), then run:

```bash
python runner_competition_evaluator.py
```

## Introduction

This report summarizes my work in the Berkeley Robot Open Autonomous Racing (ROAR) competition. The ROAR Competition S1/S2 is an autonomous racing competition hosted by the University of California, Berkeley, where high school and college students build and code simulated car models on a virtual Berkeley campus track built by the Berkeley FHL Vive Center for Enhanced Reality. The goal of the competition is to learn and race these cars for maximum speed without collisions. In the future, the learnings from this competition could be useful in real life for controlling electric cars that drive themselves on the streets. During the competition and my study in ROAR Academy, I learned about AI and autonomous driving, which inspired me to take part in this event and to introduce it to my high school's machine learning club.

The map used in this competition is the Berkeley Major map, which has many sharp turns, bumps, and hilly areas likely to cause crashes. Before this competition, I was familiar with Python and some machine learning concepts. Originally, I wanted to try a machine learning approach, but I later realized it was too difficult to finish in time. So I decided to learn the existing PID control method from the baseline and improve on it. I cloned the ROAR GitHub repository and studied the PID controller code. I also learned from the blogs and GitHub repositories of previous winners, which were very helpful.

My approach came from a series of trials and errors to optimize throttle control based on the features of the road map, add a gradual speed-up method, and add location-based, hard-coded speed control based on test run feedback.

## Strategy and Design

The code is based on the PID controller in the [baseline code](https://github.com/augcog/ROAR). A PID controller is a negative feedback loop: the error is fed back as the new input to the system and drives the control outputs (throttle and steering).

<p align="center"><img src="https://i0.wp.com/roar.berkeley.edu/wp-content/uploads/2022/10/Figure_1_PID.jpg?fit=500%2C421&ssl=1" width="400" alt="PID control for autonomous driving"><br><em>Figure 1: PID control for autonomous driving</em></p>

The first step was gathering information about the race track to find where in the map collisions were likely, such as sharp turns or narrow areas. The diagram below plots the path of the waypoints on the track from the sample code.

<p align="center"><img src="https://i0.wp.com/roar.berkeley.edu/wp-content/uploads/2022/10/Figure_2_ROAR_race_track.jpg?fit=548%2C406&ssl=1" width="450" alt="ROAR race track map"><br><em>Figure 2: ROAR race track map</em></p>

From here, I noted the areas where the car might need to slow down to avoid a collision. There are three types of obstacles: sharp turns, big bumps in the hills, and winding roads. Each dot marks a possible collision spot.

The animation below shows a sharp turn that the car must anticipate, lowering the throttle to slow down in order to avoid a collision.

<p align="center"><img src="https://i0.wp.com/roar.berkeley.edu/wp-content/uploads/2022/10/Figure_3_sharp_turns.gif?fit=1200%2C675&ssl=1" width="600" alt="Sharp turn"><br><em>Figure 3: Sharp turns</em></p>

The next animation shows a bumpy area of the track, where I also reduced the throttle in the controller.

<p align="center"><img src="https://i0.wp.com/roar.berkeley.edu/wp-content/uploads/2022/10/Figure_4_bumpy.gif?fit=1200%2C675&ssl=1" width="600" alt="Bumpy area"><br><em>Figure 4: Bumpy area</em></p>

To shorten the lap time further while still avoiding collisions, I analyzed the throttle in the previous blog's sample code more carefully by capturing the throttle values along the whole track, as shown below. The diagram compares my model with the sample model in terms of throttle and speed. My model reduces the throttle less when it reaches a risky area, so the car does not slow down too much. This leads to a higher speed overall along the track.

While gathering this data, I also noticed that when the car makes a big turn in a dangerous area, the previous code drops the throttle to a very low value and keeps it there for a long time. That long duration is probably unnecessary and costs too much speed. To save time, after a sharp throttle drop I increase the throttle little by little instead of keeping it low. The throttle now balances collision avoidance and speed, and this gradual speed-up method cut the running time by several seconds.

<p align="center"><img src="https://i0.wp.com/roar.berkeley.edu/wp-content/uploads/2022/10/Figure_5_throttle.jpg?fit=595%2C491&ssl=1" width="450" alt="Throttle control comparison"><br><em>Figure 5: Throttle control</em></p>

To do this, I built a series of `if` statements in the controller that implement the gradual speed-up. When the car meets a sharp turn, the throttle recovers step by step after its sharp drop in the risky area. The increment is set to 0.3 per step, a value I arrived at after a lot of testing.

<p align="center"><img src="https://i0.wp.com/roar.berkeley.edu/wp-content/uploads/2022/10/Figure_6_gradual.jpg?fit=1200%2C244&ssl=1" width="700" alt="Gradual speed-up code"><br><em>Figure 6: Gradual speed-up code</em></p>

## Testing

Once the design was in place, I ran the program many times to check for collisions and lap time and to optimize further. During a test run with Mr. Tianlun Zhang, he noticed that collisions were very likely around some specific coordinates. He suggested "boxing" those collision spots and reducing the speed inside them. The animation below shows one of these crash spots. Because it has both bumps and winding road, the car crashed there frequently.

<p align="center"><img src="https://i0.wp.com/roar.berkeley.edu/wp-content/uploads/2022/10/Figure_7_Crash.gif?fit=1200%2C675&ssl=1" width="600" alt="Crash in test run"><br><em>Figure 7: Crash in a test run</em></p>

To address this, I recorded the x, y, z coordinates of the collisions. Using those coordinates, I hard-coded `if` statements that box these locations. Inside the dangerous boxes, the throttle is set to 0 to avoid potential collisions.

<p align="center"><img src="https://i0.wp.com/roar.berkeley.edu/wp-content/uploads/2022/10/Figure_8_boxing.jpg?fit=1200%2C396&ssl=1" width="700" alt="Boxing method code"><br><em>Figure 8: Boxing method code</em></p>

## Conclusions and Next Steps

The ROAR Competition was very fulfilling and fun. Based on the trends in my test runs, I tuned the throttle to recover gradually and used a boxing method where collisions happened frequently.

**Strengths**
- High speed with minimal collisions.
- Handles the hardest, most crash-prone spots without sacrificing much speed.

**Limitations**
- The design process was very time-consuming: the car had to be run and tested repeatedly to tune the throttle duration and find the dangerous boxes.
- The hard-coded boxes do not carry over well to other maps.

In the future, I would like to incorporate reinforcement learning into the algorithm so it can generalize to different maps.

## Acknowledgments

A special thanks to **Dr. Allen Yang** for the inspiring ROAR Academy course and for all his support throughout the event, and to **Mr. Tianlun Zhang** for helping test-run my initial code and for suggesting boxing the dangerous areas of the map, which improved the speed further without collisions.

---

## About ROAR

This project is built on the [Robot Open Autonomous Racing (ROAR)](https://roar.berkeley.edu/) platform from UC Berkeley.

- Quick start: [augcog.github.io/ROAR/quickstart](https://augcog.github.io/ROAR/quickstart/)
- Competition instructions: [augcog.github.io/ROAR/competition_instruction](https://augcog.github.io/ROAR/competition_instruction/)
- Upstream repository: [github.com/augcog/ROAR](https://github.com/augcog/ROAR)
