### What is PID Control?

PID stands for **P**roportional, **I**ntegral, and **D**erivative. It's a super common way to make a robot part (like an arm or the drivetrain) get to where you want it to go. The goal is to be fast and accurate, without flying past the target (overshooting) or shaking back and forth (oscillating).

Think about stopping a car right at a stop line.

1.  **Proportional (P) - How far are you?**
    *   **Idea:** The farther you are from the line, the more gas you give it. As you get closer, you let up. The power you use is proportional to how far away you are (your "error").
    *   **Tuning:** This is the `kP` value. A bigger `kP` makes the robot move more aggressively. If it's too big, you'll overshoot the target. If it's too small, the robot will be super slow.

2.  **Integral (I)**
  * We do not use I in robotics so I won't touch on it here. However, it definitely has uses in complex machinery or devices that need to be perfectly accurate.

4.  **Derivative (D) - How fast are you going?**
    *   **Idea:** This part looks at how fast you're approaching the stop line. If you're too fast, it tells you to start braking early to avoid overshooting.
    *   **Tuning:** This is the `kD` value. It helps to smooth things out and reduce the overshoot caused by `kP`. It makes the robot settle down faster. However, too much D will cause the mechanism to start oscillating.

### Why Do We Tune PID?

Tuning just means finding the right `kP` and sometimes `kD` numbers for your specific robot part. Every mechanism is different because of things like weight, friction, and motor power, so you can't just copy and paste values or even take reference.

A well-tuned robot will:
*   Get to the target quickly.
*   Not overshoot much, if at all.
*   Stop at the target without shaking.
*   Hold its position even if it gets bumped.

### How to Tune (A General Guide)

The best way to do this is to graph your robot's position over time. Seeing what's happening is way easier than guessing.

1.  **Set `kI` and `kD` to 0.** We'll start with just P.
2.  **Tune `kP`:**
    *   Start with a small `kP` value.
    *   Tell your robot to go to a target position around the middle of the range of movement.
    *   Keep making `kP` bigger until the robot starts to oscillate around the target at a steady rate.
    *   A good starting `kP` is usually about half of the value that made it oscillate.
3.  **Tune `kD`:**
    *   Using your new `kP`, start adding a little bit of `kD`.
    *   The `kD` should calm down the oscillations.
    *   Increase `kD` until the shaking is gone and the robot settles nicely. Don't add too much, or the robot will become sluggish.
    *   Is not always needed. If you see that you are not getting the performance you want go ahead and tune, but top teams like 2910 just use P control

### WPILib Docs are great

The WPILib documentation is the best place to go for more info. They have great articles and tools for PID tuning.

*   **Intro to PID Control:** [https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html). This is great because it also has simulations where you can experiment digitally tuning different devices. Definitely try it in simulation *before* on a real robot.