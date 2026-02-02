#!/usr/bin/env python3
"""
OPS9 Goal-Based Testing

Tests the positioning system's ability to track motion to a specific goal.
This simulates a robot moving from (0,0) to a target position like (1,1).
"""

import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import sys

# Import the simulator components
sys.path.append('.')
from simulator import SimulatedRobot, PositioningAlgorithm


def move_to_goal(start_pos, goal_pos, max_velocity=0.5, dt=0.005):
    """
    Simulate robot moving from start to goal position

    Args:
        start_pos: (x, y) starting position
        goal_pos: (x, y) target position
        max_velocity: Maximum velocity in m/s
        dt: Time step in seconds

    Returns:
        Dictionary with test results
    """
    print(f"=== OPS9 Goal-Based Test ===")
    print(f"Start: {start_pos}")
    print(f"Goal:  {goal_pos}")
    print(f"Max velocity: {max_velocity} m/s\n")

    # Create robot and algorithm
    robot = SimulatedRobot()
    robot.x, robot.y = start_pos

    algo = PositioningAlgorithm()
    algo.x, algo.y = start_pos

    # Storage for data
    time_data = []
    true_x, true_y, true_heading = [], [], []
    est_x, est_y, est_heading = [], [], []
    distance_to_goal = []

    # Calculate path
    dx = goal_pos[0] - start_pos[0]
    dy = goal_pos[1] - start_pos[1]
    distance = np.sqrt(dx**2 + dy**2)
    target_heading = np.arctan2(dy, dx)

    print(f"Distance to goal: {distance:.3f} m")
    print(f"Target heading: {target_heading*180/np.pi:.1f}°\n")

    # Motion phases
    PHASE_ROTATE = 0  # Rotate to face goal
    PHASE_MOVE = 1    # Move toward goal
    PHASE_STOP = 2    # Stop at goal
    PHASE_DONE = 3    # Finished

    phase = PHASE_ROTATE
    t = 0.0
    step = 0

    print("Starting motion...")

    while phase != PHASE_DONE and t < 60.0:  # Max 60 seconds
        # Calculate current distance to goal
        dist_to_goal = np.sqrt((goal_pos[0] - robot.x)**2 + (goal_pos[1] - robot.y)**2)

        # Phase-based control
        if phase == PHASE_ROTATE:
            # Rotate to face goal
            heading_error = target_heading - robot.heading
            # Normalize to [-pi, pi]
            while heading_error > np.pi:
                heading_error -= 2*np.pi
            while heading_error < -np.pi:
                heading_error += 2*np.pi

            if abs(heading_error) > 0.1:  # 5.7 degrees
                # Rotate toward target
                angular_accel = 0.5 if heading_error > 0 else -0.5
                linear_accel = 0.0
            else:
                # Aligned, start moving
                phase = PHASE_MOVE
                print(f"t={t:.1f}s: Aligned to target, starting motion")
                angular_accel = 0.0
                linear_accel = 0.0

        elif phase == PHASE_MOVE:
            # Move toward goal
            if dist_to_goal > 0.1:  # More than 10cm away
                # Accelerate if below max velocity
                if robot.velocity < max_velocity:
                    linear_accel = 0.5
                else:
                    linear_accel = 0.0

                # Small heading corrections
                heading_error = target_heading - robot.heading
                while heading_error > np.pi:
                    heading_error -= 2*np.pi
                while heading_error < -np.pi:
                    heading_error += 2*np.pi

                angular_accel = 0.1 * heading_error  # Proportional control
            else:
                # Close to goal, start stopping
                phase = PHASE_STOP
                print(f"t={t:.1f}s: Reached goal, stopping")
                linear_accel = -1.0
                angular_accel = 0.0

        elif phase == PHASE_STOP:
            # Decelerate to stop
            if robot.velocity > 0.01:
                linear_accel = -1.0
                angular_accel = 0.0
            else:
                # Stopped
                phase = PHASE_DONE
                print(f"t={t:.1f}s: Stopped at goal")
                linear_accel = 0.0
                angular_accel = 0.0

        # Update robot physics
        robot.update(dt, linear_accel, angular_accel)

        # Get sensor readings
        imu_data = robot.get_imu_reading()
        encoder_data = robot.get_encoder_reading()

        # Update algorithm
        est_state = algo.update(imu_data, encoder_data, dt)

        # Store data (every 10th sample)
        if step % 10 == 0:
            time_data.append(t)
            true_x.append(robot.x)
            true_y.append(robot.y)
            true_heading.append(robot.heading)
            est_x.append(est_state['x'])
            est_y.append(est_state['y'])
            est_heading.append(est_state['heading'])
            distance_to_goal.append(dist_to_goal)

        # Print progress
        if step % 200 == 0:
            pos_err = np.sqrt((robot.x - est_state['x'])**2 + (robot.y - est_state['y'])**2)
            print(f"t={t:.1f}s: True=({robot.x:.3f},{robot.y:.3f}), "
                  f"Est=({est_state['x']:.3f},{est_state['y']:.3f}), "
                  f"Err={pos_err:.3f}m, Dist to goal={dist_to_goal:.3f}m")

        t += dt
        step += 1

    # Calculate final errors
    final_pos_error = np.sqrt((robot.x - algo.x)**2 + (robot.y - algo.y)**2)
    final_heading_error = abs(robot.heading - algo.heading) * 180 / np.pi

    # Distance from goal (true position)
    true_goal_error = np.sqrt((robot.x - goal_pos[0])**2 + (robot.y - goal_pos[1])**2)
    # Distance from goal (estimated position)
    est_goal_error = np.sqrt((algo.x - goal_pos[0])**2 + (algo.y - goal_pos[1])**2)

    print(f"\n=== Test Complete ===")
    print(f"Time taken: {t:.1f}s")
    print(f"True final position: ({robot.x:.4f}, {robot.y:.4f})")
    print(f"Estimated final position: ({algo.x:.4f}, {algo.y:.4f})")
    print(f"Goal position: {goal_pos}")
    print(f"\nPosition tracking error: {final_pos_error:.4f} m")
    print(f"Heading error: {final_heading_error:.2f}°")
    print(f"True distance from goal: {true_goal_error:.4f} m")
    print(f"Estimated distance from goal: {est_goal_error:.4f} m")

    # Evaluate performance
    print(f"\n=== Performance Evaluation ===")
    if true_goal_error < 0.05:
        print(f"✅ Goal reached! (within 5cm)")
    elif true_goal_error < 0.10:
        print(f"✅ Goal reached! (within 10cm)")
    else:
        print(f"⚠️  Missed goal by {true_goal_error*100:.1f}cm")

    if final_pos_error < 0.02:
        print(f"✅ Excellent position tracking (<2cm error)")
    elif final_pos_error < 0.05:
        print(f"✅ Good position tracking (<5cm error)")
    elif final_pos_error < 0.10:
        print(f"⚠️  Acceptable position tracking (<10cm error)")
    else:
        print(f"❌ Poor position tracking (>{final_pos_error*100:.0f}cm error)")

    # Plot results
    plot_goal_test(time_data, true_x, true_y, est_x, est_y,
                   goal_pos, distance_to_goal, final_pos_error, true_goal_error)

    return {
        'time': t,
        'true_pos': (robot.x, robot.y),
        'est_pos': (algo.x, algo.y),
        'pos_error': final_pos_error,
        'goal_error': true_goal_error,
        'est_goal_error': est_goal_error
    }


def plot_goal_test(time_data, true_x, true_y, est_x, est_y,
                   goal_pos, distance_to_goal, pos_error, goal_error):
    """Plot goal-based test results"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Trajectory with goal
    ax = axes[0, 0]
    ax.plot(true_x, true_y, 'b-', label='True path', linewidth=2)
    ax.plot(est_x, est_y, 'r--', label='Estimated path', linewidth=2)
    ax.plot(true_x[0], true_y[0], 'go', markersize=15, label='Start', zorder=5)
    ax.plot(goal_pos[0], goal_pos[1], 'r*', markersize=20, label='Goal', zorder=5)
    ax.plot(true_x[-1], true_y[-1], 'bs', markersize=10, label='Final (true)', zorder=5)
    ax.plot(est_x[-1], est_y[-1], 'r^', markersize=10, label='Final (est)', zorder=5)

    # Draw goal circle (10cm radius)
    circle = plt.Circle(goal_pos, 0.1, color='red', fill=False, linestyle='--', linewidth=2)
    ax.add_patch(circle)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'Trajectory to Goal\nFinal error: {pos_error*100:.1f}cm, Goal miss: {goal_error*100:.1f}cm')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')

    # Distance to goal over time
    ax = axes[0, 1]
    ax.plot(time_data, distance_to_goal, 'g-', linewidth=2)
    ax.axhline(y=0.1, color='r', linestyle='--', label='10cm threshold')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance to Goal (m)')
    ax.set_title('Distance to Goal Over Time')
    ax.legend()
    ax.grid(True)

    # Position error over time
    ax = axes[1, 0]
    pos_errors = [np.sqrt((tx-ex)**2 + (ty-ey)**2)
                  for tx, ty, ex, ey in zip(true_x, true_y, est_x, est_y)]
    ax.plot(time_data, np.array(pos_errors)*100, 'r-', linewidth=2)
    ax.axhline(y=2, color='g', linestyle='--', label='2cm target')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (cm)')
    ax.set_title('Position Tracking Error')
    ax.legend()
    ax.grid(True)

    # X and Y position comparison
    ax = axes[1, 1]
    ax.plot(time_data, true_x, 'b-', label='True X', linewidth=2)
    ax.plot(time_data, est_x, 'b--', label='Est X', linewidth=2)
    ax.plot(time_data, true_y, 'r-', label='True Y', linewidth=2)
    ax.plot(time_data, est_y, 'r--', label='Est Y', linewidth=2)
    ax.axhline(y=goal_pos[0], color='b', linestyle=':', alpha=0.5, label=f'Goal X={goal_pos[0]}')
    ax.axhline(y=goal_pos[1], color='r', linestyle=':', alpha=0.5, label=f'Goal Y={goal_pos[1]}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position (m)')
    ax.set_title('X and Y Position Over Time')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.savefig('goal_test_results.png', dpi=150)
    print("\nPlot saved to: goal_test_results.png")
    plt.show()


def run_multiple_goals():
    """Test multiple goal positions"""
    print("=== Testing Multiple Goals ===\n")

    goals = [
        ((0, 0), (1, 0)),    # Move 1m forward
        ((0, 0), (1, 1)),    # Move to (1,1) - your example
        ((0, 0), (0, 1)),    # Move 1m left
        ((0, 0), (2, 2)),    # Move to (2,2) - longer distance
    ]

    results = []
    for start, goal in goals:
        print(f"\n{'='*60}")
        result = move_to_goal(start, goal)
        results.append((goal, result))
        print(f"{'='*60}\n")

    # Summary
    print("\n=== Summary of All Tests ===")
    print(f"{'Goal':<15} {'Time':<8} {'Pos Error':<12} {'Goal Error':<12} {'Status'}")
    print("-" * 70)
    for goal, result in results:
        status = "✅ PASS" if result['goal_error'] < 0.1 else "⚠️  MISS"
        print(f"{str(goal):<15} {result['time']:>6.1f}s  "
              f"{result['pos_error']*100:>6.1f}cm      "
              f"{result['goal_error']*100:>6.1f}cm      {status}")


if __name__ == '__main__':
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == '--multiple':
        run_multiple_goals()
    else:
        # Single goal test: (0,0) -> (1,1)
        move_to_goal((0, 0), (1, 1))
