#!/usr/bin/env python3
"""
OPS9 Realistic Goal Test

This test simulates what happens when YOUR robot controller commands the robot
to move from (0,0) to (1,1), and we measure how accurately OPS9 tracks it.

OPS9 is a POSITIONING system, not a navigation system. It tells you WHERE you are,
not HOW to get somewhere. Your robot controller uses OPS9's position to navigate.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
sys.path.append('.')
from simulator import SimulatedRobot, PositioningAlgorithm


def test_straight_line_to_goal():
    """
    Test: Robot moves in a straight line from (0,0) to (1,1)
    This simulates a perfect robot controller that drives straight to the goal.
    We measure how accurately OPS9 tracks this motion.
    """
    print("=== OPS9 Straight Line Test: (0,0) → (1,1) ===\n")

    # Setup
    robot = SimulatedRobot()
    algo = PositioningAlgorithm()
    dt = 0.005  # 5ms

    # Calculate straight path to goal
    goal = np.array([1.0, 1.0])
    distance = np.linalg.norm(goal)
    direction = goal / distance  # Unit vector toward goal

    # Motion parameters
    max_velocity = 0.3  # m/s (slow and steady)
    accel_time = 1.0    # Accelerate for 1 second
    decel_time = 1.0    # Decelerate for 1 second

    # Calculate heading to goal
    target_heading = np.arctan2(goal[1], goal[0])  # 45 degrees

    print(f"Goal: {goal}")
    print(f"Distance: {distance:.3f} m")
    print(f"Target heading: {target_heading*180/np.pi:.1f}°")
    print(f"Max velocity: {max_velocity} m/s\n")

    # Storage
    time_data, true_x, true_y, est_x, est_y = [], [], [], [], []
    pos_errors = []

    # Phase 1: Rotate to face goal (first 2 seconds)
    print("Phase 1: Rotating to face goal...")
    t = 0.0
    while t < 2.0:
        heading_error = target_heading - robot.heading
        while heading_error > np.pi: heading_error -= 2*np.pi
        while heading_error < -np.pi: heading_error += 2*np.pi

        angular_accel = 0.5 if heading_error > 0 else -0.5
        robot.update(dt, 0.0, angular_accel)

        imu_data = robot.get_imu_reading()
        encoder_data = robot.get_encoder_reading()
        est_state = algo.update(imu_data, encoder_data, dt)

        t += dt

    print(f"  Aligned! Heading: {robot.heading*180/np.pi:.1f}°\n")

    # Phase 2: Accelerate toward goal
    print("Phase 2: Accelerating...")
    while t < 2.0 + accel_time:
        robot.update(dt, 0.5, 0.0)  # Accelerate forward
        imu_data = robot.get_imu_reading()
        encoder_data = robot.get_encoder_reading()
        est_state = algo.update(imu_data, encoder_data, dt)
        t += dt

    print(f"  Velocity: {robot.velocity:.2f} m/s\n")

    # Phase 3: Cruise at constant velocity
    print("Phase 3: Cruising to goal...")
    cruise_distance = distance - (max_velocity**2 / (2 * 0.5))  # Leave room to stop
    cruise_time = cruise_distance / max_velocity

    step = 0
    while t < 2.0 + accel_time + cruise_time:
        robot.update(dt, 0.0, 0.0)  # Constant velocity

        imu_data = robot.get_imu_reading()
        encoder_data = robot.get_encoder_reading()
        est_state = algo.update(imu_data, encoder_data, dt)

        # Store data every 10 steps
        if step % 10 == 0:
            time_data.append(t)
            true_x.append(robot.x)
            true_y.append(robot.y)
            est_x.append(est_state['x'])
            est_y.append(est_state['y'])
            pos_err = np.sqrt((robot.x - est_state['x'])**2 + (robot.y - est_state['y'])**2)
            pos_errors.append(pos_err)

        # Print progress
        if step % 200 == 0:
            dist_to_goal = np.linalg.norm(goal - np.array([robot.x, robot.y]))
            pos_err = np.sqrt((robot.x - est_state['x'])**2 + (robot.y - est_state['y'])**2)
            print(f"  t={t:.1f}s: pos=({robot.x:.3f},{robot.y:.3f}), "
                  f"err={pos_err*100:.1f}cm, dist to goal={dist_to_goal:.3f}m")

        t += dt
        step += 1

    # Phase 4: Decelerate to stop at goal
    print("\nPhase 4: Decelerating...")
    while robot.velocity > 0.01:
        robot.update(dt, -0.5, 0.0)  # Decelerate

        imu_data = robot.get_imu_reading()
        encoder_data = robot.get_encoder_reading()
        est_state = algo.update(imu_data, encoder_data, dt)

        if step % 10 == 0:
            time_data.append(t)
            true_x.append(robot.x)
            true_y.append(robot.y)
            est_x.append(est_state['x'])
            est_y.append(est_state['y'])
            pos_err = np.sqrt((robot.x - est_state['x'])**2 + (robot.y - est_state['y'])**2)
            pos_errors.append(pos_err)

        t += dt
        step += 1

    # Final results
    print(f"\n{'='*60}")
    print("=== FINAL RESULTS ===")
    print(f"{'='*60}")

    true_final = np.array([robot.x, robot.y])
    est_final = np.array([algo.x, algo.y])

    print(f"\nGoal position:      {goal}")
    print(f"True final position: ({robot.x:.4f}, {robot.y:.4f})")
    print(f"Est final position:  ({algo.x:.4f}, {algo.y:.4f})")

    # Calculate errors
    true_goal_error = np.linalg.norm(true_final - goal)
    est_goal_error = np.linalg.norm(est_final - goal)
    tracking_error = np.linalg.norm(true_final - est_final)
    avg_tracking_error = np.mean(pos_errors)
    max_tracking_error = np.max(pos_errors)

    print(f"\n--- Robot Performance (how well it reached goal) ---")
    print(f"Distance from goal: {true_goal_error*100:.1f} cm")

    print(f"\n--- OPS9 Performance (how well it tracked motion) ---")
    print(f"Final tracking error:   {tracking_error*100:.1f} cm")
    print(f"Average tracking error: {avg_tracking_error*100:.1f} cm")
    print(f"Max tracking error:     {max_tracking_error*100:.1f} cm")
    print(f"Error per meter:        {tracking_error/distance*100:.1f} cm/m")

    # Evaluation
    print(f"\n{'='*60}")
    print("=== EVALUATION ===")
    print(f"{'='*60}\n")

    if true_goal_error < 0.05:
        print("✅ Robot reached goal (within 5cm)")
    elif true_goal_error < 0.10:
        print("✅ Robot reached goal (within 10cm)")
    else:
        print(f"⚠️  Robot missed goal by {true_goal_error*100:.1f}cm")

    if tracking_error < 0.02:
        print("✅ EXCELLENT: OPS9 tracking error < 2cm")
    elif tracking_error < 0.05:
        print("✅ GOOD: OPS9 tracking error < 5cm")
    elif tracking_error < 0.10:
        print("✅ ACCEPTABLE: OPS9 tracking error < 10cm")
    else:
        print(f"⚠️  OPS9 tracking error: {tracking_error*100:.0f}cm")

    error_per_meter = tracking_error / distance
    if error_per_meter < 0.02:
        print(f"✅ EXCELLENT: {error_per_meter*100:.2f} cm/m (target: <2 cm/m)")
    else:
        print(f"⚠️  Error rate: {error_per_meter*100:.2f} cm/m (target: <2 cm/m)")

    # Plot
    plot_results(time_data, true_x, true_y, est_x, est_y, goal, pos_errors,
                 true_goal_error, tracking_error)

    return {
        'goal_error': true_goal_error,
        'tracking_error': tracking_error,
        'avg_error': avg_tracking_error,
        'error_per_meter': error_per_meter
    }


def plot_results(time_data, true_x, true_y, est_x, est_y, goal, pos_errors,
                 goal_error, tracking_error):
    """Plot test results"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Trajectory
    ax = axes[0, 0]
    ax.plot(true_x, true_y, 'b-', label='True path', linewidth=3)
    ax.plot(est_x, est_y, 'r--', label='OPS9 estimate', linewidth=2)
    ax.plot(0, 0, 'go', markersize=15, label='Start', zorder=5)
    ax.plot(goal[0], goal[1], 'r*', markersize=25, label='Goal', zorder=5)
    ax.plot(true_x[-1], true_y[-1], 'bs', markersize=12, label='Final (true)', zorder=5)
    ax.plot(est_x[-1], est_y[-1], 'r^', markersize=12, label='Final (OPS9)', zorder=5)

    # Goal circle
    circle = plt.Circle(goal, 0.05, color='green', fill=False, linestyle='--', linewidth=2, label='5cm target')
    ax.add_patch(circle)

    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title(f'Path from (0,0) to (1,1)\nGoal miss: {goal_error*100:.1f}cm, Tracking error: {tracking_error*100:.1f}cm',
                 fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')

    # Tracking error over time
    ax = axes[0, 1]
    ax.plot(time_data, np.array(pos_errors)*100, 'r-', linewidth=2)
    ax.axhline(y=2, color='g', linestyle='--', linewidth=2, label='2cm target')
    ax.axhline(y=5, color='orange', linestyle='--', linewidth=2, label='5cm acceptable')
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Tracking Error (cm)', fontsize=12)
    ax.set_title('OPS9 Position Tracking Error', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    # X and Y comparison
    ax = axes[1, 0]
    ax.plot(time_data, true_x, 'b-', label='True X', linewidth=2)
    ax.plot(time_data, est_x, 'b--', label='OPS9 X', linewidth=2, alpha=0.7)
    ax.plot(time_data, true_y, 'r-', label='True Y', linewidth=2)
    ax.plot(time_data, est_y, 'r--', label='OPS9 Y', linewidth=2, alpha=0.7)
    ax.axhline(y=goal[0], color='b', linestyle=':', alpha=0.5)
    ax.axhline(y=goal[1], color='r', linestyle=':', alpha=0.5)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Position (m)', fontsize=12)
    ax.set_title('X and Y Position Over Time', fontsize=12, fontweight='bold')
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    # Error statistics
    ax = axes[1, 1]
    ax.text(0.1, 0.9, 'OPS9 Performance Summary', fontsize=14, fontweight='bold',
            transform=ax.transAxes)
    ax.text(0.1, 0.75, f'Final tracking error: {tracking_error*100:.2f} cm',
            fontsize=12, transform=ax.transAxes)
    ax.text(0.1, 0.65, f'Average error: {np.mean(pos_errors)*100:.2f} cm',
            fontsize=12, transform=ax.transAxes)
    ax.text(0.1, 0.55, f'Max error: {np.max(pos_errors)*100:.2f} cm',
            fontsize=12, transform=ax.transAxes)
    ax.text(0.1, 0.45, f'Error per meter: {tracking_error/1.414*100:.2f} cm/m',
            fontsize=12, transform=ax.transAxes)
    ax.text(0.1, 0.35, f'Target: < 2 cm/m', fontsize=12, transform=ax.transAxes,
            color='green' if tracking_error/1.414 < 0.02 else 'orange')

    ax.text(0.1, 0.2, 'Robot Performance', fontsize=14, fontweight='bold',
            transform=ax.transAxes)
    ax.text(0.1, 0.05, f'Distance from goal: {goal_error*100:.2f} cm',
            fontsize=12, transform=ax.transAxes)

    ax.axis('off')

    plt.tight_layout()
    plt.savefig('goal_test_results.png', dpi=150, bbox_inches='tight')
    print("\n📊 Plot saved to: goal_test_results.png")
    plt.show()


if __name__ == '__main__':
    print("\n" + "="*60)
    print("  OPS9 POSITIONING SYSTEM TEST")
    print("  Testing motion from (0,0) to (1,1)")
    print("="*60 + "\n")

    result = test_straight_line_to_goal()

    print("\n" + "="*60)
    print("Test complete! Check goal_test_results.png for visualization")
    print("="*60 + "\n")
