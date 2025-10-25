#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
import math

try:
    from mcap.reader import make_reader
    from mcap_ros2.reader import read_ros2_messages
    MCAP_AVAILABLE = True
except ImportError:
    MCAP_AVAILABLE = False
    print("Error: Install with: pip3 install mcap mcap-ros2-support")


class BagAnalyzer:
    def __init__(self, bag_path):
        self.bag_path = bag_path
        self.timestamps = []
        self.x_positions = []
        self.y_positions = []
        self.yaw_angles = []
        self.linear_velocities = []
        self.angular_velocities = []
        
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle in radians"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def read_bag(self):
        """Read MCAP bag file"""
        if not MCAP_AVAILABLE:
            print("Error: mcap library not installed")
            return False
        
        try:
            print(f"Reading: {self.bag_path}")
            
            with open(self.bag_path, "rb") as f:
                reader = make_reader(f)
                start_time = None
                message_count = 0
                
                # Iterate through messages
                for msg_data in read_ros2_messages(f):
                    # msg_data is a McapROS2Message object with attributes
                    channel = msg_data.channel
                    message = msg_data.message
                    ros_msg = msg_data.ros_msg
                    
                    # Only process /odom messages
                    if channel.topic == "/odom":
                        if start_time is None:
                            start_time = message.log_time
                        
                        relative_time = (message.log_time - start_time) / 1e9
                        self.timestamps.append(relative_time)
                        self.x_positions.append(ros_msg.pose.pose.position.x)
                        self.y_positions.append(ros_msg.pose.pose.position.y)
                        self.yaw_angles.append(self.quaternion_to_yaw(ros_msg.pose.pose.orientation))
                        self.linear_velocities.append(ros_msg.twist.twist.linear.x)
                        self.angular_velocities.append(ros_msg.twist.twist.angular.z)
                        
                        message_count += 1
            
            print(f"✓ Read {message_count} messages")
            return message_count > 0
            
        except Exception as e:
            print(f"✗ Error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def calculate_metrics(self):
        """Calculate straightness metrics"""
        if len(self.x_positions) < 2:
            return None
        
        x = np.array(self.x_positions)
        y = np.array(self.y_positions)
        
        start_point = np.array([x[0], y[0]])
        end_point = np.array([x[-1], y[-1]])
        
        lateral_deviations = []
        for i in range(len(x)):
            point = np.array([x[i], y[i]])
            line_vec = end_point - start_point
            point_vec = point - start_point
            line_len = np.linalg.norm(line_vec)
            if line_len > 0:
                line_unitvec = line_vec / line_len
                projection = np.dot(point_vec, line_unitvec)
                closest_point = start_point + projection * line_unitvec
                deviation = np.linalg.norm(point - closest_point)
                lateral_deviations.append(deviation)
        
        return {
            'total_distance': np.linalg.norm(end_point - start_point),
            'max_lateral_deviation': np.max(lateral_deviations),
            'mean_lateral_deviation': np.mean(lateral_deviations),
            'std_lateral_deviation': np.std(lateral_deviations),
            'final_x': x[-1],
            'final_y': y[-1],
            'yaw_variation': np.std(self.yaw_angles),
            'lateral_deviations': lateral_deviations
        }


def find_mcap_files():
    """Find valid MCAP files in current directory"""
    mcap_files = []
    for f in os.listdir('.'):
        if f.endswith('.mcap'):
            size = os.path.getsize(f)
            if size > 10000:  # At least 10KB
                mcap_files.append(f)
    return sorted(mcap_files)


def plot_comparison(openloop, pcontrol, openloop_name, pcontrol_name):
    """Create comprehensive comparison plots"""
    
    openloop_metrics = openloop.calculate_metrics()
    pcontrol_metrics = pcontrol.calculate_metrics()
    
    if not openloop_metrics or not pcontrol_metrics:
        print("Error: Unable to calculate metrics")
        return
    
    fig = plt.figure(figsize=(18, 12))
    fig.suptitle(f'Open Loop vs P-Control Comparison', 
                 fontsize=16, fontweight='bold', y=0.995)
    
    # 1. XY Trajectory
    ax1 = plt.subplot(3, 3, 1)
    ax1.plot(openloop.x_positions, openloop.y_positions, 
             'r-', linewidth=2.5, label='Open Loop', alpha=0.8)
    ax1.plot(pcontrol.x_positions, pcontrol.y_positions, 
             'b-', linewidth=2.5, label='P-Control', alpha=0.8)
    ax1.plot([openloop.x_positions[0], openloop.x_positions[-1]], 
            [openloop.y_positions[0], openloop.y_positions[-1]], 
            'k--', linewidth=1.5, label='Ideal', alpha=0.6)
    ax1.scatter([openloop.x_positions[0]], [openloop.y_positions[0]], 
               c='green', s=100, marker='o', label='Start', zorder=5)
    ax1.set_xlabel('X Position (m)', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Y Position (m)', fontsize=11, fontweight='bold')
    ax1.set_title('Trajectory Comparison', fontsize=13, fontweight='bold')
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')
    
    # 2. X Position vs Time
    ax2 = plt.subplot(3, 3, 2)
    ax2.plot(openloop.timestamps, openloop.x_positions, 
             'r-', linewidth=2, label='Open Loop', alpha=0.8)
    ax2.plot(pcontrol.timestamps, pcontrol.x_positions, 
             'b-', linewidth=2, label='P-Control', alpha=0.8)
    ax2.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax2.set_ylabel('X Position (m)', fontsize=11, fontweight='bold')
    ax2.set_title('Forward Progress', fontsize=13, fontweight='bold')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)
    
    # 3. Y Position vs Time
    ax3 = plt.subplot(3, 3, 3)
    ax3.plot(openloop.timestamps, openloop.y_positions, 
             'r-', linewidth=2, label='Open Loop', alpha=0.8)
    ax3.plot(pcontrol.timestamps, pcontrol.y_positions, 
             'b-', linewidth=2, label='P-Control', alpha=0.8)
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=1.5, alpha=0.6)
    ax3.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax3.set_ylabel('Y Position (m)', fontsize=11, fontweight='bold')
    ax3.set_title('Lateral Drift', fontsize=13, fontweight='bold')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3)
    
    # 4. Yaw Angle
    ax4 = plt.subplot(3, 3, 4)
    ax4.plot(openloop.timestamps, np.degrees(openloop.yaw_angles), 
             'r-', linewidth=2, label='Open Loop', alpha=0.8)
    ax4.plot(pcontrol.timestamps, np.degrees(pcontrol.yaw_angles), 
             'b-', linewidth=2, label='P-Control', alpha=0.8)
    ax4.axhline(y=0, color='k', linestyle='--', linewidth=1.5, alpha=0.6)
    ax4.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax4.set_ylabel('Yaw Angle (°)', fontsize=11, fontweight='bold')
    ax4.set_title('Heading Angle', fontsize=13, fontweight='bold')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)
    
    # 5. Linear Velocity
    ax5 = plt.subplot(3, 3, 5)
    ax5.plot(openloop.timestamps, openloop.linear_velocities, 
             'r-', linewidth=2, label='Open Loop', alpha=0.8)
    ax5.plot(pcontrol.timestamps, pcontrol.linear_velocities, 
             'b-', linewidth=2, label='P-Control', alpha=0.8)
    ax5.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax5.set_ylabel('Linear Velocity (m/s)', fontsize=11, fontweight='bold')
    ax5.set_title('Forward Velocity', fontsize=13, fontweight='bold')
    ax5.legend(fontsize=9)
    ax5.grid(True, alpha=0.3)
    
    # 6. Angular Velocity
    ax6 = plt.subplot(3, 3, 6)
    ax6.plot(openloop.timestamps, np.degrees(openloop.angular_velocities), 
             'r-', linewidth=2, label='Open Loop', alpha=0.8)
    ax6.plot(pcontrol.timestamps, np.degrees(pcontrol.angular_velocities), 
             'b-', linewidth=2, label='P-Control', alpha=0.8)
    ax6.axhline(y=0, color='k', linestyle='--', linewidth=1.5, alpha=0.6)
    ax6.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax6.set_ylabel('Angular Velocity (°/s)', fontsize=11, fontweight='bold')
    ax6.set_title('Corrective Rotation', fontsize=13, fontweight='bold')
    ax6.legend(fontsize=9)
    ax6.grid(True, alpha=0.3)
    
    # 7. Lateral Deviation Distribution
    ax7 = plt.subplot(3, 3, 7)
    ax7.hist(openloop_metrics['lateral_deviations'], bins=30, alpha=0.6, 
            color='red', label='Open Loop', edgecolor='darkred', linewidth=1.2)
    ax7.hist(pcontrol_metrics['lateral_deviations'], bins=30, alpha=0.6, 
            color='blue', label='P-Control', edgecolor='darkblue', linewidth=1.2)
    ax7.set_xlabel('Lateral Deviation (m)', fontsize=11, fontweight='bold')
    ax7.set_ylabel('Frequency', fontsize=11, fontweight='bold')
    ax7.set_title('Drift Distribution', fontsize=13, fontweight='bold')
    ax7.legend(fontsize=9)
    ax7.grid(True, alpha=0.3)
    
    # 8. Absolute Lateral Deviation
    ax8 = plt.subplot(3, 3, 8)
    ax8.plot(openloop.timestamps, 
            np.abs(openloop_metrics['lateral_deviations']), 
            'r-', linewidth=2, label='Open Loop', alpha=0.8)
    ax8.plot(pcontrol.timestamps[:len(pcontrol_metrics['lateral_deviations'])], 
            np.abs(pcontrol_metrics['lateral_deviations']), 
            'b-', linewidth=2, label='P-Control', alpha=0.8)
    ax8.set_xlabel('Time (s)', fontsize=11, fontweight='bold')
    ax8.set_ylabel('|Lateral Deviation| (m)', fontsize=11, fontweight='bold')
    ax8.set_title('Absolute Drift Over Time', fontsize=13, fontweight='bold')
    ax8.legend(fontsize=9)
    ax8.grid(True, alpha=0.3)
    
    # 9. Metrics Table
    ax9 = plt.subplot(3, 3, 9)
    ax9.axis('off')
    
    max_improvement = ((openloop_metrics['max_lateral_deviation'] - 
                       pcontrol_metrics['max_lateral_deviation']) / 
                       openloop_metrics['max_lateral_deviation'] * 100) if openloop_metrics['max_lateral_deviation'] > 0 else 0
    mean_improvement = ((openloop_metrics['mean_lateral_deviation'] - 
                        pcontrol_metrics['mean_lateral_deviation']) / 
                        openloop_metrics['mean_lateral_deviation'] * 100) if openloop_metrics['mean_lateral_deviation'] > 0 else 0
    
    table_text = f"""
PERFORMANCE METRICS

{'Metric':<22} {'Open':<10} {'P-Ctrl':<10}
{'-'*44}
{'Distance (m)':<22} {openloop_metrics['total_distance']:<10.3f} {pcontrol_metrics['total_distance']:<10.3f}
{'Max Drift (m)':<22} {openloop_metrics['max_lateral_deviation']:<10.4f} {pcontrol_metrics['max_lateral_deviation']:<10.4f}
{'Avg Drift (m)':<22} {openloop_metrics['mean_lateral_deviation']:<10.4f} {pcontrol_metrics['mean_lateral_deviation']:<10.4f}
{'Std Drift (m)':<22} {openloop_metrics['std_lateral_deviation']:<10.4f} {pcontrol_metrics['std_lateral_deviation']:<10.4f}
{'Final X (m)':<22} {openloop_metrics['final_x']:<10.3f} {pcontrol_metrics['final_x']:<10.3f}
{'Final Y (m)':<22} {openloop_metrics['final_y']:<10.4f} {pcontrol_metrics['final_y']:<10.4f}
{'Yaw StdDev (rad)':<22} {openloop_metrics['yaw_variation']:<10.4f} {pcontrol_metrics['yaw_variation']:<10.4f}

IMPROVEMENT:
{'-'*44}
Max Drift:     {max_improvement:>6.1f}% reduction
Avg Drift:     {mean_improvement:>6.1f}% reduction
    """
    ax9.text(0.05, 0.95, table_text, fontsize=9, family='monospace', 
            verticalalignment='top', transform=ax9.transAxes,
            bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.3))
    
    plt.tight_layout()
    plt.savefig('controller_comparison.png', dpi=300, bbox_inches='tight')
    print(f"\n✓ Saved: controller_comparison.png")
    plt.show()


def find_mcap_files():
    """Find valid MCAP files in current directory"""
    mcap_files = []
    for f in os.listdir('.'):
        if f.endswith('.mcap'):
            size = os.path.getsize(f)
            if size > 10000:  # At least 10KB
                mcap_files.append(f)
    return sorted(mcap_files)


def main():
    print("="*70)
    print("  ROS 2 Bag Comparison: Open Loop vs P-Control")
    print("="*70)
    
    if not MCAP_AVAILABLE:
        print("\nMissing packages! Install with:")
        print("  pip3 install mcap mcap-ros2-support numpy matplotlib")
        return
    
    # Find MCAP files
    print("\n>> Searching for .mcap files...")
    mcap_files = find_mcap_files()
    
    if len(mcap_files) == 0:
        print("✗ No valid .mcap files found")
        return
    
    for f in mcap_files:
        size = os.path.getsize(f) / (1024 * 1024)
        print(f"  ✓ {f} ({size:.2f} MB)")
    
    if len(mcap_files) < 2:
        print(f"\n✗ Found only {len(mcap_files)} file(s). Need 2 for comparison.")
        return
    
    # Auto-select files
    openloop_file = mcap_files[0]
    pcontrol_file = mcap_files[1]
    
    print(f"\n>> Auto-selected:")
    print(f"  [Open Loop] {openloop_file}")
    print(f"  [P-Control] {pcontrol_file}")
    
    # Analyze Open Loop
    print(f"\n{'-'*70}")
    print(f"Analyzing Open Loop...")
    print(f"{'-'*70}")
    openloop = BagAnalyzer(openloop_file)
    if not openloop.read_bag():
        return
    
    # Analyze P-Control
    print(f"\n{'-'*70}")
    print(f"Analyzing P-Control...")
    print(f"{'-'*70}")
    pcontrol = BagAnalyzer(pcontrol_file)
    if not pcontrol.read_bag():
        return
    
    # Generate plots
    print(f"\n{'='*70}")
    print("Generating plots...")
    print(f"{'='*70}")
    
    plot_comparison(openloop, pcontrol, openloop_file, pcontrol_file)
    
    # Print summary
    openloop_metrics = openloop.calculate_metrics()
    pcontrol_metrics = pcontrol.calculate_metrics()
    
    print(f"\n{'='*70}")
    print("SUMMARY STATISTICS")
    print(f"{'='*70}")
    print(f"\nOpen Loop:")
    print(f"  Distance:      {openloop_metrics['total_distance']:.3f} m")
    print(f"  Max Drift:     {openloop_metrics['max_lateral_deviation']:.4f} m")
    print(f"  Avg Drift:     {openloop_metrics['mean_lateral_deviation']:.4f} m")
    print(f"  Final Pos:     ({openloop_metrics['final_x']:.3f}, {openloop_metrics['final_y']:.4f}) m")
    
    print(f"\nP-Control:")
    print(f"  Distance:      {pcontrol_metrics['total_distance']:.3f} m")
    print(f"  Max Drift:     {pcontrol_metrics['max_lateral_deviation']:.4f} m")
    print(f"  Avg Drift:     {pcontrol_metrics['mean_lateral_deviation']:.4f} m")
    print(f"  Final Pos:     ({pcontrol_metrics['final_x']:.3f}, {pcontrol_metrics['final_y']:.4f}) m")
    
    max_improvement = ((openloop_metrics['max_lateral_deviation'] - 
                       pcontrol_metrics['max_lateral_deviation']) / 
                       openloop_metrics['max_lateral_deviation'] * 100)
    mean_improvement = ((openloop_metrics['mean_lateral_deviation'] - 
                        pcontrol_metrics['mean_lateral_deviation']) / 
                        openloop_metrics['mean_lateral_deviation'] * 100)
    
    print(f"\n{'='*70}")
    print("IMPROVEMENT WITH P-CONTROL:")
    print(f"{'='*70}")
    print(f"  Max Drift Reduction:  {max_improvement:>6.1f}%")
    print(f"  Avg Drift Reduction:  {mean_improvement:>6.1f}%")
    print(f"{'='*70}\n")
    print("✅ Analysis Complete!\n")


if __name__ == '__main__':
    main()