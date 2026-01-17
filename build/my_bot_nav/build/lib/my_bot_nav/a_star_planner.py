import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import heapq
import math

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('custom_planner')
        
        self.subscription = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.map_data = None
        self.map_info = None
        self.get_logger().info('A* Planner initialized')
    
    def map_callback(self, msg):
        self.map_info = msg.info
        # CRITICAL FIX: Force int8 to handle -1 (unknown) correctly
        grid_1d = np.array(msg.data, dtype=np.int8)
        grid_2d = grid_1d.reshape((msg.info.height, msg.info.width))
        self.map_data = grid_2d  # Keep raw values for debugging
        self.get_logger().info('Map Updated (Int8 Fix Applied)')
    
    def world_to_grid(self, x, y):
        if not self.map_info:
            return None
        gx = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        gy = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        return (gx, gy)
    
    def grid_to_world(self, gx, gy):
        if not self.map_info:
            return None
        wx = (gx * self.map_info.resolution) + self.map_info.origin.position.x
        wy = (gy * self.map_info.resolution) + self.map_info.origin.position.y
        return (wx, wy)
    
    def heuristic(self, a, b):
        """Euclidean distance heuristic"""
        return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    
    def get_neighbors(self, node):
        """Returns 8-connected neighbors that are traversable"""
        x, y = node
        directions = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]
        neighbors = []
        
        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            
            # Check bounds
            if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
                # Check if traversable (only skip if definitely occupied)
                cell_value = self.map_data[ny][nx]
                if cell_value <= 50:  # Free (0-50) or Unknown (-1)
                    neighbors.append((nx, ny))
        
        return neighbors
    
    def a_star_search(self, start, goal):
        """A* pathfinding with enhanced debugging"""
        
        # Validate start and goal cells
        start_val = self.map_data[start[1], start[0]]
        goal_val = self.map_data[goal[1], goal[0]]
        
        self.get_logger().info(f'A* Start: {start} (cell value={start_val})')
        self.get_logger().info(f'A* Goal: {goal} (cell value={goal_val})')
        
        # Check if start/goal are in definite obstacles
        if start_val > 50:
            self.get_logger().error(f'Start is in obstacle! (value={start_val})')
            return []
        if goal_val > 50:
            self.get_logger().error(f'Goal is in obstacle! (value={goal_val})')
            return []
        
        # Initialize A*
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        nodes_explored = 0
        
        while frontier:
            current_priority, current = heapq.heappop(frontier)
            nodes_explored += 1
            
            # Check if we reached the exact goal
            if current == goal:
                self.get_logger().info(f'✓ Path found! Explored {nodes_explored} nodes')
                break
            
            # FALLBACK: Check if we're within 1 cell of goal
            if abs(current[0] - goal[0]) <= 1 and abs(current[1] - goal[1]) <= 1:
                self.get_logger().info(f'✓ Close enough to goal! Explored {nodes_explored} nodes')
                goal = current  # Update goal to current position
                break
            
            # Explore neighbors
            for next_node in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + self.heuristic(goal, next_node)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current
        
        # Check if path was found
        if goal not in came_from:
            self.get_logger().error(f'✗ No path found after exploring {nodes_explored} nodes')
            self.get_logger().error(f'  Distance from final node to goal: {self.heuristic(current, goal):.2f} cells')
            return []
        
        # Reconstruct path
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = came_from[current]
        path.reverse()
        
        return path
    
    def goal_callback(self, msg):
        if self.map_data is None:
            self.get_logger().warn('Waiting for map...')
            return
        
        # Get robot position
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
        except Exception as e:
            self.get_logger().error(f'TF Error: {e}')
            return
        
        # Convert to grid coordinates
        start_grid = self.world_to_grid(rx, ry)
        goal_grid = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        
        if start_grid is None or goal_grid is None:
            self.get_logger().error('Failed to convert world to grid coordinates')
            return
        
        # Debug: Show 3x3 grid around start
        self.get_logger().info('--- CHECKING SURROUNDINGS ---')
        sx, sy = start_grid
        for dy in [-1, 0, 1]:
            row_str = ""
            for dx in [-1, 0, 1]:
                check_x, check_y = sx + dx, sy + dy
                if (0 <= check_x < self.map_info.width) and (0 <= check_y < self.map_info.height):
                    row_str += f"{self.map_data[check_y][check_x]} "
                else:
                    row_str += "X "
            self.get_logger().info(row_str)
        self.get_logger().info('-----------------------------')
        
        # Plan path
        self.get_logger().info(f'Planning {start_grid} -> {goal_grid}...')
        path_indices = self.a_star_search(start_grid, goal_grid)
        
        if not path_indices:
            self.get_logger().warn('No path found!')
            return
        
        # Convert to Path message
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for gx, gy in path_indices:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            world_coords = self.grid_to_world(gx, gy)
            pose.pose.position.x = world_coords[0]
            pose.pose.position.y = world_coords[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'✓ Path Published ({len(path_indices)} waypoints)')

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
