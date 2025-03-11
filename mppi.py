import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
def normalize_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def calc_reference_path():
    # sine like path, 3000 points
    t = np.linspace(0, 2*np.pi, 3000)
    x = 0.5+2.0 * np.sin(t)
    y = -0.5+2.0 * np.cos(t)
    return np.vstack([x, y]).T
class CircularObstacle:
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r

list_obstacle = []
a = CircularObstacle(1,2.75,0.6)
b = CircularObstacle(1,1,0.3)
c = CircularObstacle(-1,-1,0.3)
list_obstacle.append(a)
list_obstacle.append(b)
list_obstacle.append(c)
        
class Robot:
    def __init__(self, max_w = 1, max_v = 0.5, ref_path: np.ndarray = np.array([[-100.0, 0.0], [100.0, 0.0]]),  dt = 0.05):
        self.max_w = max_w
        self.max_v = max_v
        self.ref_path = ref_path
        self.dt = dt
        self.state = np.zeros(3)
        self.traj = []
    def reset(self, init_state = np.array([0, 0, 0])):
        self.state = init_state.copy()
    def update(self, u: np.ndarray):
        v = np.clip(u[0], -self.max_v, self.max_v)
        w = np.clip(u[1], -self.max_w, self.max_w)
        self.state = self.state.astype(float)
        self.state[0] += v * np.cos(self.state[2]) * self.dt
        self.state[1] += v * np.sin(self.state[2]) * self.dt
        self.state[2] += w * self.dt
        self.state[2] = normalize_angle(self.state[2])
        self.traj.append(self.state.copy())
    def get_closest_point(self)->np.ndarray:
        # the the closest point of the reference path
        dist = np.linalg.norm(self.ref_path - self.state[:2], axis=1)
        closest_idx = np.argmin(dist)
        return self.ref_path[closest_idx]

    
    def get_state(self)->np.ndarray:
        return self.state
    
class MPPI:
    def __init__(self, dt = 0.05,horizon = 30, num_samples = 100, param_exploration = 0.0, sigma: np.ndarray = np.array([[0.5, 0.0], [0.0, 0.1]])):
        self.T = horizon
        self.K = num_samples
        self.dt = dt
        self.param_exploration = param_exploration

        self.sigma = sigma
        self.size_u = 2
        self.u_prev = np.zeros((self.T, self.size_u))

    def calc_input_control(self):
        u = self.u_prev

    def calc_epsilon(self, sigma, size_sample, size_time_step)->np.ndarray:
        if sigma.shape[0] != sigma.shape[1] or sigma.shape[0] != 2:
            print("[ERROR] sigma must be a square matrix with the size of size_dim_u.")
            raise ValueError

        # sample epsilon
        mu = np.zeros((2))
        epsilon = np.random.multivariate_normal(mu, sigma, (size_sample, size_time_step))
        return epsilon
    
controller = MPPI()
print(controller.calc_epsilon(controller.sigma, controller.K, controller.T)[98][20][0],controller.calc_epsilon(controller.sigma, controller.K, controller.T)[98][20][1])

# sim_step = 1000
# dt = 0.05
# vehicle = Robot(dt = dt, ref_path=calc_reference_path())
# vehicle.reset(np.array([0,0,0]))

# fig, ax = plt.subplots(figsize=(9, 9))
# ax.set_aspect('equal')
# ax.set_xlim(-5.0, 5.0)
# ax.set_ylim(-5.0, 5.0)
# ax.set_title("Robot Simulation")
# ax.set_xticks([])
# ax.set_yticks([])
# robot_radius = 0.3
# robot_patch = patches.Circle((vehicle.state[0], vehicle.state[1]), radius=robot_radius, fc='blue', edgecolor='black', linewidth=2.0)
# ax.add_patch(robot_patch)
# orientation_line, = ax.plot([], [], color='red', lw=2)
# collision = False
# for i in range (sim_step):
#     vehicle.update([0.2, 0.1])
#     robot_patch.set_center((vehicle.state[0], vehicle.state[1])) 
    
#     line_x = [vehicle.state[0], vehicle.state[0] + robot_radius * np.cos(vehicle.state[2])]
#     line_y = [vehicle.state[1], vehicle.state[1] + robot_radius * np.sin(vehicle.state[2])]
#     orientation_line.set_data(line_x, line_y) 
#     # visualize trajectory
#     if len(vehicle.traj) > 1:
#         traj = np.array(vehicle.traj)
#         ax.plot(traj[:,0], traj[:,1
#         ],
#         color='gray', alpha=0.5)
#     # visualize reference path
#     ax.plot(vehicle.ref_path[:,0], vehicle.ref_path[:,1], color='green', alpha=0.5)
#     # vi the closest point, dele the previous one 
#     closest = vehicle.get_closest_point()
#     if i > 0:
#         closest_point[0].remove() 
#     closest = vehicle.get_closest_point()
#     closest_point = ax.plot(closest[0], closest[1], 'ro')
#     for i in range(len(list_obstacle)):
#         ax.add_patch(patches.Circle((list_obstacle[i].x, list_obstacle[i].y), list_obstacle[i].r, fc='red', edgecolor='black', linewidth=2.0))
#         if np.linalg.norm(vehicle.state[:2] - np.array([list_obstacle[i].x, list_obstacle[i].y])) < robot_radius + list_obstacle[i].r:
#             collision = True
#             print("Collision!")
#             break

#     if collision:
#         break
    
#     plt.pause(0.05)
#     plt.draw()

#     # print(f"Step {i}: {vehicle.get_state()}")
# plt.show()

