import matplotlib.pyplot as plt
import time
import casadi as ca
import math
import numpy as np
import random
# State indices
IX = 0  # x position
IY = 1  # y position
IV = 2  # velocity
IYAW = 3  # yaw angle

# Enhanced MPC parameters
NX = 4  # x = x, y, v, yaw
NU = 2  # a = [accel, steer]
T = 10  # Prediction horizon

# Control constraints
MAX_STEER = np.deg2rad(360.0)
MAX_DSTEER = np.deg2rad(30.0)
MAX_SPEED = 5.0
MIN_SPEED = -5.0
MAX_ACCEL = 5.0

# Cost matrices
R = np.diag([0.1, 0.1])  # input cost matrix
Rd = np.diag([0.1, 0.1])  # input difference cost matrix
Q = np.diag([2.0, 2.0, 0.75, 0.75])  # state cost matrix
Qf = Q  # terminal cost matrix

# Simulation parameters
TARGET_SPEED = 0.0  # [m/s] target speed
MAX_TIME = 50.0  # max simulation time
DT = 0.1 # [s] time tick

# Target point
TARGET_X = 10.0  # [m]
TARGET_Y = 10.0  # [m]

# Obstacle parameters
OBSTACLE_RADIUS = 0.5  # [m]
SAFETY_MARGIN = 0.3  # [m] additional safety margin
MIN_DIST = OBSTACLE_RADIUS + SAFETY_MARGIN

class MovingObstacle:
    def __init__(self, x0, y0, vx, vy):
        self.x = x0
        self.y = y0
        self.vx = vx  # velocity in x direction
        self.vy = vy  # velocity in y direction
    
    def update(self, dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
    
    def predict_position(self, t):
        return self.x + self.vx * t, self.y + self.vy * t

    def check_collision(self, robot_x, robot_y):
        dist = np.hypot(self.x - robot_x, self.y - robot_y)
        return dist <= OBSTACLE_RADIUS

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

def calc_reference_state():
    """
    Calculate reference state for point tracking.
    """
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    
    # Set reference to target point for all prediction steps
    for i in range(T + 1):
        xref[IX, i] = TARGET_X
        xref[IY, i] = TARGET_Y
        xref[IV, i] = 0.0  # Target velocity (stop at target)
        xref[IYAW, i] = 0.0  # Target orientation
    
    return xref, dref


def get_point_robot_model():
    """
    Returns the point robot dynamics model using CasADi.
    """
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    v = ca.SX.sym('v')
    yaw = ca.SX.sym('yaw')
    states = ca.vertcat(x, y, v, yaw)
    
    # Control inputs
    a = ca.SX.sym('a')       # acceleration
    omega = ca.SX.sym('omega')  # angular velocity
    controls = ca.vertcat(a, omega)
    
    # Point robot dynamics
    dx = v * ca.cos(yaw)
    dy = v * ca.sin(yaw)
    dv = a
    dyaw = omega
    
    rhs = ca.vertcat(dx, dy, dv, dyaw)
    f = ca.Function('f', [states, controls], [rhs])
    
    return f

def nonlinear_mpc_control(xref, x0, dref, obstacles):
    """
    MPC controller for point tracking with multiple obstacle avoidance.
    """
    f = get_point_robot_model()
    opt = ca.Opti()
    
    # Decision variables
    X = opt.variable(NX, T + 1)
    U = opt.variable(NU, T)
    
    # Cost function
    cost = 0
    for t in range(T):
        state_error = X[:, t] - xref[:, t]
        control_error = U[:, t]
        cost += ca.mtimes([state_error.T, Q, state_error])
        cost += ca.mtimes([control_error.T, R, control_error])
        
        if t > 0:
            du = U[:, t] - U[:, t-1]
            cost += ca.mtimes([du.T, Rd, du])
        
        # Add obstacle avoidance cost for each obstacle
        for obstacle in obstacles:
            obs_x, obs_y = obstacle.predict_position(t * DT)
            dist = ca.sqrt((X[0, t] - obs_x)**2 + (X[1, t] - obs_y)**2)
            cost += 100* ca.fmax(0, MIN_DIST - dist)**2
    
    # Terminal cost
    terminal_error = X[:, T] - xref[:, T]
    cost += ca.mtimes([terminal_error.T, Qf, terminal_error])
    
    # Dynamics constraints
    for t in range(T):
        x_next = X[:, t] + DT * f(X[:, t], U[:, t])
        opt.subject_to(X[:, t+1] == x_next)
    
    # Initial state constraint
    opt.subject_to(X[:, 0] == x0)
    
    # Path constraints
    opt.subject_to(opt.bounded(-MAX_SPEED, X[IV, :], MAX_SPEED))
    opt.subject_to(opt.bounded(-MAX_ACCEL, U[0, :], MAX_ACCEL))
    opt.subject_to(opt.bounded(-MAX_STEER, U[1, :], MAX_STEER))
    
    # Add hard constraints for all obstacles
    for t in range(T):
        for obstacle in obstacles:
            obs_x, obs_y = obstacle.predict_position(t * DT)
            dist = ca.sqrt((X[0, t] - obs_x)**2 + (X[1, t] - obs_y)**2)
            opt.subject_to(dist >= MIN_DIST * 0.8)  # Slightly relaxed hard constraint
    
    opt.minimize(cost)
    
    opts = {
        'ipopt.print_level': 0,
        'ipopt.max_iter': 100,
        'ipopt.tol': 1e-4,
        'print_time': 0
    }
    opt.solver('ipopt', opts)
    
    try:
        sol = opt.solve()
        u_opt = sol.value(U)
        x_opt = sol.value(X)
        return (u_opt[0, 0], u_opt[1, 0]), \
               x_opt[0, :], x_opt[1, :], x_opt[2, :], x_opt[3, :]
    except:
        return None, None, None, None, None

def main():
    print("Starting Point Robot MPC point tracking demo with moving obstacles")
    state = State(x=-4.0, y=3.0, yaw=0.0, v=0.0)
    vox = 3*random.random() - 0.5
    voy = 3*random.random() - 0.5
    print(vox, voy)
    

    obstacles = [
        MovingObstacle(x0=2.0, y0=5.0, vx = vox, vy = voy),
        MovingObstacle(x0=4.0, y0=2.0, vx = 0, vy = abs(voy)),
        MovingObstacle(x0=6.0, y0=7.0, vx =-vox, vy = -voy)
    ]
    

    x_history = [state.x]
    y_history = [state.y]
    yaw_history = [state.yaw]
    v_history = [state.v]
    t_history = [0.0]
    

    obs_histories = [[obstacle.x] for obstacle in obstacles]
    obs_y_histories = [[obstacle.y] for obstacle in obstacles]
    
    time = 0.0
    target_reached = False
    collision_detected = False
    prev_control = (0.0, 0.0)
    
    while time <= MAX_TIME and not target_reached and not collision_detected:
        xref, dref = calc_reference_state()
        x0 = [state.x, state.y, state.v, state.yaw]
        

        control, ox, oy, ov, oyaw = nonlinear_mpc_control(xref, x0, dref, obstacles)
        
        if control is not None:
            ai, omega = control
            prev_control = control
        else:
            print("MPC optimization failed")
            ai, omega = prev_control
        

        state.x += state.v * math.cos(state.yaw) * DT
        state.y += state.v * math.sin(state.yaw) * DT
        state.yaw += omega * DT
        state.v += ai * DT
        

        for obstacle in obstacles:
            obstacle.update(DT)
            

            if obstacle.check_collision(state.x, state.y):
                print(f"Collision detected at time {time:.2f}!")
                collision_detected = True
                break
        
        if collision_detected:
            break
        

        state.v = np.clip(state.v, MIN_SPEED, MAX_SPEED)
        state.yaw = np.clip(state.yaw, -math.pi, math.pi)
        
        dist_to_target = np.hypot(state.x - TARGET_X, state.y - TARGET_Y)
        if dist_to_target < 0.5 and abs(state.v) < 0.1 and abs(state.yaw < 0.1):
            target_reached = True
            print(f"Target reached at time {time:.2f}")
        
        time += DT
        
        x_history.append(state.x)
        y_history.append(state.y)
        yaw_history.append(state.yaw)
        v_history.append(state.v)
        t_history.append(time)
        
        # Store obstacle histories
        for i, obstacle in enumerate(obstacles):
            obs_histories[i].append(obstacle.x)
            obs_y_histories[i].append(obstacle.y)
        
        # Visualization
        plt.cla()
        plt.plot(TARGET_X, TARGET_Y, "rx", label="target", markersize=10)
        
        # Plot all obstacles
        for i, obstacle in enumerate(obstacles):
            obstacle_circle = plt.Circle((obstacle.x, obstacle.y), OBSTACLE_RADIUS, 
                                      color=f'C{i+1}', fill=True, alpha=0.3)
            plt.gca().add_patch(obstacle_circle)
            safety_circle = plt.Circle((obstacle.x, obstacle.y), MIN_DIST,
                                     color=f'C{i+1}', fill=False, linestyle='--', alpha=0.3)
            plt.gca().add_patch(safety_circle)
            plt.plot(obs_histories[i], obs_y_histories[i], ":", color=f'C{i+1}', 
                    alpha=0.5, label=f"obstacle {i+1} path")
        
        if ox is not None:
            plt.plot(ox, oy, "r-.", label="MPC prediction")
        plt.plot(x_history, y_history, "-b", label="actual")
        plt.plot(state.x, state.y, "ok", label="current position")
        plt.axis("equal")
        plt.grid(True)
        plt.title(f"Time: {time:.2f}, Speed: {state.v:.2f} m/s")
        plt.legend()
        plt.pause(0.001)
    
    # Final plot
    plt.figure()
    plt.plot(TARGET_X, TARGET_Y, "rx", label="target", markersize=10)
    plt.plot(x_history, y_history, '-b', label='actual path')
    
    # Plot final positions of obstacles
    for i, obstacle in enumerate(obstacles):
        plt.plot(obs_histories[i], obs_y_histories[i], ":", color=f'C{i+1}', 
                label=f"obstacle {i+1} path")
        final_obstacle = plt.Circle((obstacle.x, obstacle.y), OBSTACLE_RADIUS, 
                                  color=f'C{i+1}', fill=True, alpha=0.3)
        plt.gca().add_patch(final_obstacle)
    
    plt.grid(True)
    plt.axis("equal")
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()