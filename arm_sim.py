import numpy as np
from matplotlib import animation as anim
from matplotlib import pyplot as plt

#length of Link1 and Link2
l_1 = 10
l_2 = 15

#goal position
x_g = -20
y_g = -4

#gain of PID controller
k_p = 0.5
k_i = 0.0002
k_d = 0.001

#initial position
x_init = 17
y_init = 0

#timestep of simulation
time_step = 10
sim_num = 100

#
x_list = []
y_list = []
theta_2_list = []
theta_1_list = []

#plote_mode is position or angle
plot_mode = "angle"

fig = plt.figure()
if plot_mode == "position":
    ax = fig.add_subplot(111)
    ax.set_aspect("equal", adjustable="box")
    plt.xlim(-25,25)
    plt.ylim(-25,25)
img_list = []


def xy(theta_1, theta_2):
    #find x, y
    x_1 = l_1 * np.cos(theta_1)
    y_1 = l_1 * np.sin(theta_1)
    x_2 = x_1 + l_2 * np.cos(theta_1 + theta_2)
    y_2 = y_1 + l_2 * np.sin(theta_1 + theta_2)
    return x_1, y_1, x_2, y_2

def inv_kinematics(x, y):
    #solving inverse kinematics
    l_3 = np.sqrt((x * x) + (y * y))
    fai_1 = np.arccos(((l_1 * l_1) + (l_3 * l_3) - (l_2 * l_2)) / (2 * l_1 * l_3))
    theta_1 = np.arctan2(y, x) - fai_1
    fai_2 = np.arccos(((l_1 * l_1) + (l_2 * l_2) - (l_3 * l_3)) / (2 * l_1 * l_2))
    theta_2 = np.pi - fai_2
    return theta_1, theta_2

def PID(k_p, k_i, k_d, theta_1, theta_2, theta_1_g, theta_2_g, error_sum_1, 
                   error_sum_2, error_pre_1, error_pre_2):
    
    error_1 = theta_1_g - theta_1
    error_2 = theta_2_g - theta_2
    error_sum_1 += error_1
    error_sum_2 += error_2
    error_diff_1 = error_1 - error_pre_1
    error_diff_2 = error_2 - error_pre_2
    m_1 = k_p * error_1 + k_i * error_sum_1 + k_d * error_diff_1
    m_2 = k_p * error_2 + k_i * error_sum_2 + k_d * error_diff_2
    return m_1, m_2, error_sum_1, error_sum_2, error_1, error_2

def controller(plot_mode):
    theta_1_g, theta_2_g = inv_kinematics(x_g, y_g)
    theta_1, theta_2 = inv_kinematics(x_init, y_init)
    x_1, y_1, x_2, y_2 = xy(theta_1,theta_2)
    error_sum_1 = 0
    error_sum_2 = 0
    error_pre_1 = 0
    error_pre_2 = 0
    for i in range(time_step):
        x_list = []
        y_list = []

        m_1, m_2, error_sum_1, error_sum_2, error_1, error_2 = PID(k_p, k_i, k_d, theta_1, theta_2, theta_1_g, theta_2_g, error_sum_1, 
                   error_sum_2, error_pre_1, error_pre_2)
        
        theta_1 += m_1
        theta_2 += m_2
        error_pre_1 = error_1
        error_pre_2 = error_2

        x_1, y_1, x_2, y_2 = xy(theta_1,theta_2)

        if plot_mode == "angle":
            theta_1_list.append(theta_1*180/np.pi)
            theta_2_list.append(theta_2*180/np.pi)
            plt.xlabel("timestep[s]")
            plt.ylabel("angle[deg]")
            img = plt.plot(theta_1_list, color = 'red')
            img += plt.plot(theta_2_list, color = 'blue')

        if plot_mode == "position":
        
            x_list.append(0)
            x_list.append(x_1)
            x_list.append(x_2)
            y_list.append(0)
            y_list.append(y_1)
            y_list.append(y_2)
            plt.xlabel("X")
            plt.ylabel("Y")
            img = plt.plot(x_list, y_list, color = 'black')
        
        img_list.append(img)

controller(plot_mode=plot_mode) 
ani = anim.ArtistAnimation(fig, img_list, interval = 500)

if plot_mode == "angle":
    ani.save("result\Angle\output_angle.gif", writer="pillow")
if plot_mode == "position":
    ani.save("result\Arm\output_position.gif", writer="pillow")

plt.show()