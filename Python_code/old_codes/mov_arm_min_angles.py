import numpy as np
import itertools
import matplotlib.pyplot as plt

from matplotlib.animation import PillowWriter

#l1 = 30.2
#l2 = 26.9
#l3 = 10.6
l1 = 20.
l2 = 20.
l3 = 20.
weight_var_angle = 0.2
max_iterations = 3000
delta_theta_deg = 0.5

def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


def forward_kinematics(theta):
    x0, y0 = 0, 0

    x1 = l1 * np.cos(theta[0])
    y1 = l1 * np.sin(theta[0])

    x2 = x1 + l2 * np.cos(theta[0] + theta[1])
    y2 = y1 + l2 * np.sin(theta[0] + theta[1])

    x3 = x2 + l3 * np.cos(theta[0] + theta[1] + theta[2])
    y3 = y2 + l3 * np.sin(theta[0] + theta[1] + theta[2])

    return [x0, x1, x2, x3], [y0, y1, y2, y3]


def decision_rotation(pos_final, choices_theta_degree, current_theta):


    tab_cost = np.ones(len(choices_theta_degree)) * np.inf   #so that the cost for the one we do nor set is infinite and can never been chosen
    for i, theta_choice in enumerate(choices_theta_degree):
        if(choices_theta_degree[0] == choices_theta_degree[3] or choices_theta_degree[1] == choices_theta_degree[4] or choices_theta_degree[2] == choices_theta_degree[5]):
            continue #not consider the possibilities that add and then remove an angle
        angle_l1 = current_theta[0] + np.deg2rad(theta_choice[0] - theta_choice[3])
        angle_l2 = current_theta[0] + current_theta[1] + np.deg2rad(theta_choice[0] +theta_choice[1] - theta_choice[3] - theta_choice[4])
        angle_l3 = current_theta[0] + current_theta[1] + current_theta[2] + np.deg2rad(theta_choice[0] +theta_choice[1]+ theta_choice[2] - theta_choice[3] - theta_choice[4] - theta_choice[5])

        x_after_rot = l1 * np.cos(angle_l1) + l2 * np.cos(angle_l2) + l3 * np.cos(angle_l3)
        y_after_rot = l1 * np.sin(angle_l1) + l2 * np.sin(angle_l2) + l3 * np.sin(angle_l3)

        new_theta3 = current_theta[2] + np.deg2rad(theta_choice[2] - theta_choice[5])

        tab_cost[i] = (x_after_rot - pos_final[0] )**2 + (y_after_rot - pos_final[1] )**2 + weight_var_angle * ((np.deg2rad(theta_choice[0] + theta_choice[3]))**2 + (np.deg2rad(theta_choice[1] + theta_choice[4]))**2 + (np.deg2rad(theta_choice[2]+ theta_choice[5]))**2 )

        
        # ce serait cool apres d'ajouter des contraintes ici par la suite pour correspondre a un bras humain

    ind_best_choice = np.argmin(tab_cost)
    best_choice_rot = choices_theta_degree[ind_best_choice]
    return best_choice_rot


def main():
    frames = []
    pos_final = np.array([-30, 50])
    current_theta = np.array([0 * np.pi, 0.85* np.pi, 0. * np.pi])
    d = delta_theta_deg

    choices_theta_degree = [
        list(choice)
        for choice in itertools.product([0., d], repeat=6)
    ]

    iteration_k = 0
    best_choice_rot = np.ones(6)

    # Visualization setup 
    plt.ion()

    fig, ax = plt.subplots()
    reach = l1 + l2 + l3 + 2

    ax.set_xlim(-reach, reach)
    ax.set_ylim(-reach, reach)
    ax.set_aspect("equal")
    ax.grid(True)

    arm_line, = ax.plot([], [], "o-", linewidth=3)
    target_point, = ax.plot(pos_final[0], pos_final[1], "rx", markersize=10)

    pause_time = 10*(10**(-30))  # increase this value to make the animation slower


    writer = PillowWriter(fps=30) # for the GIF
    with writer.saving(fig, "arm_python_angle_only_homo.gif", dpi=100):
        #  Main loop
        while not np.allclose(best_choice_rot, np.zeros(6)) and iteration_k < max_iterations:

            best_choice_rot = decision_rotation(pos_final, choices_theta_degree, current_theta)

            net_choice = np.array(best_choice_rot[:3]) - np.array(best_choice_rot[3:])
            current_theta = current_theta + np.deg2rad(net_choice)
            current_theta = wrap_angle(current_theta)

            x_points, y_points = forward_kinematics(current_theta)

            x_current = x_points[-1]
            y_current = y_points[-1]

            error = (pos_final[0] - x_current)**2 + (pos_final[1] - y_current)**2

            print(f"Iteration {iteration_k}: x = {x_current}, y = {y_current}")
            print(f"Error = {error}")
            print(f"best_choice = {best_choice_rot}")
            print("--------------------------------")

            # Update animation
            arm_line.set_data(x_points, y_points)
            ax.set_title(f"Iteration {iteration_k} | Error = {error:.4f}")

            writer.grab_frame()
            #plt.pause(pause_time)

            iteration_k += 1

    plt.ioff()
    plt.show()


if __name__ == "__main__":
    main()