import numpy as np
import itertools

l1 = 30.2
l2 = 26.9
l3 = 10.6
weight_var_angle = 0.4
max_iterations = 3000
delta_theta_deg = 0.5

def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

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

        tab_cost[i] = (x_after_rot - pos_final[0] )**2 + (y_after_rot - pos_final[1] )**2 + weight_var_angle * (((theta_choice[0] + theta_choice[3])*(np.pi/180))**2 + ((theta_choice[1] + theta_choice[4])*(np.pi/180))**2 + ((theta_choice[2]+ theta_choice[5])*(np.pi/180))**2)

        
        # ce serait cool apres d'ajouter des contraintes ici par la suite pour correspondre a un bras humain

    ind_best_choice = np.argmin(tab_cost)
    best_choice_rot = choices_theta_degree[ind_best_choice]
    return best_choice_rot

def main():
    pos_final = np.array([0.4, 5.6])
    current_theta = np.array([0.5 * np.pi, 0. * np.pi, 0. * np.pi])
    d = delta_theta_deg
    choices_theta_degree = [list(choice) for choice in itertools.product([0., d], repeat=6)]
    iteration_k = 0
    best_choice_rot = [1.,1.,1., 1., 1., 1.]
    while best_choice_rot != [0.,0.,0., 0.,0.,0.] and iteration_k < max_iterations:
        best_choice_rot = decision_rotation(pos_final, choices_theta_degree, current_theta)
        current_theta = current_theta + np.deg2rad(best_choice_rot[:3]) - np.deg2rad(best_choice_rot[3:6])
        x_current = l1 * np.cos(current_theta[0]) + l2 * np.cos(current_theta[0] + current_theta[1]) + l3 * np.cos(current_theta[0] + current_theta[1] + current_theta[2])
        y_current = l1 * np.sin(current_theta[0]) + l2 * np.sin(current_theta[0] + current_theta[1]) + l3 * np.sin(current_theta[0] + current_theta[1] + current_theta[2])

        print(f"Iteration {iteration_k}: x = {x_current}, y = {y_current}")
        print(f"Error = {(pos_final[0] - x_current )**2 + (pos_final[1] - y_current )**2}")
        print(f"best_choice = {best_choice_rot}")
        print(f"--------------------------------")
        iteration_k +=1

if __name__ == "__main__":
    main()