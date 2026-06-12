
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#include "rtwtypes.h"
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#include <math.h>
#include "mex.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 2
#define y_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
#define L1 0.145
#define L2 0.2359
#define L3 0.4191
#define small_seg -0.0275

#define WEIGHT_VAR_ANGLE 0.4
#define PI_VAL 3.14159265358979323846
#define DEG_TO_RAD (PI_VAL / 180.0)

static real_T wrap_to_pi(real_T angle)
{
    angle = fmod(angle + PI_VAL, 2.0 * PI_VAL);

    if (angle < 0.0)
    {
        angle += 2.0 * PI_VAL;
    }

    return angle - PI_VAL;
}
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Output function
 *
 */
void angles_decision_Outputs_wrapper(const real_T *final_pos,
			const real_T *current_theta_1,
			const real_T *current_theta_2,
			const real_T *current_theta_3,
			const real_T *delta_theta_deg,
			real_T *best_next_theta_1_pos,
			real_T *best_next_theta_2_pos,
			real_T *best_next_theta_3_pos,
			real_T *best_next_theta_1_neg,
			real_T *best_next_theta_2_neg,
			real_T *best_next_theta_3_neg,
			real_T *end_effector_calculated)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
real_T current_theta[3];

    real_T possible_delta_deg[3] = {-delta_theta_deg[0], 0.0, delta_theta_deg[0]};

    real_T best_cost;

    real_T best_delta_theta_1;
    real_T best_delta_theta_2;
    real_T best_delta_theta_3;

    int i1;
    int i2;
    int i3;

    current_theta[0] = wrap_to_pi(current_theta_1[0]);
    current_theta[1] = wrap_to_pi(current_theta_2[0]);
    current_theta[2] = wrap_to_pi(current_theta_3[0]);

    best_cost = 1.0e300;

    best_delta_theta_1 = 0.0;
    best_delta_theta_2 = 0.0;
    best_delta_theta_3 = 0.0;

    for (i1 = 0; i1 < 3; i1++)
    {
        for (i2 = 0; i2 < 3; i2++)
        {
            for (i3 = 0; i3 < 3; i3++)
            {
                real_T delta_theta_1;
                real_T delta_theta_2;
                real_T delta_theta_3;

                real_T new_theta_1;
                real_T new_theta_2;
                real_T new_theta_3;

                real_T angle_l1;
                real_T angle_l2;
                real_T angle_l3;

                real_T x_after_rot;
                real_T y_after_rot;

                real_T pos_cost;
                real_T angle_cost;
                real_T total_cost;

                delta_theta_1 = possible_delta_deg[i1] * DEG_TO_RAD;
                delta_theta_2 = possible_delta_deg[i2] * DEG_TO_RAD;
                delta_theta_3 = possible_delta_deg[i3] * DEG_TO_RAD;

                new_theta_1 = wrap_to_pi(current_theta[0] + delta_theta_1);
                new_theta_2 = wrap_to_pi(current_theta[1] + delta_theta_2);
                new_theta_3 = wrap_to_pi(current_theta[2] + delta_theta_3);

                angle_l1 = new_theta_1 +PI_VAL/2;
                angle_l2 = new_theta_1 + new_theta_2 +PI_VAL/2;
                angle_l3 = new_theta_1 + new_theta_2 + new_theta_3+PI_VAL/2;

                x_after_rot = L1 * cos(angle_l1)
                            + L2 * cos(angle_l2)
                            + L3 * cos(angle_l3)+small_seg;

                y_after_rot = L1 * sin(angle_l1)
                            + L2 * sin(angle_l2)
                            + L3 * sin(angle_l3);

                pos_cost = (x_after_rot - final_pos[0]) * (x_after_rot - final_pos[0])
                         + (y_after_rot - final_pos[1]) * (y_after_rot - final_pos[1]);

                angle_cost = WEIGHT_VAR_ANGLE *
                             (delta_theta_1 * delta_theta_1
                            + delta_theta_2 * delta_theta_2
                            + delta_theta_3 * delta_theta_3);

                total_cost = pos_cost + angle_cost;

                if (total_cost < best_cost)
                {
                    best_cost = total_cost;

                    best_delta_theta_1 = delta_theta_1;
                    best_delta_theta_2 = delta_theta_2;
                    best_delta_theta_3 = delta_theta_3;
                }
            }
        }
    }

    /*
     * Output commands:
     *
     * best_next_theta_i_pos = 1 if joint i must rotate by +delta_theta_deg
     * best_next_theta_i_neg = 1 if joint i must rotate by -delta_theta_deg
     *
     * If no rotation is selected:
     * pos = 0 and neg = 0
     */

    end_effector_calculated[0] = L1 * cos(current_theta[0]+PI_VAL/2)
                           + L2 * cos(current_theta[0] + current_theta[1] + PI_VAL/2)
                           + L3 * cos(current_theta[0] + current_theta[1] + current_theta[2] + PI_VAL/2) + small_seg;
    end_effector_calculated[1] = 0.0;
    end_effector_calculated[2] = L1 * sin(current_theta[0]+PI_VAL/2)
                           + L2 * sin(current_theta[0] + current_theta[1]+PI_VAL/2)
                           + L3 * sin(current_theta[0] + current_theta[1] + current_theta[2]+PI_VAL/2);


    best_next_theta_1_pos[0] = (best_delta_theta_1 > 0.0) ? 1.0 : 0.0;
    best_next_theta_2_pos[0] = (best_delta_theta_2 > 0.0) ? 1.0 : 0.0;
    best_next_theta_3_pos[0] = (best_delta_theta_3 > 0.0) ? 1.0 : 0.0;

    best_next_theta_1_neg[0] = (best_delta_theta_1 < 0.0) ? 1.0 : 0.0;
    best_next_theta_2_neg[0] = (best_delta_theta_2 < 0.0) ? 1.0 : 0.0;
    best_next_theta_3_neg[0] = (best_delta_theta_3 < 0.0) ? 1.0 : 0.0;
    
   /*
    * mexPrintf("-------------------------\n");
    * mexPrintf("best_next_theta_1_pos = %f\n", best_next_theta_1_pos[0]);
    * mexPrintf("best_next_theta_2_pos = %f\n", best_next_theta_2_pos[0]);
    * mexPrintf("best_next_theta_3_pos = %f\n", best_next_theta_3_pos[0]);
    * mexPrintf("best_next_theta_1_neg = %f\n", best_next_theta_1_neg[0]);
    * mexPrintf("best_next_theta_2_neg = %f\n", best_next_theta_2_neg[0]);
    * mexPrintf("best_next_theta_3_neg = %f\n", best_next_theta_3_neg[0]);
    */
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


