#Neema Ahmadian - Collab - July 2022

import numpy as np

#This class handles all coordinates in the telloVision project.
class Coordinates:
    
    def __init__(self):
        self.transforms = {}
    
    # Argument Format: np.array([[x], [y], [z]])
    # g0_r: global origin; x,y,z given in the relative coordinate frame
    # gxa_r: any second point along the global x-axis; x, y, z given in the relative coordinate frame
    def init_global_tracker(self, g0_r, gxa_r):
        x_delta_r = gxa_r[0] - g0_r[0]
        y_delta_r = gxa_r[1] - g0_r[1]
        slope = y_delta_r / x_delta_r
        phi = np.arctan(slope)

        if y_delta_r > 0 and x_delta_r > 0:
            theta = -phi
            # print("Quadrant 1 - Theta(neg) = " + str(theta))
        elif y_delta_r > 0 and x_delta_r < 0:
            theta = -np.pi - phi
            # print("Quadrant 2 - Theta(neg) = " + str(theta))
        elif y_delta_r < 0 and x_delta_r < 0:
            theta = np.pi - phi
            # print("Quadrant 3 - Theta(pos) = " + str(theta))
        elif y_delta_r < 0 and x_delta_r > 0:
            theta = -phi
            # print("Quadrant 4 - Theta(pos) = " + str(theta))

        cos_theta = np.cos(theta)[0]
        sin_theta = np.sin(theta)[0]
        rotation_matrix = np.array([[cos_theta, -sin_theta, 0.],
                                    [-sin_theta, -cos_theta, 0.],
                                    [0., 0., 1.]])
        translation_matrix = -np.matmul(rotation_matrix, g0_r)
        wide_matrix = np.hstack([rotation_matrix, translation_matrix])
        self.transforms["tracker"] = np.vstack([wide_matrix, [0.0, 0.0, 0.0, 1.0]])


    # Argument Format: np.array([[x], [y], [z]])
    #     m0_g: mission pad origin; x,y,z given in the global coordinate frame
    def init_global_mission_pad(self, pad_id, m0_g):
        pad_key = "pad_" + str(pad_id)
        left_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [0, 0, 0]])
        right_matrix = np.vstack([m0_g, np.array([1])])
        self.transforms[pad_key] = np.hstack([left_matrix, right_matrix])

    # Input format: np.array([[x], [y], [z]])
    #     p_r: arbitrary point in the tracking area; x,y,z given in the relative coordinate frame
    # Output:
    #     p_g: p_r expressed in the global coordinate frame
    def global_pos_tracker(self, p_r):
        p_r = np.vstack((p_r, np.array([1])))
        p_g = np.matmul(self.transforms["tracker"], p_r)
        return np.delete(p_g, 3, 0)

    # Input format: np.array([[x], [y], [z]])
    #     drone_m: drone position; x,y,z given in the mission pad coordinate frame
    #     pad_id: pad id integer
    def global_drone_pos(self, pad_id, drone_m):
        pad_key = "pad_" + str(pad_id)
        drone_m = np.vstack((drone_m, np.array([1])))
        drone_g = np.matmul(self.transforms[pad_key], drone_m)
        return np.delete(drone_g, 3, 0)

    #This function transforms a global coordinate to the drone's own coordinate frame
    def global_pos_drone_relative(self, p_g, drone_g):
        # delta between p_g and drone_g in the global frame (units: m)
        delta_g = p_g - drone_g
        return delta_g


