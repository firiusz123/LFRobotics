import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class Kinematics:
    def __init__(self):
        self.Table = []
        self.Matrices = []
        self.positions = []

    def Transform_matrix(self, theta, d, a, alfa):
        T = np.array([[np.cos(theta), -np.sin(theta)*np.cos(alfa), np.sin(alfa)*np.sin(theta), a*np.cos(theta)],
                       [np.sin(theta), np.cos(alfa)*np.cos(theta), -np.sin(alfa)*np.cos(theta), a*np.sin(theta)],
                       [0, np.sin(alfa), np.cos(alfa), d],
                       [0, 0, 0, 1]])
        return T

    def add_values(self, dh_parameters):
        self.Table.append(dh_parameters)

    def show_table(self, angles_in_degrees=True):
        header = [" Theta", "d", "a", "Alfa"]
        print("+---------" * len(header) + "+")
        print("|", end=" ")
        for h in header:
            print(f"{h:8}", end=" | ")
        print()
        print("+---------" * len(header) + "+")
        for row in self.Table:
            print("|", end=" ")
            for i, value in enumerate(row):
                # Convert angles to degrees 
                if angles_in_degrees and i == 0 or i ==3:
                    value = np.degrees(value)
                print(f"{value:8.3f} |", end=" ")
            print()
        print("+---------" * len(header) + "+")

    def get_transformed_values(self):
        k = []
        for i in self.Table:
            T = Kinematics.Transform_matrix(self, i[0], i[1], i[2], i[3])
            k.append(T)
        self.Matrices = k

    def geting_positions(self):
        self.positions = []
        m = np.array(self.Matrices)
        self.positions.append([0, 0, 0])
        self.positions.append(m[0][:3, 3])
        for i in range(len(m)-1):
            result = np.linalg.multi_dot(m[0:i+2])
            print(result[:3,3])
            self.positions.append(result[:3, 3])

    def plotting_points(self):
        positions_array = np.array(self.positions)
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        ax.scatter(positions_array[:, 0], positions_array[:, 1], positions_array[:, 2], c='r', marker='o')
        ax.plot(positions_array[:, 0], positions_array[:, 1], positions_array[:, 2], linestyle='-', color='b')
        for i, pos in enumerate(positions_array):
            ax.text(pos[0], pos[1], pos[2], f'Joint {i+1}', color='black', fontsize=8, ha='right')

        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.set_zlabel('Z-axis')
        n = 450
        ax.set_xlim([-n, n])
        ax.set_ylim([-n, n])
        ax.set_zlim([-n, n])
        ax.quiver(0, 0, 0, 4, 0, 0, color='g', label='X-axis')
        ax.quiver(0, 0, 0, 0, 4, 0, color='b', label='Y-axis')
        ax.quiver(0, 0, 0, 0, 0, 4, color='y', label='Z-axis')
        ax.view_init(elev=30, azim=-75)
        plt.legend()
        
        plt.show()

    def inverse_kinematics_optimization(self, target_position):
        def objective_function(params):
            a, theta1, theta2, theta3 = params
            self.Table[0][1] = a
            self.Table[2][0] = theta1
            self.Table[3][0] = theta2
            self.Table[4][0] = theta3
            self.get_transformed_values()
            end_f_matrix=np.linalg.multi_dot(self.Matrices)
            end_effector_position = end_f_matrix[:3, 3]
            error = np.linalg.norm(end_effector_position - target_position)
            return error
        #bounds = [(0, 10), (-np.pi, np.pi), (-np.pi, np.pi), (-np.pi, np.pi)]
        initial_guess = [self.Table[0][1], self.Table[2][0], self.Table[3][0], self.Table[4][0]]
        bounds = [(None, None), (-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2), (-np.pi/2, np.pi/2)]
        result = minimize(objective_function, initial_guess, method='L-BFGS-B', tol=1e-4 , bounds=bounds)
        print(result)
        if result.success:
            optimized_params = result.x
            self.Table[0][1], self.Table[2][0], self.Table[3][0], self.Table[4][0] = optimized_params
            self.get_transformed_values()
            self.geting_positions()
            self.plotting_points()
            self.show_table()
        else:
            print("Optimization failed.")


# Example usage:
# Target end-effector position
target_position = np.array([324.85281, 102.42641, 277.40774] )


k = Kinematics()

# Define DH parameters
k.add_values([0, 100, 0, 0])
k.add_values([0, 0, 100, np.radians(90)])
k.add_values([np.radians(0), 0, 120, 0])
k.add_values([np.radians(0), 0, 120, 0])
k.add_values([np.radians(0), 0, 140, 0])


k.get_transformed_values()

k.inverse_kinematics_optimization(target_position)
