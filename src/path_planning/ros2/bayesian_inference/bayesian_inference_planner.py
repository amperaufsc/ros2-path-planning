from scipy.spatial import Delaunay
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

class Vehicle_Pose:
    def __init__(self,x_position, y_position, yaw_position):
        self.x_position_track_reference_frame = x_position
        self.y_position_track_reference_frame = y_position
        self.yaw_position_track_reference_frame = yaw_position
class Bayesian_Inference_Gains:
    def __init__(self, max_angle_change_gain, std_dvt_track_width_gain, 
                       std_dvt_left_right_cones, max_wrong_color_gain, sqd_diff_path_len_sensor_range):
        self.max_angle_change_gain = max_angle_change_gain 
        self.std_dvt_track_width_gain = std_dvt_track_width_gain
        self.std_dvt_left_right_cones = std_dvt_left_right_cones
        self.max_wrong_color_gain = max_wrong_color_gain
        self.sqd_diff_path_len_sensor_range = sqd_diff_path_len_sensor_range
        pass
    

class TreeNode:
    def __init__(self,key, mean_point_coord,parent = None):
        self.key = key
        self.parent = parent
        self.children = []
        self.cost = 0
        self.parentNumber = 0
        if parent:
            self.parentNumber = parent.parentNumber+1
        self.max_angle_change = 0
        self.wrong_color = 0
        self.mean_point_coord = mean_point_coord
        self.path_sqd_lenght = 0
        self.dist_cones = 0
        self.right_cone = []
        self.left_cone = []

class Tree:
    def __init__(self, root: TreeNode):
        self.root = root
        self.leaves = [root]
        self.visited = [root]

    def insertNode(self, node: TreeNode):
        self.visited.append(node)
        if node.parent in self.leaves:
            self.leaves.remove(node.parent)
        self.insertLeaf(node)
        node.parent.children.append(node)
        node.parentNumber = node.parent.parentNumber + 1

    def removeNode(self, node:TreeNode):
        node.parent.children.remove(node)
        if node in self.leaves:
            self.leaves.remove(node)
    
    def insertLeaf(self, node: TreeNode):
        if self.leaves:
            index = len(self.leaves)
            for i,leaf in enumerate(self.leaves):
                if leaf.cost > node.cost:
                    index = i
                    break
            if index == len(self.leaves):
                self.leaves.append(node)
            else:
                self.leaves = self.leaves[:index] + [node] + self.leaves[index:]
        else:
            self.leaves.append(node)


class Bayesian_Inference_Planner:
    def __init__(self, gains: Bayesian_Inference_Gains, beam_width = 3, iterations_number = 10):
        self.beam_width = beam_width
        self.iterations_number = iterations_number
        self.gains = gains

    def update_map(self, global_map):
        self.global_map = global_map

    def update_vehicle_pose(self, vehicle_pose):
        self.vehicle_pose = vehicle_pose

    def triangulate(self, vertices):
        cones = vertices
        cones = np.append(cones,[self.vehicle_pose], axis=0)
        return Delaunay(cones).simplices
    
    def get_mean_point(self, side, vertex):
        return np.array((vertex[side[0]]+vertex[side[1]])/2)

    def get_possible_paths_from_origin(self,tri, origin_index):
        possible_paths = tri[np.where(tri == origin_index)[:1]]
        possible_paths = [np.delete(i, np.where(i == origin_index)) for i in possible_paths]
        return possible_paths
    
    def calculate_max_angle(self, node, Car_Orientation):
        v1 = node.mean_point_coord - node.parent.mean_point_coord
        if node.parentNumber > 1:
            v2 = node.parent.mean_point_coord - node.parent.parent.mean_point_coord
        else:
            v2 = Car_Orientation
        u_v1 = v1/np.linalg.norm(v1)
        u_v2 = v2/np.linalg.norm(v2)
        prod = np.dot(u_v1,u_v2)
        if prod > 1:
            prod = 1
        elif prod < -1:
            prod = -1
        ang = np.arccos(prod)
        node.max_angle_change = ang if ang > node.parent.max_angle_change else node.parent.max_angle_change

    def calculate_path_lenght(self, node:TreeNode):
        pass

    def calculate_max_wrong_color(self, node:TreeNode):
        node.wrong_color = node.parent.wrong_color
        if node.left_cone[3] == 4:
            node.wrong_color += 1
        if node.right_cone[3] == 0:
            node.wrong_color += 1

    def split_right_and_left_cones(self, node:TreeNode, cones):
        Pi = node.parent.mean_point_coord
        Pf = node.mean_point_coord
        v = Pf - Pi
        if node.key[0] == len(cones):
            cone1 = cones[node.key[1]]
        else:
            cone1 = cones[node.key[0]]

        if node.key[1] == len(cones):
            cone2 = cones[node.key[0]]
        else:
            cone2 = cones[node.key[1]]

        if v[1] >= 0 and v[0] < 0 or v[1] < 0 and v[0] < 0:
            if cone1[1] > (v[1]/v[0]*(cone1[0]-Pi[0])+Pi[1]):
                node.left_cone = cone2
                node.right_cone = cone1
            else:
                node.left_cone = cone1
                node.right_cone = cone2
        else:
            if cone1[1] > (v[1]/v[0]*(cone1[0]-Pi[0])+Pi[1]):
                node.left_cone = cone1
                node.right_cone = cone2
            else:
                node.left_cone = cone2
                node.right_cone = cone1
    
    def calculate_var_dist_cones(self, node:TreeNode):
        if node.parentNumber > 1:
            dist_cones_pattern = 2.5
            desv_left = np.linalg.norm(node.left_cone[:2]-node.parent.left_cone[:2]) - dist_cones_pattern
            desv_right = np.linalg.norm(node.right_cone[:2]-node.parent.right_cone[:2]) - dist_cones_pattern
            desv_left = desv_left*desv_left
            desv_right = desv_right*desv_right
            var_desv = desv_right + desv_left
            node.dist_cones = node.parent.dist_cones + var_desv

    def calculate_cost(self, node:TreeNode, Car_Orientation, cones):
        self.calculate_max_angle(node, Car_Orientation)
        self.split_right_and_left_cones(node, cones)
        self.calculate_max_wrong_color(node)
        self.calculate_var_dist_cones(node)
        node.cost = self.gains.max_angle_change_gain*node.max_angle_change + self.gains.max_wrong_color_gain*node.wrong_color
    
    def get_possible_paths_from_node(self,node:TreeNode, tri):
        possible_tri = tri[np.where(tri == node.key[0])[0]]
        possible_tri = possible_tri[np.where(possible_tri == node.key[1])[0]]

        if node.parentNumber == 1:
            indexes = np.where(possible_tri == node.parent.key)[0]
            possible_tri = np.delete(possible_tri,indexes, axis=0)
            a = np.delete(possible_tri,np.where(possible_tri == node.key[0])[1])
            b = np.delete(possible_tri,np.where(possible_tri == node.key[1])[1])
            possible_paths = np.array([a,b])
        else:
            a = 1 if node.parent.key[0] in node.key else 0
            indexes = np.where(possible_tri == node.parent.key[a])[0]
            if not(np.delete(possible_tri,indexes, axis=0).any()):
                return np.array([])
            possible_tri = np.delete(possible_tri,indexes, axis=0)[0]
            a = np.delete(possible_tri,np.where(possible_tri == node.key[0])[0])
            b = np.delete(possible_tri,np.where(possible_tri == node.key[1])[0])
            possible_paths = np.array([a,b])

        return possible_paths
    
    def get_path_from_tree(self,tree,node:TreeNode):
        path = []
        while node != tree.root:
            path.append(node.mean_point_coord)
            node = node.parent
        path.append(tree.root.mean_point_coord)
        path = np.array(path)
        return path[::-1]
    
    def plot_paths(self,tree, cones,v, tri, Car_Position, Car_Orientation):
        blue = np.array([i for i in cones if i[3]==0])
        yellow = np.array([i for i in cones if i[3]==2])

        plt.cla()
        for i,leaf in enumerate(tree.leaves):
            path = self.get_path_from_tree(tree, leaf)
            if i == 0:
                plt.plot(path[:,0],path[:,1],c="r")
            else:
                plt.plot(path[:,0],path[:,1],c="black")
            plt.text(path[:,0][-1],path[:,1][-1],str(round(leaf.wrong_color,2)),horizontalalignment ='center')
        plt.axis('equal')
        plt.scatter(Car_Position[0], Car_Position[1], color="k")
        plt.quiver(Car_Position[0], Car_Position[1], Car_Orientation[0], Car_Orientation[1], color="k")
        plt.scatter(blue[:,0], blue[:,1])
        plt.scatter(yellow[:,0], yellow[:,1])
        #plt.triplot(v[:, 0], v[:, 1], tri, c="black", linewidth = 0.5)
        plt.draw()
        plt.pause(0.05)

    def plan_path(self, obstacle_numpy_array, Vehicle_Pose:Vehicle_Pose):
        cones_array = obstacle_numpy_array

        Car_Position = np.array([Vehicle_Pose.x_position_track_reference_frame, Vehicle_Pose.y_position_track_reference_frame])
        Car_Orientation = np.array([np.cos(Vehicle_Pose.yaw_position_track_reference_frame),np.sin(Vehicle_Pose.yaw_position_track_reference_frame)])

        vertices = np.vstack([cones_array[:,:2],Car_Position])
        origin = len(vertices)-1

        tri  = Delaunay(vertices).simplices

        root = TreeNode(origin,vertices[origin])
        tree = Tree(root)

        for option in self.get_possible_paths_from_origin(tri,origin):
            newnode = TreeNode(option,self.get_mean_point(option,vertices), parent=root)
            self.calculate_cost(newnode, Car_Orientation, cones_array)
            tree.insertNode(newnode)
        
        for i in range(self.iterations_number):
            queue = tree.leaves[:self.beam_width].copy()
            
            for leaf in tree.leaves[self.beam_width:]:
                tree.removeNode(leaf)
            for node in queue:
                paths = self.get_possible_paths_from_node(node, tri)
                if not(paths.any()):
                    break
                for path in paths:
                    newnode = TreeNode(path,self.get_mean_point(path,vertices), parent=node)
                    self.calculate_cost(newnode, Car_Orientation, cones_array)
                    if  newnode.max_angle_change < np.pi/2:
                        tree.insertNode(newnode)
                    # self.plot_paths(tree,cones_array,vertices,tri, Car_Position, Car_Orientation)
        
        # print(tree.leaves[0].max_angle_change)
        return self.get_path_from_tree(tree,tree.leaves[0])

    def get_interpolated_path(self, obstacle_numpy_array, Vehicle_Pose:Vehicle_Pose):
        path = self.plan_path( obstacle_numpy_array, Vehicle_Pose)
        distance = np.cumsum( np.sqrt(np.sum( np.diff(path, axis=0)**2, axis=1 )) )
        distance = np.insert(distance, 0, 0)/distance[-1]
        interpolator =  interp1d(distance, path, kind="slinear", axis=0)

        return interpolator(np.linspace(0,1,100))