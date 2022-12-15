import math
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from scipy._lib.decorator import decorator as _decorator
from scipy.spatial import Voronoi

class Voro:
    def __init__(self):
        self.npoint = 5
        self.maxvalue = 10

    def voronoi_plot(self, vor, ax=None, **kw):

        if vor.points.shape[1] != 2:
            raise ValueError("Voronoi diagram is not 2-D")

        if kw.get('show_points', True):
            point_size = kw.get('point_size', None)
            ax.plot(vor.points[:, 0], vor.points[:, 1], '.', markersize=point_size)
        if kw.get('show_vertices', True):
            ax.plot(vor.vertices[:, 0], vor.vertices[:, 1], 'o')

        line_colors = kw.get('line_colors', 'k')
        line_width = kw.get('line_width', 1.0)
        line_alpha = kw.get('line_alpha', 1.0)

        center = vor.points.mean(axis=0)
        ptp_bound = vor.points.ptp(axis=0)


        # finite_segments   = ( , )
        # infinite_segments = ( , )
        # this means point to point (point = vertices)

        finite_segments = []
        infinite_segments = []
        vector_point_to_point = []
        vector_point_to_farpoint = []

        ####### create dictionally #######
        dic = {}
        if(len(vor.vertices)==4):
            key = ['1', '2', '3', '4']
        else:
            key = ['1', '2', '3']
        for k in key:
            dic[k] = []
        ##################################

        for pointidx, simplex in zip(vor.ridge_points, vor.ridge_vertices):
            simplex = np.asarray(simplex)
            i = simplex[simplex >= 0][0]
            ######### create vector #########
            if np.all(simplex >= 0):
                ##### create point_to_point of vector ######
                vector_point_to_point=(vor.vertices[simplex[1]] - vor.vertices[simplex[0]])
                dic[key[i]].append(vector_point_to_point)

                print('combination(simplex) : ', simplex)
                print('vor.vertices[simplex[0]] : ', vor.vertices[simplex[0]])
                print('vor.vertices[simplex[1]] : ', vor.vertices[simplex[1]])
                print('vertices_point : ', i)
                print('vector_point_to_point : ', vector_point_to_point)
                print('')
                ############################################
            else:
                ##### create far_point of vector ######
                t = vor.points[pointidx[1]] - vor.points[pointidx[0]]
                t /= np.linalg.norm(t)
                n = np.array([-t[1], t[0]])
                midpoint = vor.points[pointidx].mean(axis=0)
                direction = np.sign(np.dot(midpoint - center, n)) * n
                if (vor.furthest_site):
                    direction = -direction
                far_point = vor.vertices[i] + direction * ptp_bound.max()
                vector_point_to_farpoint = (far_point - vor.vertices[i])
                dic[key[i]].append(vector_point_to_farpoint)
                ########################################
                

            ###############################
            if np.all(simplex >= 0):
                i = simplex[simplex >= 0][0]
                # print('vertices',i)
                finite_segments.append(vor.vertices[simplex])
                # vec_size_1 = np.linalg.norm(vec_1)
                # vec_size_2 = np.linalg.norm(vec_2)
            else:
                i = simplex[simplex >= 0][0]
                # vor.points[] is robots location
                # pointidx is focusing two robots number ex) pointidx[0 4]
                t = vor.points[pointidx[1]] - vor.points[pointidx[0]]
                # vec_size_1 = np.linalg.norm(vec_1)
                # vec_size_2 = np.linalg.norm(vec_2)
                t /= np.linalg.norm(t)
                n = np.array([-t[1], t[0]])
                midpoint = vor.points[pointidx].mean(axis=0)
                direction = np.sign(np.dot(midpoint - center, n)) * n
                if (vor.furthest_site):
                    direction = -direction
                far_point = vor.vertices[i] + direction * ptp_bound.max()
                infinite_segments.append([vor.vertices[i], far_point])

        # print(dic)
        # print(' ')

        ###### create combination of vector #######
        import itertools
        pair = {}
        dictionary_holder = {}
        j = 0
        if(len(vor.vertices)==4):
            for i in range(len(dic)):
                dictionary_holder[i] = dic[key[i][0]]
                # print('i : ', i, ' , ', 'b :' , b[i])
        for i in range(len(dictionary_holder)):
            for i in itertools.combinations(dictionary_holder[i], 2):
                pair[j] = i
                # print(pair[j])
                j += 1
        # print(' ')
        # print('pair[0]', pair[0])        
        # print('pair[0]', pair[0][0])        
        # print(' ')
        ###########################################
        # print(len(pair))
        for a in range(len(pair)):
            vector_1 = pair[a][0]
            vector_2 = pair[a][1]
            vector_1_size = np.linalg.norm(vector_1)
            vector_2_size = np.linalg.norm(vector_2)
            angle_bisector = 0.2 * ((vector_2_size*vector_1 + vector_1_size*vector_2) / (vector_1_size+vector_2_size))
            # print('angle_bisector :', angle_bisector)
            ax.plot(angle_bisector[0], angle_bisector[1], 'o', color='r' )

        ax.add_collection(LineCollection(finite_segments, colors='r' , lw=1,alpha=line_alpha,linestyle='solid'))
        ax.add_collection(LineCollection(infinite_segments,colors=line_colors, lw=1,alpha=line_alpha,linestyle='solid'))

        return ax.figure

    def draw_voronoi(self, ax):
        points = np.array([[np.random.rand()*self.maxvalue,np.random.rand()*self.maxvalue] for i in range(self.npoint)])
        vor = Voronoi(points)
        self.voronoi_plot(vor, ax=ax)
        return points

class World:
    def __init__(self):
        self.objects = []
        self.points_robot = []
        self.count = 0
    
    def append(self, obj):
        self.objects.append(obj)
    
    def world_draw(self):
        fig = plt.figure(figsize=(8,8), facecolor='lightgray')
        ax = fig.add_subplot(111)
        
        voronoi = Voro()
        self.points_robot = voronoi.draw_voronoi(ax)

        # points_robot : location of robots
        # print('robot location', self.points_robot)
        #appendの引数objはIdealRobotのインスタンスであるから
        #obj.drawはClass IdealRobotのdraw関数である.
        for obj in self.objects:    #appendした物体を次々に描画
            obj.robot_draw(ax, self.points_robot, self.count)
            self.count+=1

        #axの調整
        ax.set_aspect('equal')    #縦横比を座標の値と一致させる
        ax.set_xlim(0,10)         #X軸を-5m×5mの範囲で描画
        ax.set_ylim(0,10)         #Y軸も同様に
        ax.set_xlabel("X",fontsize=20)
        ax.set_ylabel("Y",fontsize=20)

        plt.show()

class IdealRobot:
    def __init__(self, pose, color="black"):
        self.pose = pose
        self.r = 0.2
        self.color = color
    
    def robot_draw(self, ax, points_robot, i):
        # print(i)
        x, y = points_robot[i]
        theta =  math.pi/6
        xn = x + self.r * math.cos(theta)
        yn = y + self.r * math.sin(theta)

        ax.plot([x,xn], [y,yn], color=self.color)
        c = patches.Circle(xy=(x,y), radius=self.r, fill=False, color=self.color)

        ax.add_patch(c)

world = World()
#np.array([x,y,theta])であり、poseに参照される
#.Tは転置

robot1 = IdealRobot( np.array([2,3]).T)          #ロボットのインスタンスを生成
robot2 = IdealRobot( np.array([8,9]).T, "red")   #ロボットのインスタンスを生成（赤）
robot3 = IdealRobot( np.array([8,9]).T, "blue")
robot4 = IdealRobot( np.array([8,9]).T, "green")
robot5 = IdealRobot( np.array([8,9]).T, "orange")
world.append(robot1)
world.append(robot2)
world.append(robot3)
world.append(robot4)
world.append(robot5)


world.world_draw()