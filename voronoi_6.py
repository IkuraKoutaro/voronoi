import math
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from scipy._lib.decorator import decorator as _decorator
from scipy.spatial import Voronoi, voronoi_plot_2d

class Voro:
    def __init__(self):
        self.npoint = 5
        self.maxvalue = 10

    def _adjust_bound(self, ax, points):
        margin = 0.1 * points.ptp(axis=0)
        xy_min = points.min(axis=0) - margin
        xy_max = points.max(axis=0) + margin
        ax.set_xlim(xy_min[0], xy_max[0])
        ax.set_ylim(xy_min[1], xy_max[1])

    # def delaunay_plot(self, tri, ax=None):
    #     if tri.points.shape[1] != 2:
    #         raise ValueError("Delaunay triangulation is not 2-D")
    #     x, y = tri.points.T
    #     ax.plot(x, y, 'o')
    #     ax.triplot(x, y, tri.simplices.copy())
    #     self._adjust_bound(ax, tri.points)
    #     return ax.figure

    def convex_hull_plot(self, hull, ax=None):
        if hull.points.shape[1] != 2:
            raise ValueError("Convex hull is not 2-D")
        ax.plot(hull.points[:, 0], hull.points[:, 1], 'o')
        line_segments = [hull.points[simplex] for simplex in hull.simplices]
        ax.add_collection(LineCollection(line_segments,colors='k',linestyle='solid'))
        self._adjust_bound(ax, hull.points)
        return ax.figure

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
        # numpy.ptp : Range of values (maximum - minimum) along an axis.
        ptp_bound = vor.points.ptp(axis=0)

        print('vor.points : ', vor.points)
        print('center : ', center)
        print('ptp_bound : ', ptp_bound)

        finite_segments = []
        infinite_segments = []
        for pointidx, simplex in zip(vor.ridge_points, vor.ridge_vertices):

            simplex = np.asarray(simplex)

            if np.all(simplex >= 0):
                i = simplex[simplex >= 0][0]
                # vec_1 = vor.points[pointidx[0]] - vor.vertices[i]
                # vec_2 = vor.points[pointidx[1]] - vor.vertices[i]
                # vec_size_1 = np.linalg.norm(vec_1)
                # vec_size_2 = np.linalg.norm(vec_2)
                # vec_normal_1 = vec_1 / vec_size_1
                # vec_normal_2 = vec_1 / vec_size_2

                finite_segments.append(vor.vertices[simplex])
                # finite_segments.append(vor.vertices[simplex]+0.2)
                # finite_segments.append(vor.vertices[simplex]-0.2)
            else:
                i = simplex[simplex >= 0][0]  # finite end Voronoi vertex
                # vor.points[] is robots location
                # pointidx is focusing two robots number ex) pointidx[0 4]
                # t is ????????????2?????????????????????????????????
                t = vor.points[pointidx[1]] - vor.points[pointidx[0]]  # tangent

                # print('pointidx[1]', pointidx[1])
                # print('pointidx[0]', pointidx[0])
                # print('vor.points[pointidx[1]] : ', vor.points[pointidx[1]])
                # print('vor.points[pointidx[0]] : ', vor.points[pointidx[0]])
                # print('t = vor.points[pointidx[1]] - vor.points[pointidx[0]]: ', vor.points[pointidx[1]] - vor.points[pointidx[0]])
                # print('np.linalg.norm(t) : ', np.linalg.norm(t))
                # print(' ')

                # p.linalg.norm(t) is norm
                # t is normalized t
                t /= np.linalg.norm(t)
                n = np.array([-t[1], t[0]])

                # print('t /= np.linalg.norm(t) :', t)
                # print('t[0] :', t[0])
                # print('t[1] :', t[1])
                # print('n : ' , n)
                # print(' ')

                # midpoint is points that shows between robots
                midpoint = vor.points[pointidx].mean(axis=0)
                direction = np.sign(np.dot(midpoint - center, n)) * n


                # vec_1 = vor.points[pointidx[0]] - vor.vertices[i]
                # vec_2 = vor.points[pointidx[1]] - vor.vertices[i]
                # vec_size_1 = np.linalg.norm(vec_1)
                # vec_size_2 = np.linalg.norm(vec_2)
                # vec_normal_1 = vec_1 / vec_size_1
                # vec_normal_2 = vec_1 / vec_size_2

                print('==============================')
                print('vor.points : ', vor.points[i])
                # print('vor.vertices : ', vor.vertices[i])
                # print('vec : ', vec)
                # print('vec_size :', vec_size)
                # print('vec_normal : ', vec_normal)
                # print('==============================')

                print('pointidx :', pointidx)
                print('midpoint :', midpoint,)
                print('n :', n)
                print('midpoint-center :', midpoint - center)
                print('np.dot(midpoint - center, n) :', np.dot(midpoint - center, n))
                print('np.sign(np.dot(midpoint - center, n)) : ', np.sign(np.dot(midpoint - center, n)))
                print('direction', direction)
                print(' ')

                if (vor.furthest_site):
                    direction = -direction
                # far_point is edge of line (not vertex)
                far_point = vor.vertices[i] + direction * ptp_bound.max()

                infinite_segments.append([vor.vertices[i], far_point])
                # infinite_segments.append([vor.vertices[i]+vec_normal_1*0.2, far_point+vec_normal_1*0.2])
                # infinite_segments.append([vor.vertices[i]-vec_normal_1*0.2, far_point-vec_normal_2*0.2])



                # print('i :', i)
                # print('vor.vertices[i] :', vor.vertices[i])
                # print('vor.vertices[i]+0.2 :', vor.vertices[i]+0.2)
                # print('ptp_bound.max() : ', ptp_bound.max())
                # print('far_point', far_point)
                print('=================================================================')

        ax.add_collection(LineCollection(finite_segments, colors=line_colors , lw=1,alpha=line_alpha,linestyle='solid'))
        ax.add_collection(LineCollection(infinite_segments,colors=line_colors, lw=1,alpha=line_alpha,linestyle='solid'))

        self._adjust_bound(ax, vor.points)

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
        print('robot location', self.points_robot)
        #append?????????obj???IdealRobot????????????????????????????????????
        #obj.draw???Class IdealRobot???draw???????????????.
        for obj in self.objects:    #append??????????????????????????????
            obj.robot_draw(ax, self.points_robot, self.count)
            self.count+=1

        #ax?????????
        ax.set_aspect('equal')    #??????????????????????????????????????????
        ax.set_xlim(0,10)         #X??????-5m??5m??????????????????
        ax.set_ylim(0,10)         #Y???????????????
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
#np.array([x,y,theta])????????????pose??????????????????
#.T?????????

robot1 = IdealRobot( np.array([2,3]).T)          #??????????????????????????????????????????
robot2 = IdealRobot( np.array([8,9]).T, "red")   #???????????????????????????????????????????????????
robot3 = IdealRobot( np.array([8,9]).T, "blue")
robot4 = IdealRobot( np.array([8,9]).T, "green")
robot5 = IdealRobot( np.array([8,9]).T, "orange")
world.append(robot1)
world.append(robot2)
world.append(robot3)
world.append(robot4)
world.append(robot5)

world.world_draw()