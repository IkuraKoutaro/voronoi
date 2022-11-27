import math
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from scipy._lib.decorator import decorator as _decorator
from scipy.spatial import Voronoi, voronoi_plot_2d

# __all__ = ['delaunay_plot_2d', 'convex_hull_plot_2d', 'voronoi_plot_2d']

#===================================================#

class Voro:
    def __init__(self):
        self.npoint = 5
        self.maxvalue = 10

    # @_decorator
    # def _held_figure(func, obj, ax=None, **kw):

    #     if ax is None:
    #         fig = plt.figure()
    #         ax = fig.gca()
    #         return func(obj, ax=ax, **kw)

    #     was_held = getattr(ax, 'ishold', lambda: True)()
    #     if was_held:
    #         return func(obj, ax=ax, **kw)
    #     try:
    #         ax.hold(True)
    #         return func(obj, ax=ax, **kw)
    #     finally:
    #         ax.hold(was_held)

    def _adjust_bound(self, ax, points):
        margin = 0.1 * points.ptp(axis=0)
        xy_min = points.min(axis=0) - margin
        xy_max = points.max(axis=0) + margin
        ax.set_xlim(xy_min[0], xy_max[0])
        ax.set_ylim(xy_min[1], xy_max[1])

    # @_held_figure
    def delaunay_plot(self, tri, ax=None):

        if tri.points.shape[1] != 2:
            raise ValueError("Delaunay triangulation is not 2-D")

        x, y = tri.points.T
        ax.plot(x, y, 'o')
        ax.triplot(x, y, tri.simplices.copy())

        self._adjust_bound(ax, tri.points)

        return ax.figure

    # @_held_figure
    def convex_hull_plot(self, hull, ax=None):

        from matplotlib.collections import LineCollection

        if hull.points.shape[1] != 2:
            raise ValueError("Convex hull is not 2-D")

        ax.plot(hull.points[:, 0], hull.points[:, 1], 'o')
        line_segments = [hull.points[simplex] for simplex in hull.simplices]
        ax.add_collection(LineCollection(line_segments,colors='k',linestyle='solid'))
        self._adjust_bound(ax, hull.points)

        return ax.figure


    # @_held_figure
    def voronoi_plot(self, vor, ax=None, **kw):
        from matplotlib.collections import LineCollection

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
        print('ptp_bound : ', ptp_bound)

        finite_segments = []
        infinite_segments = []
        for pointidx, simplex in zip(vor.ridge_points, vor.ridge_vertices):
            simplex = np.asarray(simplex)
            if np.all(simplex >= 0):
                finite_segments.append(vor.vertices[simplex])
            else:
                i = simplex[simplex >= 0][0]  # finite end Voronoi vertex
                # vor.points[] is robots location
                # t は隣り合う2つのロボットの座標の差を意味する
                t = vor.points[pointidx[1]] - vor.points[pointidx[0]]  # tangent

                print('pointidx[1]', pointidx[1])
                print('pointidx[0]', pointidx[0])
                print('vor.points[pointidx[1]] : ', vor.points[pointidx[1]])
                print('vor.points[pointidx[0]] : ', vor.points[pointidx[0]])
                print('t = vor.points[pointidx[1]] - vor.points[pointidx[0]]: ', vor.points[pointidx[1]] - vor.points[pointidx[0]])
                print('np.linalg.norm(t) : ', np.linalg.norm(t))
                print(' ')

                # p.linalg.norm(t) is norm
                # t is normalized t
                t /= np.linalg.norm(t)
                n = np.array([-t[1], t[0]])

                print('t /= np.linalg.norm(t) :', t)
                print('t[0] :', t[0])
                print('t[1] :', t[1])
                print('n : ' , n)
                print(' ')

                # midpoint is points that shows between robots
                midpoint = vor.points[pointidx].mean(axis=0)
                direction = np.sign(np.dot(midpoint - center, n)) * n
                
                print('pointidx :', pointidx)
                print('midpoint :', midpoint,)
                print('n :', n)
                print('np.dot(midpoint - center, n) :', np.dot(midpoint - center, n))
                print('np.sign(np.dot(midpoint - center, n)) : ', np.sign(np.dot(midpoint - center, n)))
                print('direction', direction)

                if (vor.furthest_site):
                    direction = -direction
                
                far_point = vor.vertices[i] + direction * ptp_bound.max()

                print('ptp_bound.max() : ', ptp_bound.max())
                # far_point is edge of line (not vertex)
                print('far_point', far_point)

                infinite_segments.append([vor.vertices[i], far_point])

                print('==========================================')

        ax.add_collection(LineCollection(finite_segments, colors=line_colors,lw=line_width,alpha=line_alpha,linestyle='solid'))
        ax.add_collection(LineCollection(infinite_segments,colors=line_colors,lw=line_width,alpha=line_alpha,linestyle='solid'))

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

robot1 = IdealRobot( np.array([2,3]).T)              #ロボットのインスタンスを生成
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