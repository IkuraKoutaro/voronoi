import math
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from scipy._lib.decorator import decorator as _decorator
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib.animation as anm

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

    def convex_hull_plot(self, hull, ax=None):
        if hull.points.shape[1] != 2:
            raise ValueError("Convex hull is not 2-D")
        ax.plot(hull.points[:, 0], hull.points[:, 1], 'o')
        line_segments = [hull.points[simplex] for simplex in hull.simplices]
        ax.add_collection(LineCollection(line_segments,colors='k',linestyle='solid'))
        self._adjust_bound(ax, hull.points)
        return ax.figure

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

        # print('vor.points : ', vor.points)
        # print('center : ', center)
        # print('ptp_bound : ', ptp_bound)

        finite_segments = []
        infinite_segments = []
        for pointidx, simplex in zip(vor.ridge_points, vor.ridge_vertices):

            simplex = np.asarray(simplex)

            if np.all(simplex >= 0):
                i = simplex[simplex >= 0][0]
                vec_1 = vor.points[pointidx[0]] - vor.vertices[i]
                vec_2 = vor.points[pointidx[1]] - vor.vertices[i]
                vec_size_1 = np.linalg.norm(vec_1)
                vec_size_2 = np.linalg.norm(vec_2)
                vec_normal_1 = vec_1 / vec_size_1
                vec_normal_2 = vec_1 / vec_size_2

                finite_segments.append(vor.vertices[simplex])
                # finite_segments.append(vor.vertices[simplex]+0.2)
                # finite_segments.append(vor.vertices[simplex]-0.2)
            else:
                i = simplex[simplex >= 0][0]  # finite end Voronoi vertex
                # vor.points[] is robots location
                # pointidx is focusing two robots number ex) pointidx[0 4]
                # t is 隣り合う2つのロボットの座標の差
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


                vec_1 = vor.points[pointidx[0]] - vor.vertices[i]
                vec_2 = vor.points[pointidx[1]] - vor.vertices[i]
                vec_size_1 = np.linalg.norm(vec_1)
                vec_size_2 = np.linalg.norm(vec_2)
                vec_normal_1 = vec_1 / vec_size_1
                vec_normal_2 = vec_1 / vec_size_2

                # print('==============================')
                # print('vor.points : ', vor.points[i])
                # print('vor.vertices : ', vor.vertices[i])
                # print('vec : ', vec)
                # print('vec_size :', vec_size)
                # print('vec_normal : ', vec_normal)
                # print('==============================')

                # print('pointidx :', pointidx)
                # print('midpoint :', midpoint,)
                # print('n :', n)
                # print('midpoint-center :', midpoint - center)
                # print('np.dot(midpoint - center, n) :', np.dot(midpoint - center, n))
                # print('np.sign(np.dot(midpoint - center, n)) : ', np.sign(np.dot(midpoint - center, n)))
                # print('direction', direction)
                # print(' ')

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
                #print('=================================================================')

        ax.add_collection(LineCollection(finite_segments, colors=line_colors,lw=1,alpha=line_alpha,linestyle='solid'))
        ax.add_collection(LineCollection(infinite_segments,colors=line_colors,lw=1,alpha=line_alpha,linestyle='solid'))

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

        ax.set_aspect('equal')    
        ax.set_xlim(0,10)
        ax.set_ylim(0,10)
        ax.set_xlabel("X",fontsize=20)
        ax.set_ylabel("Y",fontsize=20)

        #描画する図形のリスト
        elems = []
        self.ani = anm.FuncAnimation(fig, self.world_one_step, fargs=(elems,ax), frames=40, interval=1000, repeat=False)
        #fargs : one_step(func)に渡す引数
        #one_stepの引数i(ステップの番号)は自動的に渡るので渡す必要はない
        plt.show()

    def world_one_step(self, i, elems, ax):
        while elems:elems.pop().remove()
        elems.append(ax.text(-4.4,4.5,"t="+str(i),fontsize=10))
        for obj in self.objects:    #appendした物体を次々に描画
            
            obj.robot_draw(ax, self.points_robot, self.count, elems)
            self.count+=1
            if hasattr(obj, "world_one_step"): obj.world_one_step(1.0)

class IdealRobot:
    def __init__(self, pose, agent=None, color="black"):
        self.pose = pose
        self.r = 0.2
        self.agent = agent
        self.color = color
        self.poses = [pose]  #軌跡の描画用
    
    def robot_draw(self, ax, points_robot, i, elems):
        x, y = points_robot[i]
        theta =  math.pi/6
        xn = x + self.r * math.cos(theta)
        yn = y + self.r * math.sin(theta)
        elems += ax.plot([x,xn],[y,yn],color=self.color)  #ロボットの向きを示す線分の描画 #x,yは円の中心 #xからxnに向けて線を引く #yも同様
        c = patches.Circle(xy=(x,y), radius=self.r, fill=False, color=self.color)
        elems.append(ax.add_patch(c))

        self.poses.append(self.pose)
        elems += ax.plot([e[0] for e in self.poses], [e[1] for e in self.poses], linewidth=0.5, color="black")
    
    #引数で与えられてたposeに姿勢の変化量を足して,移動後の姿勢を返す関数
    @classmethod  #オブジェクトを作らなくてもメソッドを実行できるようにするため
    def state_transition(cls, nu, omega, time, pose):
        t0 = pose[2]
        if math.fabs(omega) < 1e-10:   #角速度がほぼゼロの場合とそうでない場合で場合分け
            return pose + np.array([nu*math.cos(t0), nu*math.sin(t0), omega]) * time
        else:
            return pose+np.array([nu/omega*(math.sin(t0+omega*time)-math.sin(t0)), nu/omega*(-math.cos(t0+omega*time)+math.cos(t0)), omega*time])
    
    #one_stepはエージェントが存在する場合にはエージェントからロボットの速度,角速度を受け取り,姿勢を更新する処理を行う
    #time_interval:離散時間1ステップが何秒であるかをしているするもの
    def idealRobot_one_step(self, time_interval):
        if not self.agent:return
        nu, omega = self.agent.decision()
        self.pose = self.state_transition(nu, omega,time_interval, self.pose)

class Agent:
    def __init__(self, nu, omega):
        self.nu = nu
        self.omega = omega
    def decision(self, observation=None):
        return self.nu, self.omega


world = World()
straight = Agent(0.2,0.0)                 #0.2[m/s]で直進
circling = Agent(0.2,10.0/180*math.pi)    #0.2[m/s], 10[deg/s]（円を描く）
#np.array([x,y,theta])であり、poseに参照される
#.Tは転置

robot1 = IdealRobot( np.array([2,3]).T, straight)          #ロボットのインスタンスを生成
robot2 = IdealRobot( np.array([8,9]).T, circling, "red")   #ロボットのインスタンスを生成（赤）
robot3 = IdealRobot( np.array([8,9]).T, "blue")
robot4 = IdealRobot( np.array([8,9]).T, "green")
robot5 = IdealRobot( np.array([8,9]).T, "orange")
world.append(robot1)
world.append(robot2)
world.append(robot3)
world.append(robot4)
world.append(robot5)

world.world_draw()