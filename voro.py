import math
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from scipy._lib.decorator import decorator
from scipy.spatial import Voronoi
import matplotlib.animation as anm

class Voro:
    def __init__(self):
        self.npoint = 3
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
        finite_segments = []
        infinite_segments = []

        for pointidx, simplex in zip(vor.ridge_points, vor.ridge_vertices):
            simplex = np.asarray(simplex)
            if np.all(simplex >= 0):
                i = simplex[simplex >= 0][0]
                finite_segments.append(vor.vertices[simplex])
            else:
                i = simplex[simplex >= 0][0]
                t = vor.points[pointidx[1]] - vor.points[pointidx[0]]
                t /= np.linalg.norm(t)
                n = np.array([-t[1], t[0]])
                # midpoint is points that shows between robots
                midpoint = vor.points[pointidx].mean(axis=0)
                direction = np.sign(np.dot(midpoint - center, n)) * n
                if (vor.furthest_site):
                    direction = -direction
                # far_point is edge of line (not vertex)
                far_point = vor.vertices[i] + direction * ptp_bound.max()
                infinite_segments.append([vor.vertices[i], far_point])
        ax.add_collection(LineCollection(finite_segments, colors=line_colors , lw=1,alpha=line_alpha,linestyle='solid'))
        ax.add_collection(LineCollection(infinite_segments,colors=line_colors, lw=1,alpha=line_alpha,linestyle='solid'))

        return ax.figure

    def draw_voronoi(self, ax):
        points = np.array([[np.random.rand()*self.maxvalue,np.random.rand()*self.maxvalue] for i in range(self.npoint)])
        vor = Voronoi(points)
        self.voronoi_plot(vor, ax=ax)
        return points

class World:
    def __init__(self, time_span, time_interval, debug=False):
        self.objects = []
        self.points_robot = []
        self.debug = debug
        self.time_span = time_span       #何秒間シミュレートするか
        self.time_interval = time_interval   #Δt
        self.fig = plt.figure(figsize=(4,4))
        self.ax = self.fig.add_subplot(111)
    
    def append(self, obj):
        self.objects.append(obj)
    
    def voro_draw(self):
        voronoi = Voro()
        self.points_robot = voronoi.draw_voronoi(self.ax)
        for obj in self.objects:    #appendした物体を次々に描画
            obj.robot_draw(self.ax, self.points_robot, self.count)
            self.count+=1

    def draw(self):
        self.ax.set_aspect('equal')   
        self.ax.set_xlim(-5,5)
        self.ax.set_ylim(-5,5)
        self.ax.set_xlabel("X",fontsize=20)
        self.ax.set_ylabel("Y",fontsize=20)
        elems = []
        if self.debug:
            for i in range(1000): self.one_step(i, elems, self.ax)
        else:
            self.ani = anm.FuncAnimation(self.fig, self.one_step, fargs=(elems,self.ax), frames=int(self.time_span/self.time_interval)+1, interval=int(self.time_interval*1000), repeat=False)
            plt.show()
            
    def one_step(self, i, elems, ax):
        while elems:elems.pop().remove()
        time_str = "t = %.2f[s]" %(self.time_interval*i)
        elems.append(ax.text(-4.4,4.5,time_str,fontsize=10))
        for obj in self.objects:
            obj.draw(ax, elems)
            if hasattr(obj, "one_step"): obj.one_step(self.time_interval)

class IdealRobot:
    def __init__(self, pose, agent=None, color="black"):
        self.pose = pose   
        self.r = 0.2      
        self.color = color 
        self.agent = agent
        self.poses = [pose]  
        
    def draw(self, ax, elems):
        x, y, theta = self.pose               
        xn = x + self.r * math.cos(theta)        #ロボットの鼻先のx座標
        yn = y + self.r * math.sin(theta)        #ロボットの鼻先のy座標
        elems += ax.plot([x,xn],[y,yn],color=self.color)  #ロボットの向きを示す線分の描画 #x,yは円の中心 #xからxnに向けて線を引く #yも同様
        c = patches.Circle(xy=(x,y), radius=self.r, fill=False, color=self.color)
        elems.append(ax.add_patch(c))      #上のpatches.Circleでロボットの胴体を示す円を作ってサブプロットへ登録
        #drawメソッドでは引数にサブプロットaxをとって、axに対して描画の指令を加えていく
        
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
    def one_step(self, time_interval):
        if not self.agent:return
        nu, omega = self.agent.decision()
        self.pose = self.state_transition(nu, omega, time_interval, self.pose)

class Agent:
    def __init__(self, nu, omega):
        self.nu = nu
        self.omega = omega
    
    def decision(self, observation=None):
        return self.nu, self.omega

class Landmark:
    def __init__(self, x, y):
        self.pos = np.array([x,y]).T
        self.id = None
    
    def draw(self, ax, elems):
        c = ax.scatter(self.pos[0], self.pos[1], s=100, marker="*", label="landmarks", color="orange")
        elems.append(c)
        elems.append(ax.text(self.pos[0],self.pos[1],"id:"+str(self.id),fontsize=10))

class Map:
    def __init__(self):
        self.landmarks = []                        
    
    def append_landmark(self, landmark):           
        landmark.id = len(self.landmarks)        
        self.landmarks.append(landmark)
    
    def draw(self, ax, elems): 
        for lm in self.landmarks: lm.draw(ax,elems)

world = World(10,1)

m = Map()
m.append_landmark(Landmark(2,-2))
m.append_landmark(Landmark(-1,-3))
m.append_landmark(Landmark(3,3))
world.append(m) 

straight = Agent(0.2,0.0)  
circling = Agent(0.2,10.0/180*math.pi)  
robot1 = IdealRobot( np.array([2,2,math.pi/6]).T, straight) 
robot2 = IdealRobot( np.array([-2,-2,math.pi/5*6]).T, circling, "red") 
robot3 = IdealRobot( np.array([0,0,0]).T, color="blue") 
world.append(robot1)
world.append(robot2)
world.append(robot3)

world.draw()
