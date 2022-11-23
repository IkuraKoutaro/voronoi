import math
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Voronoi, voronoi_plot_2d


class Voro:
    def __init__(self):
        self.npoint = 5
        self.maxvalue = 10
    
    def draw_voronoi(self, ax):
        points = np.array([[np.random.rand()*self.maxvalue,np.random.rand()*self.maxvalue] for i in range(self.npoint)])
        #print(points)
        vor = Voronoi(points)
        voronoi_plot_2d(vor, ax=ax)

class World:
    def __init__(self):
        self.objects = []
    
    def append(self, obj):
        self.objects.append(obj)
    
    def draw(self):
        fig = plt.figure(figsize=(8,8), facecolor='lightgray')
        ax = fig.add_subplot(111)
        
        voronoi = Voro()
        voronoi.draw_voronoi(ax)
        #appendの引数objはIdealRobotのインスタンスであるから
        #obj.drawはClass IdealRobotのdraw関数である.
        for obj in self.objects:    #appendした物体を次々に描画
            obj.robot_draw(ax)
        
        # voronoi = Voro()
        # voronoi.draw_voronoi(ax)

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
    
    def robot_draw(self, ax):
        x, y, theta = self.pose
        xn = x + self.r * math.cos(theta)
        yn = y + self.r * math.sin(theta)

        ax.plot([x,xn], [y,yn], color=self.color)
        c = patches.Circle(xy=(x,y), radius=self.r, fill=False, color=self.color)

        ax.add_patch(c)

world = World()
#np.array([x,y,theta])であり、poseに参照される
#.Tは転置

robot1 = IdealRobot( np.array([2,3,math.pi/6]).T)              #ロボットのインスタンスを生成
robot2 = IdealRobot( np.array([8,9,math.pi/5*6]).T, "red")   #ロボットのインスタンスを生成（赤）
world.append(robot1)
world.append(robot2)
world.draw()