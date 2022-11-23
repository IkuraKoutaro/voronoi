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
        vor = Voronoi(points)
        voronoi_plot_2d(vor, ax=ax)

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

        print(self.points_robot)
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
robot5 = IdealRobot( np.array([8,9]).T, "yellow")
world.append(robot1)
world.append(robot2)
world.append(robot3)
world.append(robot4)
world.append(robot5)

world.world_draw()