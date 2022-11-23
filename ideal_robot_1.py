import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import numpy as np

class World:
    def __init__(self):
        self.objects = []
    
    def append(self, obj):
        self.objects.append(obj)
    
    def draw(self):
        fig = plt.figure(figsize=(8,8))
        #figsize=(width, height)
        ax = fig.add_subplot(111)
        #plt.figureにグラフを描画するためにsubplotを追加する必要がある
        #subplotの追加は、add_subplotメソッドを使用する
        #(111)の意味は「figの中にサブプロットを1行1列に並べて、今作っているのはその1番目である」
        #つまり、figの中に優位つのサブプロットを創っているという意味になる
        ax.set_aspect('equal')    #縦横比を座標の値と一致させる
        ax.set_xlim(-5,5)         #X軸を-5m×5mの範囲で描画
        ax.set_ylim(-5,5)         #Y軸も同様に
        ax.set_xlabel("X",fontsize=20)
        ax.set_ylabel("Y",fontsize=20)
        
        for obj in self.objects:    #appendした物体を次々に描画
            obj.draw(ax)
        
        plt.show()


world = World()
world.draw()

import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import numpy as np


# In[5]:


class IdealRobot:
    def __init__(self, pose, color="black"):
        self.pose = pose    #引数から姿勢の初期値を設定
        self.r = 0.2        #これは描画のためなので固定値
        self.color = color  #引数から描画するときの色を設定
        
    def draw(self, ax):
        x, y, theta = self.pose                  #姿勢の変数を分解して3つの変数へ
        xn = x + self.r * math.cos(theta)        #ロボットの鼻先のx座標
        yn = y + self.r * math.sin(theta)        #ロボットの鼻先のy座標
        ax.plot([x,xn],[y,yn],color=self.color)  #ロボットの向きを示す線分の描画 #x,yは円の中心 #xからxnに向けて線を引く #yも同様
        c = patches.Circle(xy=(x,y), radius=self.r, fill=False, color=self.color)
        ax.add_patch(c)      #上のpatches.Circleでロボットの胴体を示す円を作ってサブプロットへ登録
        #drawメソッドでは引数にサブプロットaxをとって、axに対して描画の指令を加えていく
# In[6]:
world = World()
#np.array([x,y,theta])であり、poseに参照される
#.Tは転置
robot1 = IdealRobot( np.array([2,3,math.pi/6]).T)              #ロボットのインスタンスを生成
robot2 = IdealRobot( np.array([-2,-1,math.pi/5*6]).T, "red")   #ロボットのインスタンスを生成（赤）
world.append(robot1)
world.append(robot2)
world.draw()