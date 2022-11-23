#!/usr/bin/env python
# coding: utf-8

# In[2]:


import matplotlib
#matplotlib.use('nbagg')
#matplotlib.use('nbagg')の機能を使用することでJupyterNotebook上でアニメーションを作ることができる
import matplotlib.animation as anm
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import numpy as np

import pandas
from scipy.spatial import Voronoi, voronoi_plot_2d
# In[3]:


class World:
    def __init__(self, time_span, time_interval, debug=False):
        self.objects = []
        self.debug = debug
        self.time_span = time_span       #何秒間シミュレートするか
        self.time_interval = time_interval   #Δt
    
    ###引数で与えられたrobotをobjectsに与える###
    def append(self, obj):
        self.objects.append(obj)
    
    def draw(self):
        fig = plt.figure(figsize=(4,4))
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
        
        #描画する図形のリスト
        elems = []
        
        # if self.debug:
        #     for i in range(1000): self.one_step(i, elems, ax)
        # else:
        self.ani = anm.FuncAnimation(fig, self.one_step, fargs=(elems,ax), frames=(int(self.time_span/self.time_interval)+1)*3, interval=int(self.time_interval*1000), repeat=False)
            #fargs : one_step(func)に渡す引数
            #one_stepの引数i(ステップの番号)は自動的に渡るので渡す必要はない
        plt.show()
            
    #アニメーションを1コマ進めるためのメソッド
    def one_step(self, i, elems, ax):
        while elems:elems.pop().remove()
        #2重の描画を防ぐためにelemsのリストにある図形をいったんクリアする処理
        #popはリストの1番後ろからオブジェクトを取り出してリストから消す関数
        #removeはelemsに入っているオブジェクトのメソッドで、オブジェクト自身を消去する関数
        time_str = "t = %.2f[s]" %(self.time_interval*i)   #時刻として表示する文字列
        elems.append(ax.text(-4.4,4.5,time_str,fontsize=10))
        #テキストのオブジェクトを追加するコードであり、t=という文字列にステップの番号iを文字列にしてくっつけている
        #ax.textは作ったオブジェクトの参照を返し。リストelemsに追加される
        for obj in self.objects:    #appendした物体を次々に描画
            obj.draw(ax, elems)
            #hasattr:オブジェクトにメソッドがあるかどうかを調べる関数
            #objにはappendしたrobotが入っている.
            #したがって, 以下のone_stepはIdealRobotのone_step
            if hasattr(obj, "one_step"): obj.one_step(self.time_interval)

class Voronoi:
    def __init__(self, npoint, maxvalue):
        self.npoint = npoint
        self.maxvalue = maxvalue
        
    def add_point(self):
        points =np.array([[np.random.rand()*self.maxvalue,np.random.rand()*self.maxvalue] for i in range(self.npoint)])

        vor = Voronoi(points)
        
        voronoi_plot_2d(vor, ax=ax)


    plt.show()



class IdealRobot:
    def __init__(self, pose, agent=None, sensor=None, color="black"):
        #sensor:IdealCameraクラスが渡される
        self.pose = pose    #引数から姿勢の初期値を設定
        self.r = 0.2        #これは描画のためなので固定値
        self.color = color  #引数から描画するときの色を設定
        self.agent = agent
        self.poses = [pose]  #軌跡の描画用
        self.sensor = sensor
        
    def draw(self, ax, elems):
        x, y, theta = self.pose                  #姿勢の変数を分解して3つの変数へ
        xn = x + self.r * math.cos(theta)        #ロボットの鼻先のx座標
        yn = y + self.r * math.sin(theta)        #ロボットの鼻先のy座標
        elems += ax.plot([x,xn],[y,yn],color=self.color)  #ロボットの向きを示す線分の描画 #x,yは円の中心 #xからxnに向けて線を引く #yも同様
        c = patches.Circle(xy=(x,y), radius=self.r, fill=False, color=self.color)
        elems.append(ax.add_patch(c))      #上のpatches.Circleでロボットの胴体を示す円を作ってサブプロットへ登録
        #drawメソッドでは引数にサブプロットaxをとって、axに対して描画の指令を加えていく
        self.poses.append(self.pose)
        elems += ax.plot([e[0] for e in self.poses], [e[1] for e in self.poses], linewidth=0.5, color="black")
        
        ###IdealCamera.drawを呼び出す###
        if self.sensor and len(self.pose) > 1: 
            #IdealCamera.draw
            self.sensor.draw(ax, elems, self.poses[-2])
            #self.poses[-2]:センサ値を得た時刻の姿勢
        
        ###エージェントの情報を描画###
        if self.agent and hasattr(self.agent, "draw"):
            self.agent.draw(ax, elems)
    
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
        obs = self.sensor.data(self.pose) if self.sensor else None
        nu, omega = self.agent.decision(obs)
        #state_transitionから移動後の姿勢を受け取り,self.poseに渡す
        self.pose = self.state_transition(nu, omega, time_interval, self.pose)


# In[5]:


class Agent:
    def __init__(self, nu, omega):
        self.nu = nu
        self.omega = omega
    
    #observation:センサ値の受け渡しに使われる.
    def decision(self, observation=None):
        return self.nu, self.omega


# In[6]:


class Landmark:
    def __init__(self, x, y):
        self.pos = np.array([x,y]).T
        self.id = None
    
    def draw(self, ax, elems):
        c = ax.scatter(self.pos[0], self.pos[1], s=100, marker="*", label="landmarks", color="orange")
        #scatter：散布図を描くためのメソッド 
        #点を星形(marker="*")
        #点のサイズ(s=100)
        elems.append(c)
        elems.append(ax.text(self.pos[0],self.pos[1],"id:"+str(self.id),fontsize=10))


# In[7]:


class Map:
    def __init__(self):
        self.landmarks = []                         #空のランドマークリストを準備
    
    def append_landmark(self, landmark):            #ランドマークを追加
        #上記の引数landmarkはクラスのLandmarkである
        #Landmarkクラスのインスタンス変数idにアクセス
        landmark.id = len(self.landmarks)           #追加するランドマークにIDを与える 
        self.landmarks.append(landmark)
    
    def draw(self, ax, elems):                      #描画(Landmarkのdrawを順に呼び出し)
        for lm in self.landmarks: lm.draw(ax,elems)


# In[8]:


class IdealCamera:
    #initの引数に地図のオブジェクトを渡す
    def __init__(self, env_map, distance_range=(0.5,6.0), direction_range=(-math.pi/3,math.pi/3)):
        self.map = env_map
        self.lastdata =[]    #最後に計測した結果
        self.distance_range = distance_range
        self.direction_range = direction_range
    
    ###ランドマークが計測できる条件###
    def visible(self, polarpos):
        if polarpos is None:
            return False
        
        return self.direction_range[0] <= polarpos[0] <= self.distance_range[1] and self.direction_range[0] <= polarpos[1] <= self.direction_range[1]
    
    ###すべてのランドマークの観測結果を返すためのメソッド###
    def data(self, cam_pose):
        #observed=センサ値を入れるリスト
        observed = []
        ###observedリストにセンサ値を入れる###
        for lm in self.map.landmarks:    #Mapクラスのインスタンス変数landmarksにアクセス #landmarksにはランドマークが入っている
            #z=observation_functionメソッドで計算したnp.array([np.hypot(*diff), phi]).Tが入る
            z = self.observation_function(cam_pose,lm.pos)
            if self.visible(z): #観測条件を追加（計測範囲ならば返すセンサ値のリストに追加する）
                #append((z,lm.id))はタプル型のオブジェクトを作るときのもので,
                #(a,b)は「変数aと変数bをペアにする」という意味合いになる
                observed.append((z, lm.id))
        
        self.lastdata = observed
        return observed
    
    ###出力方程式の計算###
    @classmethod
    #cam_pose : カメラの姿勢(=ロボットの姿勢)
    #obj_pos  : 物体の位置(=ランドマーク)
    def observation_function(cls, cam_pose, obj_pos):
        #diff = m-x, m-y
        diff = obj_pos - cam_pose[0:2] #[0:2]=リストのスライス（0～1の値を取り出す） #0=xの値,1=yの値
        #phi=カメラからランドマークの向き
        phi = math.atan2(diff[1],diff[0]) - cam_pose[2]
        ###角度の正規化###
        while phi >= np.pi:
            phi -= 2*np.pi
        while phi < -np.pi:
            phi += 2*np.pi
        #hypot(x,y)=√(x^2+y^2)を返す関数
        #hypot(*diff)でdiffの各要素m-x,m-yがx,yとしてhypotにわたる
        return np.array([np.hypot(*diff), phi]).T
    
    ###計測の様子を図示（センサ値を描画する）###
    #センサ値から世界座標系でのランドマークの位置を計算し直し、ロボットとランドマークの間にピンク色の線分を引く
    def draw(self, ax, elems, cam_pose):
        for lm in self.lastdata:
            x, y, theta = cam_pose
            distance, direction = lm[0][0], lm[0][1]
            lx = x + distance * math.cos(direction + theta)
            ly = y + distance * math.sin(direction + theta)
            elems += ax.plot([x,lx],[y,ly],color="pink") #[x,lx]:xからlxまでプロット, [y,ly]:yからlyまでプロット


# In[9]:


if __name__=='__main__':
    world = World(20,1)

    ###地図を生成して3つのランドマークを追加###
    m = Map()
    m.append_landmark(Landmark(2,-2))
    m.append_landmark(Landmark(-1,-3))
    m.append_landmark(Landmark(3,3))
    world.append(m)                           #ワールドに地図を追加

    straight = Agent(0.2,0.0)                 #0.2[m/s]で直進
    circling = Agent(0.2,10.0/180*math.pi)    #0.2[m/s], 10[deg/s]（円を描く）
    #np.array([x,y,theta])であり、poseに参照される #.Tは転置
    robot1 = IdealRobot( np.array([2,3,math.pi/6]).T, sensor=IdealCamera(env_map=m), agent=straight)              #ロボットのインスタンスを生成
    robot2 = IdealRobot( np.array([-2,-1,math.pi/5*6]).T, sensor=IdealCamera(env_map=m), agent=circling, color="red")   #ロボットのインスタンスを生成（赤）
    #robot3 = IdealRobot( np.array([0,0,0]).T, color="blue")   #エージェントを与えないロボット
    world.append(robot1)
    world.append(robot2)

    voronoi = Voronoi(8,10)
    v = voronoi.add_point
    # world.append(v)
    # world.append(robot3)

    ###アニメーションを実行###
    world.draw()


# In[9]:


###IdealCameraクラスが正しくランドマークのセンサ値を返してくるか確認###
cam = IdealCamera(env_map=m)  #m=Map() #IdealCameraのインスタンスcam作成
p = cam.data(robot2.pose)     #IdealCameraのメソッドdata
print(p)
