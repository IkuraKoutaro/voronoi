

import matplotlib
#matplotlib.use('nbagg')
#matplotlib.use('nbagg')の機能を使用することでJupyterNotebook上でアニメーションを作ることができる
import matplotlib.animation as anm
import matplotlib.pyplot as plt
import math
import matplotlib.patches as patches
import numpy as np


# In[2]:


class World:
    def __init__(self, debug=False):
        self.object = []
        self.debug = debug
        
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
        elems = [ ]
        
        if self.debug:
            for i in range(1000):           #デバッグ時はアニメーションさせない
                self.one_step(i, elems, ax)
        else:
            self.ani = anm.FuncAnimation(fig, self.one_step, fargs=(elems,ax), frames=10, interval=1000, repeat=False)
            #fargs : one_step(func)に渡す引数
            #one_stepの引数i(ステップの番号)は自動的に渡るので渡す必要はない
            plt.show()
            
    #アニメーションを1コマ進めるためのメソッド
    def one_step(self, i, elems, ax):
        while elems:elems.pop().remove()
        #2重の描画を防ぐためにelemsのリストにある図形をいったんクリアする処理
        #popはリストの1番後ろからオブジェクトを取り出してリストから消す関数
        #removeはelemsに入っているオブジェクトのメソッドで、オブジェクト自身を消去する関数
        elems.append(ax.text(-4.4,4.5,"t="+str(i),fontsize=10))
        #テキストのオブジェクトを追加するコードであり、t=という文字列にステップの番号iを文字列にしてくっつけている
        #ax.textは作ったオブジェクトの参照を返し。リストelemsに追加される


# In[3]:


world = World()
world.draw()


# In[ ]:



