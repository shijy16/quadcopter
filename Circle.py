import math
from decimal import Decimal
import numpy as np
from scipy import interpolate

Multip = 2

#r = 1.5*0.55*delta         turn 1m into position

def get_new_road(road):
    result_list = []
    obs_list = build_obstacle()

    for i in range(len(road)-1):
        x1 = road[i]
        x2 = road[i+1]
        temp_new_way(x1,x2,obs_list,result_list)
    result_list.append(road[len(road)-1]) 
    # for i in result_list:
    #     print('hello '+str(i))
    return result_list

def get_h_line(point_list,p_num):
    x = []
    y = []
    for i in point_list:
        x.append(i[1])
        y.append(i[0])
    x = np.array(x)
    y = np.array(y)
    xx = np.linspace(x.min(), x.max(), p_num)
    zz = []
    for i in range(p_num):
        a = int(i*len(point_list)/p_num)
        zz.append(point_list[a][2])
    li = interpolate.interp1d(x,y,kind = "quadratic")
    yy = li(xx)
    result = []
    for i in range(len(xx)):
        a = [yy[i],xx[i],zz[i]]
        result.append(a)
    return result

def temp_new_way(x1,x2,obs_list,result_list):
    result_list.append(x1)
    print('check road ' + str(x1) + '===>' +str(x2))

    for ob in obs_list:
        cross,p0,p1 = ob.cross(x1,x2)
        if not cross:
            x3 = [p0,p1,x2[2]]
            temp_new_way(x3,x2,obs_list,result_list)


def build_obstacle():
    ob_list = []
    new_list = []
    file = open('obstacle.txt','r')
    for i in file:
        i = i.replace('\n','')
        i = eval(i)
        ob_list.append(i)
    for i in ob_list:
        obj = Circle([i[0],i[1]])
        print(i[0],i[1])
        new_list.append(obj)
    return new_list


class  Circle():
    def __init__( self,origin):
        self.r = 1
        self.origin = origin

    def get_qie_point(self,x1,x2):
        # yi xie xiao de xi jie wen ti, returnCode.etc
        _,a,b = self.get_best_point(x1,x2)
        _,c,d = self.get_best_point(x2,x1)

        x = [a,b]
        y = [c,d]
        # print(x,y)
        A = x1[1]-x[1]
        B = x[0]-x1[0]
        C = x1[0]*x[1]-x1[1]*x[0]

        a = x2[1]-y[1]
        b = y[0]-x2[0]
        c = x2[0]*y[1]-x2[1]*y[0]

        # print(A,B,C)
        # print(a,b,c)
        temp1 = (c/b-C/B)/(A/B-a/b)
        temp2 = (c/a-C/A)/(B/A-b/a)
        temp1 = round(temp1,2)
        temp2 = round(temp2,2)

        print('point:',temp1,temp2)

        return temp1,temp2

    def get_best_point(self,x1,x2):
        x0 = self.origin[0]
        y0 = self.origin[1]
        x = x1[0]
        y = x1[1]
        d1 = math.sqrt((x-x0)*(x-x0)+(y-y0)*(y-y0))      #distance x1 to origin 
        d2 = math.sqrt(d1*d1 - self.r*self.r)
        A = x1[1]-x2[1]
        B = x2[0]-x1[0]
        C = x1[0]*x2[1]-x1[1]*x2[0]

        vx = (x0-x)/d1
        vy = (y0-y)/d1

        f = math.asin( self.r / d1);

        temp1 = round(vx * math.cos(f) - vy * math.sin(f),2)
        temp2 = round(vx * math.sin(f) + vy * math.cos(f),2)
        temp3 = round(vx * math.cos(-f) - vy * math.sin(-f),2)
        temp4 = round(vx * math.sin(-f) + vy * math.cos(-f),2)
        temp1 = round(x + temp1 * d2 * Multip,2)
        temp2 = round(y + temp2 * d2 * Multip,2)
        temp3 = round(x + temp3 * d2 * Multip,2)
        temp4 = round(y + temp4 * d2 * Multip,2)
        s1 = abs(A*temp1+B*temp2+C)
        s2 = abs(A*temp3+B*temp4+C)
        d3 = math.sqrt( (x1[0]-x2[0])*(x1[0]-x2[0])+(x1[1]-x2[1])*(x1[1]-x2[1]) )
        if d3 < d2:
            return True,0,0
        else:
            if s1 > s2:
                return False,temp3,temp4
            else:
                return False,temp1,temp2

    def cross(self,x1,x2):      #True means can corss
        x1[0] = round(x1[0],2)
        x1[1] = round(x1[1],2)
        x2[0] = round(x2[0],2)
        x2[1] = round(x2[1],2)
        A = x1[1]-x2[1]
        B = x2[0]-x1[0]
        C = x1[0]*x2[1]-x1[1]*x2[0]
        temp = A*self.origin[0]+B*self.origin[1]+C
        dist = abs(temp)
        dist = dist/(math.sqrt(A*A+B*B))
        if self.r < dist:

            return True,0,0
        else:   #add point to cross
            result,_,_ = self.get_best_point(x1,x2)
            p0,p1 = self.get_qie_point(x1,x2)
            return result,p0,p1


if __name__ == '__main__':
    # circle = Circle([-1.7,-3.5])
    # a,x,y = circle.cross([-0.09999998658895493, -6.250014305114746],[-3.0, -0.9000134468078613+0.5])
    # print(x,y)
    # mylist = build_obstacle()
    # print(mylist[0].origin)
    circle = Circle([1,1])
    a,b = circle.get_qie_point([-1,4/3],[3,4/3])