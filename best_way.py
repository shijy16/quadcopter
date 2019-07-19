import operator
import math
import numpy

def get_best_road():
    file = open('position.txt','r')
    mylist = []
    distance_dict = {}      #distance_dict[i][j] = distance
    road_list = []         #point_list[i] = [1,2,3] | [3,5,4]
    dis_dict2 = {}          #dis_dict2[total_distance] = [1,2,3]   key:total_distacne,value:road
    for i in file:
        i = i.replace('\n','')
        i = eval(i)
        mylist.append(i)

    for i in range(len(mylist)):
        distance_dict[i] = {}
        for j in range(len(mylist)):
            if i == j:
                distance_dict[i][j] = 10000;
            else:
                dx = mylist[i][0]-mylist[j][0]
                dy = mylist[i][1]-mylist[j][1]
                distance_dict[i][j] = math.sqrt(dx*dx+dy*dy)
    #force sort#
    #--------------------------------------------------------#
    # for i in range(1,8):
    #     for j in range(1,8):
    #         for k in range(1,8):
    #             if i == j or j == k or i == k:
    #                 continue
    #             else:
    #                 temp = [i,j,k]
    #                 road_list.append(temp)
    # 
    # for i in range(len(road_list)):
    #     temp = road_list[i]
    #     # print(temp[0],temp[1],temp[2])
    #     # last_dis = 
    #     temp_dist = distance_dict[0][temp[0]]+distance_dict[temp[0]][temp[1]]
    #       +distance_dict[temp[1]][temp[2]]+distance_dict[temp[2]][8]
    #     temp.append('end')
    #     temp.insert(0,'start')
    #     dis_dict2[temp_dist] = temp
    #--------------------------------------------------------#
    #dynamic programming
    best = [0,0,0,0,0,0,0]
    best_dict = {}
    best_temp = 1000
    best_point = -1
    for i in range(1,8):
        best_dict[i] = []

    for i in range(1,8):
        for j in range(1,8):

            if distance_dict[i][j] < best_temp:
                best_temp = distance_dict[i][j]
                best_point = j

        best[i-1] += best_temp;
        best_dict[i].append(best_point)
        best_temp = 1000
        j = best_point
        # now chose the last point
        for k in range(1,8):
            if k == i:
                continue
            else:
                if distance_dict[j][k] < best_temp:
                    best_temp = distance_dict[j][k]
                    best_point = k

        best[i-1] += best_temp;
        best_dict[i].append(best_point)
        best_temp = 1000
    for i in range(1,8):
        best[i-1] = best[i-1]+distance_dict[0][i]+distance_dict[best_dict[i][1]][8]

    result_dict = {}
    for i in range(1,8):
        result_dict[best[i-1]] = [i,best_dict[i][0],best_dict[i][1]]

    result = sorted(result_dict)
    pos_list = []
    # print(len(result_dict[result[0]]))
    for i in range( len(result_dict[result[0]]) ):
        # print(i)
        index = result_dict[result[0]][i]
        dx = 0.5
        # print("mylist ",mylist[0])
        temp1 = [mylist[index][0],mylist[index][1]-0.5,mylist[index][2]]
        temp2 = [mylist[index][0],mylist[index][1]+0.5,mylist[index][2]]
        pos_list.append(temp1)
        pos_list.append(temp2)
        #pos_list.append(mylist[index])
    return result[0],pos_list

    


if __name__ == '__main__':
    length,road = get_best_road()