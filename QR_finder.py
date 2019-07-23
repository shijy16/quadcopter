# -*- coding=utf-8 -*-
import os
import cv2
import numpy as np
import copy

def reshape_image(image):
    '''归一化图片尺寸：短边400，长边不超过800，短边400，长边超过800以长边800为主'''
    width,height=image.shape[1],image.shape[0]
    min_len=width
    scale=width*1.0/400
    new_width=400
    new_height=int(height/scale)
    if new_height>800:
        new_height=800
        scale=height*1.0/800
        new_width=int(width/scale)
    out=cv2.resize(image,(new_width,new_height))
    return out

def detecte(image):
    '''提取所有轮廓'''
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    # print(gray)
    _,gray=cv2.threshold(gray,239,255,0)
    img,contours,hierachy=cv2.findContours(gray,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cv2.imwrite('1.jpg',img)
    # print(contours[0])
    # c = np.asarray(c)
    if len(contours) > 0: 
        c = []
        for i in contours:
            for p in i:
                c.append([p[0][0],p[0][1]])
        c = np.asarray(c)
        # c = max(contours, key = cv2.contourArea)
        min_rect = cv2.minAreaRect(c)
        center = min_rect[0]
        # box = cv2.boxPoints(min_rect)
        # image = cv2.rectangle(image, (box[0][0],box[0][1]),(box[2][0],box[2][1]), (0,0,255), 5)
        # cv2.imshow('1',image)
        # cv2.waitKey(0)
        # cv2.imwrite('1.jpg',image)
        w = min_rect[1][0]
        l = min_rect[1][1]
        #print ((box[0][0] + box[1][0] + box[2][0] + box[3][0]) / 4)
        #print ((box[0][1] + box[1][1] + box[2][1] + box[3][1]) / 4)
        if l < w:
            l,w = w,l
        return center,[l,w]
    else:
        return None,None
        
def compute_1(contours,i,j):
    '''最外面的轮廓和子轮廓的比例'''
    area1 = cv2.contourArea(contours[i])
    area2 = cv2.contourArea(contours[j])
    if area2==0:
        return False
    ratio = area1 * 1.0 / area2
    if abs(ratio - 49.0 / 25):
        return True
    return False
def compute_2(contours,i,j):
    '''子轮廓和子子轮廓的比例'''
    area1 = cv2.contourArea(contours[i])
    area2 = cv2.contourArea(contours[j])
    if area2==0:
        return False
    ratio = area1 * 1.0 / area2
    if abs(ratio - 25.0 / 9):
        return True
    return False
def compute_center(contours,i):
    '''计算轮廓中心点'''
    M=cv2.moments(contours[i])
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return cx,cy
def detect_contours(vec):
    '''判断这个轮廓和它的子轮廓以及子子轮廓的中心的间距是否足够小'''
    distance_1=np.sqrt((vec[0]-vec[2])**2+(vec[1]-vec[3])**2)
    distance_2=np.sqrt((vec[0]-vec[4])**2+(vec[1]-vec[5])**2)
    distance_3=np.sqrt((vec[2]-vec[4])**2+(vec[3]-vec[5])**2)
    if sum((distance_1,distance_2,distance_3))/3<3:
        return True
    return False
def juge_angle(rec):
    '''判断寻找是否有三个点可以围成等腰直角三角形'''
    if len(rec)<3:
        return -1,-1,-1
    for i in range(len(rec)):
        for j in range(i+1,len(rec)):
            for k in range(j+1,len(rec)):
                distance_1 = np.sqrt((rec[i][0] - rec[j][0]) ** 2 + (rec[i][1] - rec[j][1]) ** 2)
                distance_2 = np.sqrt((rec[i][0] - rec[k][0]) ** 2 + (rec[i][1] - rec[k][1]) ** 2)
                distance_3 = np.sqrt((rec[j][0] - rec[k][0]) ** 2 + (rec[j][1] - rec[k][1]) ** 2)
                if abs(distance_1-distance_2)<5:
                    if abs(np.sqrt(np.square(distance_1)+np.square(distance_2))-distance_3)<5:
                        return i,j,k
                elif abs(distance_1-distance_3)<5:
                    if abs(np.sqrt(np.square(distance_1)+np.square(distance_3))-distance_2)<5:
                        return i,j,k
                elif abs(distance_2-distance_3)<5:
                    if abs(np.sqrt(np.square(distance_2)+np.square(distance_3))-distance_1)<5:
                        return i,j,k
    return -1,-1,-1

def walk(img):
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (9, 9),0)
    gradX = cv2.Sobel(blurred, ddepth=cv2.CV_32F, dx=1, dy=0)
    gradY = cv2.Sobel(blurred, ddepth=cv2.CV_32F, dx=0, dy=1)
 
    gradient = cv2.subtract(gradX, gradY)
    gradient = cv2.convertScaleAbs(gradient)
    
    blurred = cv2.GaussianBlur(gradient, (9, 9),0)
    (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
    # 建立一个椭圆核函数
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25))
    # 执行图像形态学, 细节直接查文档，很简单
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)
    # 这里opencv3返回的是三个参数
    (_, cnts, _) = cv2.findContours(closed.copy(), 
        cv2.RETR_LIST, 
        cv2.CHAIN_APPROX_SIMPLE)
    if cnts == [] :
        c = []
    else:
        c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
    # compute the rotated bounding box of the largest contour
    rect = cv2.minAreaRect(c)
    box = np.int0(cv2.boxPoints(rect))
    #print ((box[0][0] + box[1][0] + box[2][0] + box[3][0]) / 4)
    #print ((box[0][1] + box[1][1] + box[2][1] + box[3][1]) / 4)
    draw_img = drawcnts_and_cut(img, box)
    # cv2.imshow('draw_img', draw_img)
    #cv2.waitKey(0)
    return (box[0][0] + box[1][0] + box[2][0] + box[3][0]) / 4, (box[0][1] + box[1][1] + box[2][1] + box[3][1]) / 4

def find(image,contours,hierachy,root=0):
    '''找到符合要求的轮廓'''
    rec=[]
    for i in range(len(hierachy)):
        child = hierachy[i][2]
        child_child=hierachy[child][2]
        if child!=-1 and hierachy[child][2]!=-1:
            if compute_1(contours, i, child) and compute_2(contours,child,child_child):
                cx1,cy1=compute_center(contours,i)
                cx2,cy2=compute_center(contours,child)
                cx3,cy3=compute_center(contours,child_child)
                if detect_contours([cx1,cy1,cx2,cy2,cx3,cy3]):
                    rec.append([cx1,cy1,cx2,cy2,cx3,cy3,i,child,child_child])
    '''计算得到所有在比例上符合要求的轮廓中心点'''
    i,j,k=juge_angle(rec)
    # print(i,j,k)
    if i==-1 or j== -1 or k==-1:
        return -2,None
    
    ts = np.concatenate((contours[rec[i][6]], contours[rec[j][6]], contours[rec[k][6]]))
    rect = cv2.minAreaRect(ts)
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    result=copy.deepcopy(image)
    # print((box[0][0] + box[1][0]) / 2)
    # print((box[1][1] + box[2][1]) / 2)
    # cv2.drawContours(result, [box], 0, (0, 0, 255), 2)
    # cv2.circle(result,(int((box[0][0] + box[2][0]) / 2), int((box[1][1] + box[0][1]) / 2)),2,(255,0,255),2)

    # # cv2.drawContours(image,contours,rec[i][6],(255,0,0),2)
    # # cv2.drawContours(image,contours,rec[j][6],(255,0,0),2)
    # # cv2.drawContours(image,contours,rec[k][6],(255,0,0),2)
    # # cv2.imshow('img0',image)
    # # cv2.waitKey(0)
    # cv2.imshow('img1',result)
    # cv2.waitKey(0)
    return 0,box
if __name__ == '__main__':
    image = cv2.imread("1.jpg")
    image=reshape_image(image)
    # cv2.imshow('img', image)
    # cv2.waitKey(0)
    center,size=detecte(image)
    print(center,size)
    #cv2.waitKey(0)
    # print(find(image,contours,np.squeeze(hierachy)))
