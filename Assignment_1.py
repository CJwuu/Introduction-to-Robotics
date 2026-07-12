import numpy as np #定义绕三个坐标轴旋转的基本旋转矩阵
#绕X轴旋转
def Rx(angle):
    c=np.cos(angle) #定义c为cos(angle)
    s=np.sin(angle) #定义s为sin(angle)
    R=np.array([[1,0,0],
                [0,c,-s],
                [0,s,c]]) #X轴旋转矩阵
    return R

#绕Y轴旋转
def Ry(angle):
    c=np.cos(angle)
    s=np.sin(angle)
    R=np.array([[c,0,s],
                [0,1,0],
                [-s,0,c]]) #Y轴旋转矩阵
    return R

# 绕Z轴旋转
def Rz(angle):
    c=np.cos(angle)
    s=np.sin(angle)
    R=np.array([[c,-s,0],
                [s,c,0],
                [0,0,1]]) #Z轴旋转矩阵
    return R

#Extrinsic Euler angles：旋转轴固定不动 R=Rx*Ry*Rz
#XYZ
def extrinsic_XYZ(a,b,c):
    return Rx(a)@Ry(b)@Rz(c)

#XZY
def extrinsic_XZY(a,b,c):
    return Rx(a)@Rz(b)@Ry(c)

#YXZ
def extrinsic_YXZ(a,b,c):
    return Ry(a)@Rx(b)@Rz(c)

#XYX
def extrinsic_XYX(a,b,c):
    return Rx(a)@Ry(b)@Rx(c)

#YZY
def extrinsic_YZY(a,b,c):
    return Ry(a)@Rz(b)@Ry(c)

#ZXZ
def extrinsic_ZXZ(a,b,c):
    return Rz(a)@Rx(b)@Rz(c)

#Intrinsic Euler Angles：旋转轴跟随物体一起变化 将Extrinsic Euler angles旋转顺序反过来
#XYZ
def intrinsic_XYZ(a,b,c):
    return Rz(c)@Ry(b)@Rx(a)

#XZY
def intrinsic_XZY(a,b,c):
    return Ry(c)@Rz(b)@Rx(a)

#YXZ
def intrinsic_YXZ(a,b,c):
    return Rz(c)@Rx(b)@Ry(a)

#XYX
def intrinsic_XYX(a,b,c):
    return Rx(c)@Ry(b)@Rx(a)

#YZY
def intrinsic_YZY(a,b,c):
    return Ry(c)@Rz(b)@Ry(a)

#ZXZ
def intrinsic_ZXZ(a,b,c):
    return Rz(c)@Rx(b)@Rz(a)

#测试
if __name__ == "__main__":
   #设置三个欧拉角
   angle_1=np.deg2rad(45)
   angle_2=np.deg2rad(30)
   angle_3=np.deg2rad(120)
   print("测试:Extrinsic Euler angles XYZ")
   R1=extrinsic_XYZ(angle_1,angle_2,angle_3)
   print(R1)

   print("测试:Intrinsic Euler Angles XYZ")
   R2=intrinsic_XYZ(angle_1,angle_2,angle_3)
   print(R2)