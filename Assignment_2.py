import numpy as np #导入数值计算库
import math

def dh_transform(theta,d,a,alpha): #定义单个关节的 DH 变换矩阵
    ct=np.cos(theta)
    st=np.sin(theta)
    ca=np.cos(alpha)
    sa=np.sin(alpha) #计算三角函数值
    T=np.array([[ct,-st*ca,st*sa,a*ct],
                [st,ct*ca,-ct*sa,a*st],
                [0,sa,ca,d],
                [0,0,0,1]])
    return T #按照标准DH公式构造矩阵

def ur5e_fk(joint_angles_deg): #UR5e的正向运动学主函数
    q=[math.radians(angle) for angle in joint_angles_deg] #把角度转换成弧度
    dh_params=[[0.089159,0,np.pi/2] #关节1
               [0,-0.42500,0], #关节2
               [0,-0.39225,0], #关节3
               [0.10915,0,np.pi/2], #关节4
               [0.09465,0,-np.pi/2], #关节5
               [0.08230,0,0] #关节6   
               ]
    #从基座（单位矩阵 I）开始，依次右乘每个关节的变换矩阵
    T_total=np.eye(4)
    for i in range(6):
        d=dh_params[i][0]
        a=dh_params[i][1]
        alpha=dh_params[i][2] #取出第i个关节的d,a,alpha
        theta=q[i] #当前的关节角度q[i]
        T_i=dh_transform(theta, d, a, alpha) #计算当前关节的变换矩阵
        T_total=T_total @ T_i #累积相乘
    return T_total

#把旋转矩阵转成RPY欧拉角
def rot_to_rpy(R):
    sy=np.sqrt(R[0,0]**2+R[1,0]**2) #pitch的正弦值
    if sy > 1e-6:
        roll=np.arctan2(R[2,1],R[2,2])
        pitch=np.arctan2(-R[2,0],sy)
        yaw=np.arctan2(R[1,0],R[0,0])
    else:
        roll=np.arctan2(-R[1,2],R[1,1])
        pitch=np.arctan2(-R[2,0],sy)
        yaw=0.0 #设yaw=0
    return np.array([roll,pitch,yaw]) #判断是否接近万向节死锁

#主程序
if __name__ == "__main__":
    print("UR5e正向运动学计算")
    q_example1=[0,0,0,0,0,0] #所有关节角度为0度（完全伸直向后姿态）
    T1=ur5e_fk(q_example1)
    print("\n【示例1：全部关节为0°】")
    print("末端齐次矩阵T(4x4)：")
    np.set_printoptions(precision=4,suppress=True)  # 设置显示精度
    print(T1)
    pos1=T1[0:3,3] #提取位置
    print(f"末端位置(X,Y,Z)：{pos1[0]:.4f}, {pos1[1]:.4f}, {pos1[2]:.4f} 米")
    R1=T1[0:3, 0:3] #提取姿态
    rpy1=rot_to_rpy(R1)
    print(f"末端姿态RPY(滚转, 俯仰, 偏航)：{np.degrees(rpy1)}度")
    print("\n"+"-"*50)
    
    q_example2 = [0,-90,0,0,0,0] #大臂竖直向上（q2=-90°）
    T2=ur5e_fk(q_example2)
    pos2=T2[0:3,3] #提取位置
    print("【示例2：大臂竖直向上(q2=-90°)】")
    print(f"末端位置(X,Y,Z)：{pos2[0]:.4f}, {pos2[1]:.4f}, {pos2[2]:.4f}米")