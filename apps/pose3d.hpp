/** @file
 *  @brief 机器人在线运行框架
 *  @author 鞠孝亮 (JU Xiaoliang)
 *  @date 2018.06
 *  @note 本项目使用MIT开源协议，请遵守该协议使用
 */
///TODO:: before apply new members notice the copy construction func
#ifndef POSE_HPP
#define POSE_HPP

//Eigen3
//#include "eigen3/Eigen/Eigen"

//STD
#include <iostream>
#include <iomanip>
#include <cmath>
#include "assert.h"
/**
 * @brief The Pose class 实现三维向量用于坐标表示
 *
 */
template <class T>
class Pose3d
{
public:
    Pose3d();
    /**
     * @brief Pose 构造函数
     * @param a Pose [a, b, c]
     * @param b
     * @param c
     */
    Pose3d(T a, T b, T c, T d, T e, T f);
    /**
     * @brief Pose 拷贝构造函数
     * @param p
     */
    Pose3d(const Pose3d& p);
    T value[6];/**< 三维数据实际存储位置 */
    T &x;
    T &y;
    T &z;
    T &roll;
    T &pitch;
    T &yaw;
    int64_t timestamp;


public:
    inline double normXY(){return sqrt((x * x) + (y * y));}
    /**
     * @brief reset 重置三维向量为[0, 0, 0]
     */
    void reset();
    /**
     * @brief finiteCheck 检查数据是否有限
     * @return true为有限,false为无限无效
     */
    bool finiteCheck();
    /**
     * @brief operator + 加法运算符
     * @param b 被加对象
     * @return 运算结果
     */
    Pose3d<T> operator + (Pose3d<T> const&b)const;
    /**
     * @brief operator - 减法运算符
     * @param b 被减数
     * @return 运算结果
     */
    Pose3d<T> operator - (Pose3d<T> const&b)const;
    /**
     * @brief operator = 赋值运算符
     * @param b
     * @return
     */
    Pose3d<T>& operator = (const Pose3d<T> &b);
    T& operator [] (unsigned int index);

    /**
     * @brief showElement 显示变量信息
     */
    void showElement();
    /**
     * @brief toInt 将连续空间离散化
     * @param resolution 离散分辨率
     * @return 离散化结果
     */
    Pose3d<int> toInt(Pose3d<double> resolution);

};

template <class T>
Pose3d<T>::Pose3d():x(value[0]),y(value[1]),z(value[2]),roll(value[3]),pitch(value[4]),yaw(value[5])
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    roll = pitch = yaw = 0.0;
    timestamp = 0;
}

template <class T>
Pose3d<T>::Pose3d(T a,T b,T c,T d, T e, T f):x(value[0]),y(value[1]),z(value[2]),roll(value[3]),pitch(value[4]),yaw(value[5])
{
    x = a;
    y = b;
    z = c;
    roll = d;
    pitch = e;
    yaw = f;
    timestamp = 0;
}

template <class T>
Pose3d<T>::Pose3d(const Pose3d &p):x(value[0]),y(value[1]),z(value[2]),roll(value[3]),pitch(value[4]),yaw(value[5])
{
    x = p.x;
    y = p.y;
    z = p.z;
    roll = p.roll;
    pitch = p.pitch;
    yaw = p.yaw;
    timestamp = p.timestamp;
}

template <class T>
void Pose3d<T>::reset()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    roll = pitch = yaw = 0.0;
}

//template <class T>
//bool Pose3d<T>::finiteCheck()// Check for NAN or INF
//{
//    if (!finite(x))
//      return false;
//    if (!finite(y))
//      return false;
//    if (!finite(z))
//      return false;
//    if (!finite(roll))
//      return false;
//    if (!finite(pitch))
//      return false;
//    if (!finite(yaw))
//      return false;
//    return true;
//}

template <class T>
Pose3d<T> Pose3d<T>::operator +(const Pose3d<T> &b) const
{
    Pose3d r;
    r.x = this->x + b.x;
    r.y = this->y + b.y;
    r.z = this->z + b.z;
    r.roll = this->roll + b.roll;
    r.pitch = this->pitch + b.pitch;
    r.yaw = this->yaw + b.yaw;
    return r;
}

template <class T>
Pose3d<T> Pose3d<T>::operator -(const Pose3d<T> &b) const
{
    Pose3d r;
    r.x = this->x - b.x;
    r.y = this->y - b.y;
    r.z = this->z - b.z;
    r.roll = this->roll - b.roll;
    r.pitch = this->pitch - b.pitch;
    r.yaw = this->yaw - b.yaw;
    return r;
}

template <class T>
Pose3d<T>& Pose3d<T>::operator =(const Pose3d<T> &b)
{
    for(int i = 0; i != 6; i++){
        value[i] = b.value[i];
    }
    timestamp = b.timestamp;
    return *this;
}

template <class T>
T &Pose3d<T>::operator [](unsigned int index)
{
    assert(index < 6);
    return value[index];
}

template <class T>
void Pose3d<T>::showElement()
{
    /*
     * 仅作为调试使用工具
    */
    std::cout<<"PFvec:"<<std::setprecision(3)<<x<<"\t"<<y<<"\t"<<z<<"\t"<<roll<<"\t"<<pitch<<"\t"<<yaw<<std::endl;
}

template<class T>
Pose3d<int> Pose3d<T>::toInt(Pose3d<double> resolution)
{
    Pose3d<int> result;
    result.x = static_cast<int>(this->x / resolution.x);
    result.y = static_cast<int>(this->y / resolution.y);
    result.z = static_cast<int>(this->z / resolution.z);
    result.roll = static_cast<int>(this->roll / resolution.roll);
    result.pitch = static_cast<int>(this->pitch / resolution.pitch);
    result.yaw = static_cast<int>(this->yaw / resolution.yaw);

//    double limitTheta = 0.0;
//    limitTheta = atan2(sin(this->theta), cos(this->theta));
//    result.theta = static_cast<int>(limitTheta / resolution.theta);
    return result;
}


#endif // POSE_HPP
