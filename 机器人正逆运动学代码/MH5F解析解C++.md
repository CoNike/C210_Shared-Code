### 此代码是安川机器人MH5F逆解解析解代码

> 经过测试，此代码能够满足安川机器人的控制需求。

```c
/*
*   安川机器人逆解解析解代码 ，输入：目标其次变换矩阵，当前状态，目标设备
*                          输出：六关节角度（一组解）
*                          其中，d6机器人长度是                      0.08,
*                               OPT六维力传感器一半厚度是             0.037,
*                               从传感器一半厚度到末端执行器末端长度为  0.1665
*                          后两个参数随着机器人硬件长度的不同而进行修改。
*/
rw::math::Q MH5F_inverse(const Transform3D<double> &target,
                                 rw::kinematics::State& currentstate,
                                 rw::models::Device::Ptr device)
{
   double d1 = 0.33;
   double a1 = 0.088;
   double a2 = 0.31;
   double a3 = 0.04;
   double d4 = -0.305;
   double d6 = -0.08 - 0.037 - 0.1665;

    rw::math::Q _upper = device->getBounds().second;
    rw::math::Q _lower = device->getBounds().first;

   double p1;
   Eigen::MatrixXd Result;
   Eigen::Matrix<double, 16, 6> Q_result;

   Vector3D<double> n = target.R().getCol(0);
   Vector3D<double> o = target.R().getCol(1);
   Vector3D<double> a = target.R().getCol(2);
   Vector3D<double> p = target.P();

    if(d6 < 0) {d6 = -d6;}
    p = p - a*d6;

    double q1_1 = std::atan2(p(1),p(0));
    double q1_2 = std::atan2(-p(1),-p(0));
    double q[16][6] = {0};
    for(int temp = 0; temp < 8;temp++)
    {
        q[temp][0] = q1_1;
        q[temp+8][0] = q1_2;
    }
   for(int temp = 0; temp < 2 ;temp++) {
       double m1 = p(0) - a1 * cos(q[7 * temp + 1][0]);
       double n1 = p(1) - a1 * sin(q[7 * temp + 1][0]);
       p1 = p(2) - d1;
       double m3 = 2 * a2 * a3;
       double n3 = 2 * a2 * d4;
       double p3 = m1 * m1 + n1 * n1 + p1 * p1 - a3 * a3 - d4 * d4 - a2 * a2;
       if (m3 * m3 + n3 * n3 - p3 * p3 < 0) {
           for (int nan_temp = 8 * temp; nan_temp < (8 * temp + 8); nan_temp++) {
               q[nan_temp][2] = NAN;
           }
       } else {
               double q3_1 = atan2(m3, n3) - atan2(p3, sqrt(m3 * m3 + n3 * n3 - p3 * p3));

               if(q3_1 > 3.1415926)
               {
                   q3_1 = -(2 * 3.1415926 - q3_1);
                   for (int nan_temp = 8 * temp; nan_temp < (8 * temp + 5); nan_temp++) {
                       q[nan_temp][2] = q3_1;
                   }
               }else { for (int nan_temp = 8 * temp; nan_temp < (8 * temp + 5); nan_temp++) {
                       q[nan_temp][2] = q3_1; }
               }

               double q3_2 = atan2(m3, n3) - atan2(p3, -sqrt(m3 * m3 + n3 * n3 - p3 * p3));
               if(q3_2 > 3.1415926)
               {
                   q3_2 = -(2 * 3.1415926 - q3_2);
                   for (int nan_temp = (8 * temp + 4); nan_temp < (8 * temp + 8); nan_temp++) {
                       q[nan_temp][2] = q3_2;}
               } else{
                   for (int nan_temp = (8 * temp + 4); nan_temp < (8 * temp + 8); nan_temp++) {
                       q[nan_temp][2] = q3_2;}
               }
       }
   }

   for(int temp = 0;temp < 8;temp++)
   {
       double m2 = a2 + a3 *cos(q[temp * 2 + 1][2]) - d4 *sin(q[temp * 2 +1][2]);
       double n2 = -d4 * cos(q[temp * 2 + 1][2]) - a3 * sin(q[temp * 2 + 1][2]);
       if( ((temp+1) % 2) == 1)
       {
           double q2 = atan2(m2 , n2) - atan2(p1 , sqrt(m2*m2 + n2*n2 - p1*p1));
           q[2 * temp][1] = q2;
           q[2 * temp + 1][1] = q2;
       } else{
           double q2 = atan2(m2 , n2) - atan2(p1 , -sqrt(m2*m2 + n2*n2 -p1*p1));
           q[2 * temp][1] = q2;
           q[2 * temp + 1][1] = q2;
       }
   }
    int m = 16;
    for(int temp = 0; temp < m; temp++)
    {
        double k = a1 + a2 * sin(q[temp][1]) + a2 *sin(q[temp][1] - q[temp][2]) - d4 *cos(q[temp][1] - q[temp][2]);
        q1_1 = atan2(p(1)/k ,p(0)/k);
        if ( abs(q[temp][0] - q1_1) > 1e-6)
        {
            q[temp][0] = NAN;
        }
    }
    for (int temp_cow = 0;temp_cow < 16;temp_cow++)
    {
       for(int temp_col = 0; temp_col < 6; temp_col++)
       {
           Q_result(temp_cow, temp_col) = q[temp_cow][temp_col];
       }
    }

    Result.resize(16,6);
    int j=0;
    int count = 16;
    for (int i=0;i<16;i++)
    {
        if  (isnan( Q_result(i,0)))
        { count--; }
        else
        {
            Result.block(j,0,1,6)=Q_result.block(i,0,1,6);
            j++;
        }
    }
    if (count>0)
    {
        Result.conservativeResize(count,6);
    }

    m = count;
    for(int temp = 0;temp < m;temp++)
    {
        double c5 = a(2)*sin(Result(temp,1) - Result(temp,2)) - a(0)*cos(Result(temp,1) -
                Result(temp,2)) * cos(Result(temp,0)) - a(1)*cos(Result(temp,1) - Result(temp,2))*sin(Result(temp, 0));
        if((temp + 1) % 2 == 1)
        {
            double q5 = acos(-c5);
            Result(temp, 4) = q5;
        } else{
            double q5 = -acos(-c5);
            Result(temp, 4) = q5;
        }
    }
   for(int temp = 0;temp < m ;temp++)
   {
      double c6 = (n(2) * sin(Result(temp, 1) - Result(temp,2)) - n(0)*cos(Result(temp, 1) - Result(temp,2))*cos(Result(temp, 0))
              - n(1) * cos(Result(temp, 1) - Result(temp, 2)) * sin(Result(temp,0))) / sin(Result(temp, 4)) ;
      double s6 = (o(2) * sin(Result(temp, 1) - Result(temp,2)) - o(0)*cos(Result(temp, 1) - Result(temp,2))*cos(Result(temp, 0))
              - o(1) * cos(Result(temp, 1) - Result(temp, 2)) * sin(Result(temp,0))) / sin(Result(temp, 4)) ;
      double q6 = atan2(s6 , c6);
      Result(temp, 5) = q6;

      double s4 = (a(1)* cos(Result(temp,0)) - a(0) *sin(Result(temp,0)))/ sin (Result(temp, 4));
      double c4 = (a(2) * cos(Result(temp,1) - Result(temp,2)) + a(0)*sin(Result(temp,1) - Result(temp,2)) * cos(Result(temp,0))
              + a(1) * sin(Result(temp,1) - Result(temp,2)) * sin(Result(temp,0))) / sin(Result(temp,4));

      double q4 = atan2(s4, c4);
      Result(temp,3) = q4;
   }

    rw::math::Q last_q = device->getQ(currentstate); //changing state;

    Eigen::Matrix<double,1,6>  theta ;
    for(size_t temp = 0;temp<6;temp++)
    {
        theta(0,temp)= last_q(temp);
    }
    int row = Result.rows();
    Eigen::MatrixXd dis;
    dis.resize(1,row);
    for (int i=0;i<row;i++)
    {
        Eigen::Matrix<double,1,6> delta = theta - Result.block(i,0,1,6);
        dis(i)=delta.norm();
    }
    int min_row,min_col;
    double min_position = dis.minCoeff(&min_row,&min_col);

     Q result_q(6,0,0,0,0,0,0);
    for(size_t temp_q = 0;temp_q<6;temp_q++)
    {
        result_q(temp_q) = Result(min_col, temp_q);
    }

    for(size_t temp = 0;temp<6;temp++)
    {
        result_q(temp) = (result_q(temp) < _lower(temp)) ? (_lower(temp)) : result_q(temp);
        result_q(temp) = (result_q(temp) > _upper(temp)) ? (_upper(temp)) : result_q(temp);
    }

    return result_q;
}
```
