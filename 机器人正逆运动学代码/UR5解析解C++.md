### 经过测试，与正运动学相符
> 由于UR5 存在制作误差，末端x方向会有固定大约3mm的误差，这个UR5解析解的逆解对于末端位置要求非常高的地方需要进行补偿。

```c
/*
*   UR5 解析解函数  输入：目标其次变换矩阵，当前状态，所用设备
*                 输出：六关节角度（一组）已经选择。
*/
rw::math::Q UR5_inverse(const Transform3D<double> &target,
                        rw::kinematics::State& currentstate,
                        rw::models::Device::Ptr device)
{
    double a2 = -0.425;
    double a3 = -0.39225;
    double d1 = 0.089159;
    double d4 = 0.10915;
    double d5 = 0.09465;
    double d6 = 0.0823;

    double q3;

    Vector3D<double> n = target.R().getCol(0);
    Vector3D<double> o = target.R().getCol(1);
    Vector3D<double> a = target.R().getCol(2);
    Vector3D<double> p = target.P();

    Eigen::MatrixXd Result;
    Eigen::Matrix<double, 8, 6> Q_result;
    rw::math::Q _upper = device->getBounds().second;
    rw::math::Q _lower = device->getBounds().first;

    double q[8][6] = {0};

    double m1 = d6 * a(1) - p(1);
    double n1 = d6 * a(0) - p(0);

    double k = m1 * m1 + n1 * n1 -d4 * d4;
    if(k > -1e-8 && k<0)
    {
        k=0;
    }
    for(int index = 0; index <4; index++)
    {
        q[index][0] = atan2(m1,n1) - atan2(d4, sqrt(k));
        q[index + 4][0] = atan2(m1,n1) - atan2(d4, -sqrt(k));
    }
    double q5 = 0;
    for(int index = 0; index < 4; index++)
    {
         q5 = acos(a(0) * sin(q[2 * index + 1][0]) - a(1) * cos(q[2 * index + 1][0]));
        if( (index % 2) == 0) {
            q[2 * index][4] = q5;
            q[2 * index + 1][4] = q5;
        } else
        {
            q[2 * index][4] = -q5;
            q[2 * index + 1][4] = -q5;
        }
    }

    for(int index = 0;index < 8;index++)
    {
        double m6 = n(0) * sin(q[index][0]) - n(1) * cos(q[index][0]);
        double n6 = o(0) * sin(q[index][0]) - o(1) * cos(q[index][0]);

        q[index][5] = atan2(m6 ,n6) - atan2(sin(q[index][4]),0);

        double m3 = d5 *( sin(q[index][5]) * (n(0) * cos(q[index][0]) + n(1) * sin(q[index][0]))

                        + cos(q[index][5]) * (o(0) * cos(q[index][0]) + o(1) * sin(q[index][0])))

                    + p(0) * cos(q[index][0]) + p(1) * sin(q[index][0]) - d6 *(a(0) * cos(q[index][0]) + a(1) * sin(q[index][0]));

        double n3 = p(2) - d1 - a(2) * d6 + d5 * (o(2) * cos(q[index][5]) + n(2) * sin(q[index][5]) );

        double k3 = (m3 * m3 + n3 * n3 - a2 * a2 - a3 * a3) / (2 * a2 * a3);

        if(  (k3 - 1) > 1e-6 || (k3 + 1) < 1e-6 )
        {
            q3 = NAN;
        }else if(  (k3 - 1) > 0 && (k3 - 1) <= 1e-6 )
        {
            q3 = 0;
        } else if((k3 + 1) <= 0 && (k3 + 1) > 1e-6)
        {
            q3 = M_PI;
        } else {
            q3 = acos(k3);
        }

        if((index % 2) == 0)
        {
            q[index][2] = q3;
        } else{
            q[index][2] = - q3;
        }

        double s2 = ( (a3 * cos(q[index][2]) + a2) * n3 - a3 * sin(q[index][2]) * m3) / ( a2 * a2 + a3 * a3 + 2 * a2 * a3 *cos(q[index][2]));
        double c2 = ( m3 + a3 * sin(q[index][2]) * s2) / ( a3 * cos(q[index][2]) + a2);

        q[index][1] = atan2(s2,c2);

        double s234 = -sin(q[index][5]) * (n(0) * cos(q[index][0]) + n(1) * sin(q[index][0])) - cos(q[index][5]) * (o(0) * cos(q[index][0]) + o(1) * sin(q[index][0]));
        double c234 = o(2) * cos(q[index][5]) + n(2) * sin(q[index][5]);

        q[index][3] = atan2(s234 ,c234) - q[index][1] - q[index][2];

    }

    for(int index_i = 0; index_i < 8 ; index_i++)
    {
        for(int index_j = 0;index_j < 6; index_j++)
        {
            if(q[index_i][index_j] < -3.14159)
            {
                q[index_i][index_j] = q[index_i][index_j] + 2 * M_PI;

            } else if(q[index_i][index_j] >= 3.14159)
            {
                q[index_i][index_j] = q[index_i][index_j] - 2 * M_PI;
            }
        }
    }


    for (int temp_cow = 0;temp_cow < 8;temp_cow++)
    {
        for(int temp_col = 0; temp_col < 6; temp_col++)
        {
            Q_result(temp_cow, temp_col) = q[temp_cow][temp_col];
        }
    }

    Result.resize(8,6);
    int j=0;
    int count = 8;
    for (int i=0;i<8;i++)
    {
        if  (std::isnan( Q_result(i,2)))
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
