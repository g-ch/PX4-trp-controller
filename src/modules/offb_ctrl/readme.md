全局坐标系使用NED FRD 使用
INT传递数据 可以设置SCALE

[这篇文章讲的很好](http://www.jeepxie.net/article/407925.html)

使用自带数学库的方法:

```cpp
math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	math::Matrix<3, 3> R = q_att.to_dcm();
    //通过math库构建四元数；获取DCM的函数原型：无可厚非，都懂的
	/*** create rotation matrix for the quaternion */
	Matrix<3, 3> to_dcm(void) const {
		Matrix<3, 3> R;
		float aSq = data[0] * data[0];
		float bSq = data[1] * data[1];
		float cSq = data[2] * data[2];
		float dSq = data[3] * data[3];
		R.data[0][0] = aSq + bSq - cSq - dSq;
		R.data[0][1] = 2.0f * (data[1] * data[2] - data[0] * data[3]);
		R.data[0][2] = 2.0f * (data[0] * data[2] + data[1] * data[3]);
		R.data[1][0] = 2.0f * (data[1] * data[2] + data[0] * data[3]);
		R.data[1][1] = aSq - bSq + cSq - dSq;
		R.data[1][2] = 2.0f * (data[2] * data[3] - data[0] * data[1]);
		R.data[2][0] = 2.0f * (data[1] * data[3] - data[0] * data[2]);
		R.data[2][1] = 2.0f * (data[0] * data[1] + data[2] * data[3]);
		R.data[2][2] = aSq - bSq - cSq + dSq;
		return R;
		math::Vector<3> e_R = R.transposed() * (R_z % R_sp_z);// % 叉积运算，× 矩阵乘法运算
	}

```

注意串口号:
```
# UART mapping on OMNIBUSF4SD:
#
# USART1		/dev/ttyS0		SerialRX
# USART4		/dev/ttyS1		TELEM1
# USART6		/dev/ttyS2		GPS
#
# UART mapping on FMUv2/3/4:
#
# UART1			/dev/ttyS0		IO debug (except v4, 
# UART1			/dev/ttyS0		IO debug (except v4,here ttyS0 is the wifi)
# USART2		/dev/ttyS1		TELEM1 (flow control)
# USART3		/dev/ttyS2		TELEM2 (flow control)
# UART4         /dev/ttyS3      GPS(默认，对应源码查看后确定)
# UART7							CONSOLE
# UART8							SERIAL4
#
#
# UART mapping on FMUv5:
#
# UART1			/dev/ttyS0		GPS
# USART2		/dev/ttyS1		TELEM1 (flow control)
# USART3		/dev/ttyS2		TELEM2 (flow control)
# UART4			/dev/ttyS3		TELEM4
# USART6		/dev/ttyS4		TELEM3 (flow control)
# UART7			/dev/ttyS5		?
# UART8			/dev/ttyS6		CONSOLE
```

注意串口号使用时可以在参数中寻找 mav 选项禁用