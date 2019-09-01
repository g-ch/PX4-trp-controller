使用自带数学库的方法
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