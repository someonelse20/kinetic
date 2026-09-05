import os
import numpy as np
from ahrs.filters import EKF
from ahrs.common.orientation import acc2q

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath('.')), 'recordings')

data = np.genfromtxt(os.path.join(DATA_DIR, 'static.csv'), delimiter=',')
gyro_data = data[:, :3]
acc_data = data[:, 3:6]
mag_data = data[:, 6:9]

gyro_data_T = gyro_data.T
acc_data_T = acc_data.T
mag_data_T = mag_data.T

num_samples = len(acc_data_T[0])

acc0 = np.array([gyro_data_T[0, 0], gyro_data_T[1, 0], gyro_data_T[2, 0]])
print('acc0:', acc0)

ekf = EKF()
Q = np.zeros((num_samples, 4))
Q[0] = acc2q(acc0)

for t in range(1, 3):
    # Each row is a sensor axis, each column is a time step
    gyro = np.array([gyro_data_T[0, t], gyro_data_T[1, t], gyro_data_T[2, t]])
    acc = np.array([acc_data_T[0, t], acc_data_T[1, t], acc_data_T[2, t]])
    mag = np.array([mag_data_T[0, t], mag_data_T[1, t], mag_data_T[2, t]])
    print(f't={t}: gyro={gyro}, acc={acc}, mag={mag}')
    Q[t] = ekf.update(Q[t - 1], gyro, acc, mag)

print('Success!')
