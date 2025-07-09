theta_dot_b = 0.0; # this is the gyroscope bias reading. To be updated by the
# Kalman filter
dt = 100 # in milliseconds
#  To be tuned

Q_angle = 0.001;
Q_bias = 0.003;
pred_angle = 0;

# Covariance matrix P
P[0][0] = 1.0;  # Uncertainty in angle
P[0][1] = 0.0;
P[1][0] = 0.0;
P[1][1] = 1.0;  # Uncertainty in bias

R_measure = 0.03; # R is the uncertainity of the accelerometer angle.

# theta_dot_k is the raw gyroscope reading in rad/s
# theta_k is the angle estimate of the of gyro reading
# dt is the rate at which we get gyroscope readings

# Step 1: Calculating estimated angle at step k.
rate = theta_dot_k - theta_dot_b; # This is the
pred_angle += dt * rate; # new angle from the gyroscope

# Step 2: Calculating a priori state.

P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle); # Uncertainity of
# the predicted angle.
P[0][1] -= dt * P[1][1];
P[1][0] -= dt * P[1][1];
P[1][1] += Q_gyroBias * dt;

# Step 3:
y = newAngle - pred_angle; # new angle is z_k, raw value from accelerometer.

# Step 4: Innovation covariance
S = P[0][0] + R_measure;

# Step 5: Calculating Kalman Gain
K[0]= P[0][0] / S;
K[1] = P[1][0] / S;

#step 6: Update the state estimate
pred_angle+= K[0] * y;
bias += K[1] * y;

#Step 7: Update Error Covariane matrix P
P00_temp = P[0][0];
P01_temp = P[0][1];

P[0][0] -= K[0] * P00_temp;
P[0][1] -= K[0] * P01_temp;
P[1][0] -= K[1] * P00_temp;
P[1][1] -= K[1] * P01_temp;

