function R = quat_to_rot(q)
    qw = q(4); q1 = q(1); q2 = q(2); q3=q(3);
    % first row of the rotation matrix
    r00 = 2 * (qw * qw + q1 * q1) - 1;
    r01 = 2 * (q1 * q2 - qw * q3);
    r02 = 2 * (q1 * q3 + qw * q2);
    % second row of the rotation matrix
    r10 = 2 * (q1 * q2 + qw * q3);
    r11 = 2 * (qw * qw + q2 * q2) - 1;
    r12 = 2 * (q2 * q3 - qw * q1);
    % third row of the rotation matrix
    r20 = 2 * (q1 * q3 - qw * q2);
    r21 = 2 * (q2 * q3 + qw * q1);
    r22 = 2 * (qw * qw + q3 * q3) - 1;
    
    R = [r00, r01, r02;
         r10, r11, r12;
         r20, r21, r22];
end

