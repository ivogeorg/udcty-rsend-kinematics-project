r, p, y = symbols('r p y')

R_x = rot_x(r)
R_y = rot_y(p)
R_z = rot_z(y)
# Rotation matrix of gripper
R_EE = R_x * R_y * R_z

# Compensate for rotation discrepancy between DH parameters and Gazebo
Rot_err = rot_z(rad(180)) * rot_y(rad(-90))
R_EE = R_EE * Rot_err
R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
R0_3 = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
R3_6 = R0_3.inv("LU") * R_EE
R3_6_sym = simplify(T3_4[0:3,0:3] * T4_5[0:3,0:3] * T5_6[0:3,0:3] * T6_EE[0:3,0:3])
print("R3_6 symbols:")
pprint(R3_6_sym)

## Cross-check the result. See if R3_6 calculation was correct.
val = R3_6_sym.evalf(subs={q4:test_case[2][3], q5:test_case[2][4], q6:test_case[2][5]})
print("R3_6:")
pprint(simplify(R3_6))

print("val:")
pprint(val)


# Result from the above code
# ==========================
R3_6 symbols:
Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]])
R3_6:
Matrix([
[ -4.91835708354135,   0.173439166571209,   7.84681438639773],
[  1.77091656753887, -0.0592223076246051, -0.942521376442524],
[-0.377771327578033,   0.121848397167936,  0.917846279159825]])
val:
Matrix([
[-0.0215741596901083, -0.910384486857935, -0.413200486110654],
[  0.626768033475282, -0.334310639746825,  0.703845315652236],
[ -0.778907175372774, -0.243795984873589,  0.577812365662352]])
