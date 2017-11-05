## Project: Kinematics Pick & Place

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/joints.jpg
[image2]: ./misc_images/t1.jpg
[image3]: ./misc_images/t2.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Annotated figure of the robot with proper link assignments and joint rotations:

![alt text][image1]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

I used the diagrams provided in the videos on Forward Kinematics introducing the project as a starting point. The alpha column contians twist angles from z_i-1 to z_i about x_i-1 according to the right hand rule. The a column denotes distance between the two z axes along the x_i axis, which I found by referencing the diagram and the urdf file. The d column contains the distance between the x axes along the z_i axis, which I also found by referencing the diagram and the urdf file. All joints are revolute except the end effector in the last row, so they just had the associated theta in that column. The exception to this is q2, which requires a nonzero constand offset since the x axes are not parallel.  

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Transformation matrices about each joint:
```
    def TF_Matrix(alpha, a, d, q):
        TF = Matrix([
            [           cos(q),           -sin(q),           0,             a],
            [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
            [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
            [                0,                 0,           0,             1]])
        return TF

    T0_1 =  TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
    T1_2 =  TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
    T2_3 =  TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
    T3_4 =  TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
    T4_5 =  TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
    T5_6 =  TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
    T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE
```

I defined my homogenous transform as a function called TF_Matrix I could call it with different parameter values and use the subs function to populate values from the DH_Table I constructed above. The TO_EE variable contains the generalized homogenous transform from the base link to the end effector. 

The homogenous transform uses the alpha column to get the twist angle between the link frames. It plugs in values from the *a* column in [0, 3] to provide the *x* offset between the link frames. Then, the *d* column is used with the sine and cosine of the alpha value to get the *y* and *z* offsets. 

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Images of notes with more detail following:

![alt text][image2]
![alt text][image3]

Since the Kuka KR210's last three joints can be characterized as a spherical wrist, as the last three joints are revolute with axes that intersect at a single point. Thus, we can decouple the IK problem into Inverse Position Kinematics on joints 1 to 3 and Inverse Orientation Kinematics on joints 4 to 6. 

Before deriving th equations for the individual joint angles, we need to get the position of the wrist center relative to the base. I accomplish this by defining rotation matrices for roll, pitch, and yaw of the end effector, as described in the Composition of Rotations section of the project. I multiplied the matrices in sequence using the extrinsic x-y-z convention, and then multipled the result with a rotation error matrix to account for the discrepancy between the URDF and DH reference frames for the end effector. The final step to get the wrist center is to subtract the distance from the end effector pose given by the request with the distance along the z axis to get to the wrist center origin, and then multiplying this by the ROT_EE matrix derived using the steps in this paragraph. The following is my code for finding the wrist center relative to the base. 

```
            r, p, y = symbols('r p y')

            ROT_x = Matrix([[1,      0,       0],
                            [0, cos(r), -sin(r)],
                            [0, sin(r), cos(r)]]) # ROLL

            ROT_y = Matrix([[cos(p),  0,  sin(p)],
                            [0,       1,       0],
                            [-sin(p), 0,  cos(p)]]) # PITCH

            ROT_z = Matrix([[cos(y), -sin(y),  0],
                            [sin(y),  cos(y),  0],
                            [0,            0,  1]]) # YAW

            ROT_EE = ROT_z * ROT_y * ROT_x

            ROT_error = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))

            ROT_EE = ROT_EE * ROT_error
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            EE = Matrix([[px], 
                         [py],
                         [pz]])

            WC = EE - (0.303) * ROT_EE[:,2]
```

The next step is to calculate the joint angles for each joint. Theta1 is the angle from x to the position of the wrist center as seen from above, e.g. projected onto x and y. I calculated this value using atan2(WC[1], WC[0]). Next, I project the WC onto the z-y plane to get theta2 and theta3. I made use of the picture from the Inverse Kinematics with Kuka KR210 section to get each side of the triangle with vertices of WWC, base, and joint 3. Using these sides, I calculated the inner angles of the triangles using the Cosine Laws. From these inner angles, I derived theta2 and theta3 using basic triangle math. Referencing the Project Walkthrough, I accounted for the sag of link4 for theta3. The following is my code for this section. 

```
	    # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1], WC[0])
        
            # SSS triangle for theta2 and theta3
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            side_c = 1.25

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

            theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi / 2  - (angle_b + 0.036) # 0.036 accounts for sag in link4 of -0.054m
```

Now that Inverse Position Kinematics are completed, we turn to Inverse Orientation Kinematics to derive the three joint angles of the wrist center. I plugged in the first three joint angles into a subset of the transformation matrices of the first three joints to get the rotation matrices for those joints. Then, to get the remaining rotation matrices, I multipled the rotation matrix of the end effector with the inverse of the rotation matrix derived in the preceding sentence. The R0_3 and R3_6 matrices make it possible for me to calculate the angle offset for theta4, theta5, and theta6 following the math from the Euler Angles from a Rotation Matrix section of the lessons. I used atan2 and followed the equations to derive the three joint angles. The following is my code. 

```
            R0_3 = T0_1[0:3, 0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            R3_6 = R0_3.transpose() * ROT_EE

            # Euler angles from rotation matrix
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Please see the .MOV file in this directory for a video of a successful pick and place run. 

I ran into a significant hitch with my IK for the wrist center, as the wrist kept twisting and making unnecessary movements when moving to each point along the trajectory. I found that using .transpose() instead of .inv("LU") to get the inverse of the R0_3 rotation matrix resolved the problem. 

If I were to improve my work, I would try to make it faster by saving the rotation matrices to a file instead of calculating them each time. I would also take up the extra credit to calculate and plot the error of the end effector pose.  

