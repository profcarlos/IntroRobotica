m = mode();
mode(7)
//
// Forward kinematics is the problem of solving the Cartesian position and 
// orientation of a mechanism given knowledge of the kinematic structure and 
// the joint coordinates.
//
// We will work with a model of the Motoman HP6 robot
// Now we create kinematic data of a Motoman HP6 manipulator
//
//##########################################################
L = list();
//            theta    d      a      alpha
L(1) = Link([ 0       0      0.15   -%pi/2   0]);
L(2) = Link([ 0       0      0.57   -%pi     0]);
L(3) = Link([ 0       0      0.155  -%pi/2   0]);
L(4) = Link([ 0      -0.635  0       %pi/2   0]);
L(5) = Link([ 0       0      0      -%pi/2   0]);
L(6) = Link([ 0      -0.095  0       %pi     0]);

qz = [0 0 0 0 0 0]; // zero angles, L shaped pose
//Pose 0; At ZERO position
qready = [-%pi/4 %pi/8 -%pi/8 0 -%pi/4 0];
// ready pose, arm up
q0 =[0   -%pi/2   0   0   -%pi/2   0];
mHP6=SerialLink(L, 'name', 'Motoman HP6');
//##########################################################

// Consider the Motoman HP6 example again, and the joint coordinates of zero,
// which are defined by the script

qz

// The forward kinematics may be computed using fkine() method of the
// mHP6 robot object

fkine(mHP6,qz)

// returns the homogeneous transform corresponding to the last link of the 
// manipulator
//
// fkine() can also be used with a time sequence of joint coordinates, or 
// trajectory, which is generated by jtraj()

t = [0:.05:10];    // generate a time vector
q = jtraj(qz, qready, t); // compute the joint coordinate trajectory

mprintf("\n q [%s] : %dx%d",typeof(q),size(q,1),size(q,2));

// then the homogeneous transform for each set of joint coordinates is given by

T = fkine(mHP6,q);

mprintf("\n T [%s] : %dx%dx%d",typeof(T),size(T,1),size(T,2),size(T,3))

// where T is a 3-dimensional matrix, the first two dimensions are a 4x4 
// homogeneous transformation and the third dimension is time.
//
// For example, the first point is

T(:,:,1)

// and the tenth point is

T(:,:,10)
//
// Elements (1:3,4) correspond to the X, Y and Z coordinates respectively, and 
// may be plotted against time
scf(100001); subplot(3,1,1); plot(t, squeeze(T(1,4,:)));
xlabel('Time (s)');
ylabel('X (m)')
subplot(3,1,2); plot(t, squeeze(T(2,4,:)));
xlabel('Time (s)');
ylabel('Y (m)')
subplot(3,1,3); plot(t, squeeze(T(3,4,:)));
xlabel('Time (s)');
ylabel('Z (m)')

// or we could plot X against Z to get some idea of the Cartesian path followed
// by the manipulator.

scf(100002); plot(squeeze(T(1,4,:)), squeeze(T(3,4,:)));
xlabel('X (m)')
ylabel('Z (m)')
xgrid();
mode(m);

// the overloaded function plot() animates a stick figure robot moving 
// along a trajectory.
plot_robot(mHP6,q);
