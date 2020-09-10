%% Intialize path, get bag file
% Define path and contents of directory
date = "09-07-2020";
addpath("./"+date);
contents = dir("./"+date);
% Get bag file
filename = "./"+date+"/"+contents(end).name;
bag = rosbag(filename);

%% Extract Data from bag file
cmd_si_subset = select(bag,'Topic','/asctec/cmd_si');
cmd_si_bag_data = readMessages(cmd_si_subset,'DataFormat','struct');
mpc_state_subset = select(bag,'Topic','/asctec/mpc_state');
mpc_state_bag_data = readMessages(mpc_state_subset,'DataFormat','struct');

num_cmd_si = length(cmd_si_bag_data);
num_mpc_state = length(mpc_state_bag_data);
cmd_si_data = struct("t",zeros(num_cmd_si,1),...
    "thrust",zeros(num_cmd_si,1),...
    "roll",zeros(num_cmd_si,1),...
    "pitch",zeros(num_cmd_si,1),...
    "yaw",zeros(num_cmd_si,1));
mpc_state_data = struct("t",zeros(num_mpc_state,1),...
    "x",zeros(num_mpc_state,1),...
    "y",zeros(num_mpc_state,1),...
    "z",zeros(num_mpc_state,1),...
    "thrust",zeros(num_mpc_state,1),...
    "roll",zeros(num_mpc_state,1),...
    "pitch",zeros(num_mpc_state,1),...
    "yaw",zeros(num_mpc_state,1));
for i = 1:length(mpc_state_bag_data)
    t = table2array(mpc_state_subset.MessageList(i,"Time"));
    x = mpc_state_bag_data{i}.Pose.Pose.Position.X;
    y = mpc_state_bag_data{i}.Pose.Pose.Position.Y;
    z = mpc_state_bag_data{i}.Pose.Pose.Position.Z;
    thrust = mpc_state_bag_data{i}.Pose.Covariance(1);
    roll = mpc_state_bag_data{i}.Pose.Covariance(2);
    pitch = mpc_state_bag_data{i}.Pose.Covariance(3);
    yaw = mpc_state_bag_data{i}.Pose.Covariance(4);
    mpc_state_data.t(i) = t;
    mpc_state_data.x(i) = x;
    mpc_state_data.y(i) = y;
    mpc_state_data.z(i) = z;
    mpc_state_data.thrust(i) = thrust;
    mpc_state_data.roll(i) = roll;
    mpc_state_data.pitch(i) = pitch;
    mpc_state_data.yaw(i) = yaw;
end

tstart = mpc_state_data.t(1);
mpc_state_data.t = mpc_state_data.t - tstart;

%% Plot
close all;
figure(1);
hold on;
yyaxis left;
plot(mpc_state_data.t,mpc_state_data.z);
ylabel("Z pos (m)");
yyaxis right;
plot(mpc_state_data.t,mpc_state_data.thrust);
ylabel("Thrust (N)");
xlabel("time (s)");

