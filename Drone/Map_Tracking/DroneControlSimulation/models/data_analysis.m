% Visualization of the drone path
x_path = out.x;
y_path = out.y;
z_path = out.z;

x = squeeze(x_path);
y = squeeze(y_path);
z = squeeze(z_path);

%x=x*-1;
%y=y*-1;
%z=z*-1;
figure(1)
plot3(x, y, z, 'LineWidth', 2) % Thicker line for better visualization
grid on

xlabel('X Position [m]')
ylabel('Y Position [m]')
zlabel('Z Position [m]')
title('3D Drone Path')

figure(2)
subplot(1, 3, 1)
plot(x)
xlabel('Time (s)')
ylabel('X Position (m)')
title('X Position vs Time')

subplot(1, 3, 2)
plot(y)
xlabel('Time (s)')
ylabel('Y Position (m)')
title('Y Position vs Time')

subplot(1, 3, 3)
plot(z)  
xlabel('Time (s)')
ylabel('Z Position (m)')
title('Z Position vs Time')