% port must be set to the communication port used by the Arduino
% you can find the port by going to 'tools -> Port' in the Arduion
% application. For a PC, it will be something like COM6, and for a Mac, it
% will be something like /dev/cu.usbmodem1451

port='COM5';
%period = 10;
obj = serial(port, 'BaudRate', 115200);
obj.terminator = char(10);
fopen(obj)
%
% do a read to get Ready! from Arduino
%
dummy = fgets(obj);
%
% Read and display some data
%
for i=1:5,
    data = fgets(obj);
    disp(data)
end;
%
% Read data after sending command to Arduino
%
disp('Starting Counting Event in Arduino')
fprintf(obj,'%s\n','S'); % send start signal to Arduino
data=[];
k=0

% Read Data from Arduino
data = fgets(obj);
% Display what you got
disp(data)
while (~strncmp(data,'Finished',8)) % Until Arduino signals that it is done
    k=k+1;
    % change string data to cell array using tab delimiter
    dataarray = strsplit(data,char(9));
    % save data converting strings to numbers
    Time(k) = eval(dataarray{1});
    rhoDot(k) = eval(dataarray{2});
    % Read Data from Arduino
    data = fgets(obj);
    % Display what you got
    disp(data)
   
end;
fclose(obj)

%
% Plot results
%
% K=0.208; %proportional gain
% out=sim('OpenLoopResponse'); %Send data to simulink

figure(1); clf;
plot(Time, rhoDot) %Experimental
xlabel('Time (ms)')
ylabel('Forward Velocity (m/s)')

% figure(2); clf;
% plot(out.velocity); %Simulated
% xlabel('Time (ms)')
% ylabel('Angular Velocity (rad/ms)')

%
% Save results in a .mat file to use later
%
save rhoExpData.mat Time rhoDot
    

    