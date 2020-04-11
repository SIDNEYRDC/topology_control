%% VIZUALIZER for the the drone_simulator

PLOT_PERIOD = 0.5e6 ; % period to plot pdr

%% Opening the sim clock (mem share)
try
    clock_memoryshare = memmapfile('/dev/shm/clock_memspace') ;
catch
    disp('please run ./simulator first')
    return
end

%% Opening the sim layout (mem share)
try
    layout_memoryshare = memmapfile('/dev/shm/layout_memspace') ;
catch
    disp('please run ./drone_simulator first')
    return
end


%% from layout.h %%%%%%%%%%%%%%%%%%%%%%%%%%
% typedef struct {
% 	float X ; /* 4 bytes */
% 	float Y ; /* 4 bytes */
% 	float Z ; /* 4 bytes */
% } truple_t ; /* 12 bytes */
%
% typedef struct {
% 	/* current physics */
% 	truple_t	position ; /* 12 bytes */
% 	truple_t	velocity ; /* 12 bytes */
% 	float yaw_degrees ;	 /* 4 bytes */
% 	/* current actuation */
% 	truple_t	thrust ; /* 12 bytes */
% 	float		torque ; /* 4 bytes */
%
% } state_t ; /* 44 bytes */
%
% typedef struct {
% 	state_t state[MAX_NUM_DRONES+1] ; // lets ignore node 0
% } layout_t ; /* (10+1)*44 bytes = 484 bytes */
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% semaphore
% try
%     semaphore('w'); %wait
%     semaphore('p'); %post
% catch
%     mex -O -v semaphore.c % compile in case it wasn't
% end

%% get data
BYTE_POSITION = 0 ;
BYTE_VELOCITY = BYTE_POSITION + 12 ;
BYTE_YAW = BYTE_VELOCITY + 12 ;
BYTE_THRUST = BYTE_YAW + 4 ;
BYTE_TORQUE = BYTE_THRUST + 12 ;
STATE_SIZE = 44 ;
TRUPLE_SIZE = 12 ;
NUM_DRONES = 10 ;
% position = zeros(NUM_DRONES,3) ;


%% cycle to show movement
clf
set( gcf , 'position', [ 113  432 570 422 ] )
t = cell(1,10) ;
position = cell(1,10) ;
for drone_id = 1 : NUM_DRONES
    t{drone_id} = 0 ;
end

while ( 1 )
    for drone_id = 1 : NUM_DRONES
        range = drone_id * STATE_SIZE + ( BYTE_POSITION + (1:TRUPLE_SIZE) ) ;
        t{drone_id} = t{drone_id} + 1 ;
        position{drone_id}( t{drone_id} , 1 : 3 ) = double(typecast( layout_memoryshare.Data( range ) , 'single' )) ;
    end
    
    clf
    hold on
    grid on

    for drone_id = 1 : NUM_DRONES
        plot(   position{drone_id}( 1:t{drone_id} , 1 ) , ...
                position{drone_id}( 1:t{drone_id} , 2 ) , ...
                '-k')
        plot(   position{drone_id}( t{drone_id} , 1 ) , ...
                position{drone_id}( t{drone_id} , 2 ) , ...
                'ok')
            
        text(	position{drone_id}( t{drone_id} , 1 ) ,...
				position{drone_id}( t{drone_id} , 2 ) , ...
				sprintf('#%d', drone_id) ) ;

    end
    axis([-1 1  -1 1] * 80 )
    axis square
    title(sprintf('Clock: %0.4fs' , 1/1e6*single(getSimClock( clock_memoryshare )) ) )
    pause(0.5)
end






