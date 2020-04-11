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
NUM_DRONES = 5 ;
% position = zeros(NUM_DRONES,3) ;
colorcode = lines( 5 ) ;

%% cycle to show movement
clf
set( gcf , 'position', [ 1130 432 970 922 ] )
t = cell(1,10) ;
position = cell(1,10) ;
for drone_id = 1 : NUM_DRONES
    t{drone_id} = 0 ;
end

while ( 1 )
    for drone_id = 1 : NUM_DRONES
        t{drone_id} = t{drone_id} + 1 ;
        range = drone_id * STATE_SIZE + ( BYTE_POSITION + (1:TRUPLE_SIZE) ) ;
        position{drone_id}( t{drone_id} , 1 : 3 ) = double(typecast( layout_memoryshare.Data( range ) , 'single' )) ;
        range = drone_id * STATE_SIZE + ( BYTE_YAW + (1:4) ) ;
        yaw{drone_id}( t{drone_id} ) = double(typecast( layout_memoryshare.Data( range ) , 'single' )) ;
    end
    
    clf
    hold on
    grid on

    for drone_id = 1 : NUM_DRONES
        plot(   position{drone_id}( 1:t{drone_id} , 1 ) , ...
                position{drone_id}( 1:t{drone_id} , 2 ) , ...
                '-.', 'color' , colorcode ( drone_id , : ) )
        
		tmpyaw_deg = yaw{drone_id}( t{drone_id} ) + 90 ;
		tmpyaw_rad = tmpyaw_deg*pi/180 ;
		R = [ cos( tmpyaw_rad ) , -sin( tmpyaw_rad) ; sin( tmpyaw_rad) , cos( tmpyaw_rad ) ] ;
		
		% drones 4 rotors
		x = position{drone_id}( t{drone_id} , 1 ) ;
		y = position{drone_id}( t{drone_id} , 2 ) ;
		offset = pi/4 ;
		for i = 1 :4 
			offset = offset + pi/2 ;
			xx(i) = x + 2*cos(tmpyaw_rad+offset) ;
			yy(i) = y + 2*sin(tmpyaw_rad+offset) ;
		end
		plot( x   , ...
              y   , ...
                'ko', ...
                'markerfacecolor' , colorcode ( drone_id , : ) ,...
                'markersize' , 10 )
                
		plot( xx  , ...
              yy  , ...
                'ko', 'markerfacecolor' , colorcode ( drone_id , : ) )
                
        
		
		% front tip
		plot(   position{drone_id}( t{drone_id} , 1 ) + 2*cos(tmpyaw_rad  ) , ...
                position{drone_id}( t{drone_id} , 2 ) + 2*sin( tmpyaw_rad )  , ...
                'ko', ...
                'markerfacecolor' , 'k',...
                'markersize', 4 )       
         
        % label        
        text(	position{drone_id}( t{drone_id} , 1 ) + -0.5 , ...
                position{drone_id}( t{drone_id} , 2 ) + 0.5  , ...
				sprintf('%01d', drone_id  ) , 'color','w' ) ;

    end
    %axis([-1 1  -1 1] * 80 )
    axis square
    axis equal
    title(sprintf('Clock: %0.4fs' , 1/1e6*single(getSimClock( clock_memoryshare )) ) )
    pause(0.5)
end






