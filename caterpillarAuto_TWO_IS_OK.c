#include "kilolib.h"

#define SEED_ID 0
#define GRADIENT_MAX 255
#define FORMED_OK 1
#define FORMED_NO 0

#define TIME_CHECK_MAXER 32 * 5 // 5s
#define TIME_CHECK_MINOR 32 * 10
#define TIME_LAST_GRADIENT 32 * 5 // 3s
#define TIME_LAST_MOTION_UPDATE 32 * 0.4 // 0.4

#define ADJUSTSIZEGENERATION 3
#define SUCESSRATE 0.2

#define DISTANCE_GRADIENT 70  // 70mm
#define DISTANCE_MIN 33 // 33mm
#define DISTANCE_MAX 100 // 100mm
#define DISTANCE_MOVE 53 // 60mm
#define DISTANCE_COLLIDE 38 // 45mm
#define DISTANCE_STOP 35 // 40mm

#define STOP 0
#define FORWARD 1
#define LEFT 2
#define RIGHT 3
#define MOVE 4
#define COMPLETED 5

#define NUM_RETAIN 1 // 1, cannot lost; 0, lost after some time.

#define GRADIENT_MAX_YES 1
#define GRADIENT_MAX_NO 0
#define GRADIENT_MINOR_YES 1
#define GRADIENT_MINOR_NO 0
#define YES 1
#define NO 0
#define UPDATE 1
#define UNUPDATE 0
#define LOGIC_CLOSER 0
#define LOGIC_EQUAL 1
#define LOGIC_FARER 2

int last_first_logic = LOGIC_EQUAL;
int last_second_logic = LOGIC_EQUAL;

int flag_maxest = GRADIENT_MAX_NO;
int flag_minor = GRADIENT_MINOR_NO;
int update_distance_to_motivated = UNUPDATE;
int update_distance_to_motivator = UNUPDATE;
int update_state_motivated = UNUPDATE;
int update_state_motivator = UNUPDATE;

int own_gradient = GRADIENT_MAX;
int received_gradient = 0;
int state_motivated = STOP;
int state_motivator = STOP;
int state_myself = STOP;

int current_motion = FORWARD;
int next_motion = FORWARD;
int num_retain = 0;
int num_stop = 0;
int my_fault = YES;

int offspring = FORWARD;
// This mutateProb guarantee the invariant of the current motion.
int probMutate = 0.7;

int distance = DISTANCE_STOP;
int distance_to_motivator = DISTANCE_MOVE;
int distance_to_motivated = DISTANCE_MAX;
int distance_to_motivated_parent = DISTANCE_MAX;

int formed_state = FORMED_NO;
uint32_t last_gradient_anchored;
uint32_t last_found_maxer;
uint32_t last_found_minor;
uint32_t last_motion_update;
message_t message;


// Generate 0 or 1 randomly
int randBinary(){
    // Generate an 8-bit random number (between 0 and 2^8 - 1 = 255).
    int random_number = rand_hard();

    // Compute the remainder of random_number when divided by 2.
    // This gives a new random number in the set {0, 1}.
    int random_direction = (random_number % 2);

    return random_direction;
}

// Generate a random number in the closed interval (0, 1).
float rand(){
    // Generate an 8-bit random number (between 0 and 2^8 - 1 = 255).
    int random_number = rand_hard();

    float result = random_number / 255;

    return result;
}


// Function to set led states
void set_led()
{
    // Set the LED color based on the gradient.
    switch (own_gradient) {
        case 0:
            set_color(RGB(1, 1, 1)); // White
            break;
        case 1:
            set_color(RGB(1, 0, 0)); // Red
            break;
        case 2:
            set_color(RGB(0, 1, 0)); // Green
            break;
        case 3:
            set_color(RGB(0, 0, 1)); // Blue
            break;
        case 4:
            set_color(RGB(1, 1, 0)); // Yellow
            break;
        default:
            set_color(RGB(1, 0, 1)); // Magneta
            break;
    }
    //  set_color(RGB(0, 1, 1));
}

// Function to handle motion.
void set_motion(int new_motion)
{   
    // Only take an an action if the motion is being changed.
    if (current_motion != new_motion)
    {   
        current_motion = new_motion;
        
        if (current_motion == STOP)
        {   
            set_motors(0, 0);
        }
        else if (current_motion == FORWARD)
        {   
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (current_motion == LEFT)
        {   
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
        else if (current_motion == RIGHT)
        {   
            spinup_motors();
            set_motors(0, kilo_turn_right);
        }
    }
}

void setup()
{   
    //If the robot is the seed, its gradient should be 0: overwrite the 
    // previously set value of GRADIENT_MAX.
    if (kilo_uid == SEED_ID)
    {   
        own_gradient = 0;
		distance_to_motivator = DISTANCE_MOVE;
		update_distance_to_motivator = UPDATE;
		update_state_motivator = UPDATE;
		flag_minor = GRADIENT_MINOR_YES;
		state_motivator = COMPLETED;
    }   
        
    // Set the transmission message.
    message.type = NORMAL;
    message.data[0] = own_gradient;
	// Sequence has not been formed completely.
	message.data[1] = formed_state;
	message.data[2] = state_motivator;
	message.crc = message_crc(&message);
}

void check_own_gradient() {
	// If no neighbors detected within IME_LAST_GRADIENT seconds
	// then sleep waiting for be activated.
    if ( (kilo_uid != SEED_ID) && (kilo_ticks > (last_gradient_anchored + TIME_LAST_GRADIENT)) && (own_gradient < GRADIENT_MAX))
    {   
        own_gradient = GRADIENT_MAX;
		formed_state = FORMED_NO;
    }  
}


void move() {
	int next_motion = offspring;
	 set_color(RGB(0, 0, 0));
	// closer and closer
	if (distance_to_motivated < distance_to_motivated_parent) 
	{
		set_color(RGB(1, 0, 0));
		last_second_logic = last_first_logic;
		last_first_logic = LOGIC_CLOSER;
		//num_retain = NUM_RETAIN + 1; 
		next_motion = offspring;
	}
	// farer and farer
	// If the distance_to_motivated keep unchanged, it is unusual.
	else if (distance_to_motivated > distance_to_motivated_parent)
	{
		set_color(RGB(0, 1, 0));
		last_second_logic = last_first_logic;
		last_first_logic = LOGIC_FARER;
		//num_retain = NUM_RETAIN;
		// It's not my fault, so I continue my movement as before.
		if (my_fault == NO)
		{
			my_fault = YES;
			next_motion = offspring;
		}
		else
		{
			switch (offspring)
			{
				case LEFT:
					next_motion = RIGHT;
					break;
				case RIGHT:
					next_motion = LEFT;
					break;
				case FORWARD:
					if (randBinary() == 1) 
					{
						next_motion = LEFT;
					} 
					else 
					{
						next_motion = RIGHT;
					}   
					break;
				default:
					break;
			}
		}
	}
	else
	{
		if (flag_maxest == GRADIENT_MAX_YES)
		{
			//num_retain = 0;
			next_motion = FORWARD;
		}
		else
		{
			set_color(RGB(0, 0, 1));
			if ((last_second_logic != LOGIC_FARER) && (last_first_logic == LOGIC_CLOSER))
			{
				// Update.
				//num_retain = NUM_RETAIN;
	        	switch (offspring)
				{
					case LEFT:
            		   	next_motion = RIGHT;
        	        	break;
		            case RIGHT:
	    	           	next_motion = LEFT;
        	        	break;
            		case FORWARD:
                		if (randBinary() == 1) 
						{
                    		next_motion = LEFT;
                		} else 
						{
               	    		next_motion = RIGHT;
           	    		}   
       	        		break;
   	        		default:
						break;
        		}
			}
			else
			{
				next_motion = offspring;
			}
			// Update.
			last_second_logic = last_first_logic;
			last_first_logic = LOGIC_EQUAL;
		}
	}
	
		
	offspring = next_motion;
	set_motion(offspring);
	distance_to_motivated_parent = distance_to_motivated;

}

void loop() {
	//set_color(RGB(0, 0, 0));
	//set_led();
	check_own_gradient();
	// Move only when the sequence has already formed.
	if ((formed_state == FORMED_OK) && (own_gradient < GRADIENT_MAX))
	{
		//set_color(RGB(0, 0, 0));
		//if ((update_state_motivator == UPDATE) && (update_state_motivated == UPDATE) && (state_motivator == COMPLETED) && (state_motivated != MOVE)) 
		/*	
		*/

		if ((state_motivator == COMPLETED) && (state_motivated != MOVE))
		{
			// Update.
			if (flag_minor == GRADIENT_MINOR_NO)
			{
				update_state_motivator = UNUPDATE;
			}
			
			if (flag_maxest == GRADIENT_MAX_NO)
			{
				update_state_motivated = UNUPDATE;
			}

			// The motion cannot be updated too fast,
			// otherwise the kilobots will be crazy 
			// due to frequency motion changes.
			// If leave the motivator too near,
			// and leave the motivated far enough,
			// then I can move.

			if ((distance_to_motivator <= DISTANCE_MOVE) && (distance_to_motivated > DISTANCE_COLLIDE))
			{
				//set_color(RGB(0, 1, 0));
			    if (kilo_ticks > (last_motion_update + TIME_LAST_MOTION_UPDATE))
				{
					last_motion_update = kilo_ticks;
					state_myself = MOVE;
					move();
				}
			}
			else
			{
				//set_color(RGB(0, 0, 1));
				set_motion(STOP);
				state_myself = COMPLETED;
				num_retain = 0;
			}
		}
		else
		{
			//set_color(RGB(0, 0, 0));
			set_motion(STOP);
		}
    }
	else
	{
		set_motion(STOP);
		num_retain = 0;
	}
}


message_t *message_tx()
{
	message.data[0] = own_gradient;
	message.data[1] = formed_state;
	message.data[2] = state_myself;
	/*		
	switch (state_myself)
	{
		case STOP:
			set_color(RGB(1, 0, 0));
			break;
		case MOVE:
			set_color(RGB(0, 1, 0));
			break;
		case COMPLETED:
			set_color(RGB(0, 0, 1));
			break;
		default:
			 set_color(RGB(0, 1, 1));
			 break;
	}
	*/
	message.crc = message_crc(&message);
    return &message;
}

void message_rx(message_t *m, distance_measurement_t *d)
{
	//set_color(RGB(1, 0, 0));
    received_gradient = m->data[0];
	// Assure that the received data is valid.
	if (received_gradient != own_gradient)  
	{
    	distance = estimate_distance(d);
		// Valid data in the valid distance.
		if (distance <= DISTANCE_GRADIENT)
		{
			last_gradient_anchored = kilo_ticks;
			// The message was sent by my motivated.
			// I found someone's gradient maxer than mine in the world.
			// My formed state is determined by my maxer.
			if (received_gradient > own_gradient)
			{
				last_found_maxer = kilo_ticks;
				if (received_gradient == (own_gradient + 1))
				{
					formed_state = m->data[1];
					state_motivated =  m->data[2];
					/*
					switch (state_motivated)
					{
					case STOP:
						set_color(RGB(1, 0, 0));
						break;
					case MOVE:
						set_color(RGB(0, 1, 0));
						break;
					case COMPLETED:
						set_color(RGB(0, 0, 1));
						break;
					default:
						set_color(RGB(0, 1, 1));
						break;
					}
					*/
					update_state_motivated = UPDATE;
					if (state_motivated != MOVE) {
						if ((num_stop ++) == 1) 
						{
							my_fault = NO;
						}
						distance_to_motivated = distance;
						update_distance_to_motivated = UPDATE;
					}
					else
					{
						num_stop = 0;
					}
				}
			}
			// The message was sent by my motivator.
			// (received_gradient < own_gradient)
			else
			{
				last_found_minor = kilo_ticks;
				own_gradient = received_gradient + 1;
				state_motivator =  m->data[2];
				update_state_motivator = UPDATE;
				if (state_motivator != MOVE)
				{
					distance_to_motivator = distance;
					update_distance_to_motivator = UPDATE;
				}
			}
		}
		
		flag_maxest = GRADIENT_MAX_NO;
		
		// I have neighbours whose gradient is minor than mine.
		// Meanwhile long time no find gradient maxer than mine.
		// I am the maxest one in my local world.
		if((kilo_uid != SEED_ID) && (kilo_ticks > (last_found_maxer + TIME_CHECK_MAXER)))
		{
			formed_state = FORMED_OK;
			distance_to_motivated = DISTANCE_MAX;
			distance_to_motivated_parent = distance_to_motivated;
			update_distance_to_motivated = UPDATE;
			state_motivated = COMPLETED;
			update_state_motivated = UPDATE;
			flag_maxest = GRADIENT_MAX_YES;
		}	
		/*	
		// I am the minor in the sequence.
		if (kilo_ticks > (last_found_minor + TIME_CHECK_MINOR))
		{
			flag_minor = GRADIENT_MINOR_YES;
			distance_to_motivator = DISTANCE_MOVE;
			update_distance_to_motivator = UPDATE;
			state_motivator = COMPLETED;
			update_state_motivator = UPDATE;
			set_color(RGB(1, 0, 1));
		}
		*/
	}	
}


int main()
{
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_message_tx = message_tx;
    kilo_start(setup, loop);

    return 0;
}
