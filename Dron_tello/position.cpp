#include "position.h"

using namespace dsp;

grid_coordinates::grid_coordinates()
{
            for(int i =0;i < 3;i++)
            {
                this->general_position[i] = 0;
                this->particular_position[i] = 0;
                this->precision[i] =  0;
            }
            this->tilt_angle = 0;
            this->system = Systems::Cartesian;
}

int grid_coordinates::get_coordinate(short coordinate)
{
    return this->general_position[coordinate]*this->precisions[coordinate]+this->particular_position[coordinate];
}

void grid_coordinates::change_coordinates(short coordinate_system)
{
    /*
    I'm planning to set up spherical and cartesian coordinates for now,
    thus we only need transforms of form cartesian->spherical and back,
    which can be easily done using a matrix, however in different scenarios 
    It would come in handy to place any other desired coordinate system, 
    so switching between them would require two steps
    arbitrary_first->cartesian->arbitrary_second 
    */
    
}

void grid_coordinates::update_position(double *move_amount,double delta_time)
{
    /*
    Here a matrix would be needed too, as move amount 
    would be counted as average velocity over the given delta time
    in Cartesian coordinates, so in case the position is in Cartesian
    we only need identity, but in any other case there needs to be an array
    of transformations in place
    */
    if(this->system == Systems::Cartesian)
    {
        for(i=0;i<3;i++)
        {
            this->particular_position[i]=move_amount[i]*delta_time;
            while(this->particular_position[i] >= this->precision[i])
            {
                this->particular_position[i] -= this->precision[i];
                this->general_position[i]++;
            }
            while(this->particular_position[i] < 0)
            {
                this->particular_position[i] += this->precision[i];
                this->general_position[i]--;
            }
        }
    }
}