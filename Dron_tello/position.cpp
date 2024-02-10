#include "position.h"

using namespace dsp;

int grid_coordinates::get_coordinate(short coordinate)
{
    return this.general_position[coordinate]*this.precisions[coordinate]+this.particular_position[coordinate];
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
}