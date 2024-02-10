#ifndef POSITION_H
#define POSITION_H

// Drone position control
namespace dpc
{
    class grid_coordinates
    {
        private:
            long long general_position[3];
            double particular_position[3];
            double precisions[3];
            double angle;
        public:
            int get_coordinate(short coordinate);
            void change_coordinates(short coordinate_system);
            void update_position(double *move_amount, double delta_time);
    }
}

#endif