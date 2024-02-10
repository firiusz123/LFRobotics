#ifndef POSITION_H
#define POSITION_H

// Drone position control
namespace dpc
{
    class grid_coordinates
    {
        private:
            struct block
            {
                long long first_coordinate;
                long long second_coordinate;
                long long third_coordinate;
            };
            block general_position;
            struck in_block
            {
                double first_coordinate;
                double second_coordinate;
                double third_coordinate;
            };
            in_block particular_position;
        public:
            int get_coordinate(short coordinate);
            void change_coordinates(short coordinate_system);
            void update_position(double *move_amount, double time);
    }
}

#endif