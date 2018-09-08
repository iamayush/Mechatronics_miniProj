/* Demonstration of Localization and Mapping
 * using Minimal and Inexpensive Components
 * (For SE 243: Mechatronics mini project)
 *
 * explore_traj.c
 * Trajectory for exploration of world
 * Created on: 08-Apr-2018
 * Author: Ayush Sinha
 *
 *                  World
 *          ---------------------
 *          | 15 | 14 | 13 | 12 |
 *          ---------------------
 *          |  8 |  9 | 10 | 11 |
 *          ---------------------
 *          |  7 |  6 |  5 |  4 |
 *          ---------------------
 *          | 0  |  1 |  2 |  3 |
 *          ---------------------
 */
int explore_traj[7][3] = {0}; // 7 straight line trajectories reqd to explore world
/* row   = # of trajectory
 * col 0 = # of cells to travel forward = cell_cnt
 * col 1 = Left(0) or Right(1) turn
 * col 2 = start cell number */
void exploration(void){
    explore_traj[0][0] = 3;
    explore_traj[0][1] = 0;
    explore_traj[0][2] = 0;

    explore_traj[1][0] = 1;
    explore_traj[1][1] = 0;
    explore_traj[1][2] = 3;

    explore_traj[2][0] = 3;
    explore_traj[2][1] = 1;
    explore_traj[2][2] = 4;

    explore_traj[3][0] = 1;
    explore_traj[3][1] = 1;
    explore_traj[3][2] = 7;

    explore_traj[4][0] = 3;
    explore_traj[4][1] = 0;
    explore_traj[4][2] = 8;

    explore_traj[5][0] = 1;
    explore_traj[5][1] = 0;
    explore_traj[5][2] = 11;

    explore_traj[6][0] = 3;
    explore_traj[6][1] = 0;
    explore_traj[6][2] = 12;
}

