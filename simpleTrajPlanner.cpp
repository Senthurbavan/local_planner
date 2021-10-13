#include <vector>
#include <queue>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "trajectory.h"
#include "line_iterator.h"

using namespace cv;
using namespace std;

static const unsigned int ROW = 300;   //y
static const unsigned int COL = 300;   //x
static const unsigned int SCALE = 2; //xvvv
static const unsigned int UNREACHABLE_COST = ROW * COL + 1;
static const unsigned char OBSTACLE_COST = 127;

double resolution_ = 0.01;
double sim_time_ = 2.0;
double sim_granularity_ = 0.1;
double angular_sim_granularity_ = 0.1;

double path_distance_bias_ = 1.0;
double goal_distance_bias_ = 1.0;
double occdist_scale_ = 1.0;

// Creating a shortcut for int, int pair type
typedef pair<int, int> Pair;
typedef pair<double, double> Dpair;

// Creating a shortcut for pair<int, pair<int, int>> type
typedef pair<double, pair<int, int>> pPair;

struct GridCell
{
    int cx, cy;
    int target_dist;
    bool target_mark;
};

vector<vector<GridCell>> pathmap(ROW, vector<GridCell>(COL));
vector<vector<GridCell>> goalmap(ROW, vector<GridCell>(COL));
vector<vector<int>> costmap(ROW, vector<int>(COL, 0));
vector<Dpair> footprint_;
double footprint_array[4][2] = {{0.1, -0.12}, {0.1, 0.12}, {-0.1, 0.12}, {-0.1, -0.12}}; // (x, y) format

// A Utility Function to check whether given cell (row, col)
// is a valid cell or not.
bool isValid(int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL);
}

void fillFootprint(void)
{
    int footprint_len = sizeof(footprint_array) / sizeof(footprint_array[0]);
    for (int i = 0; i < footprint_len; i++)
    {
        Dpair point;
        point.first = footprint_array[i][1]; //p.first is y coordinate
        point.second = footprint_array[i][0];
        footprint_.push_back(point);
    }
    return;
}

bool worldToMap(double wx, double wy, int &mx, int &my)
{
    if (wx < 0 || wy < 0) // change when the real scenario is finalized
    {
        return false;
    }

    mx = (int)(wx / resolution_);
    my = (int)(wy / resolution_);

    if (mx >= COL || my >= ROW)
    {
        return false;
    }

    return true;
}

void mapToWorld(int mx, int my, double &wx, double &wy)
{
    wx = mx * resolution_;
    wy = my * resolution_;
}

// Compute velocity based on acceleration
double computeNewVelocity(double vg, double vi, double a_max, double dt)
{
    if ((vg - vi) >= 0)
    {
        return min(vg, vi + a_max * dt);
    }
    return max(vg, vi - a_max * dt);
}

// Compute x position based on velocity
double computeNewXPosition(double xi, double vx, double vy, double theta, double dt)
{
    return xi + (vx * cos(theta) + vy * cos(M_PI_2 + theta)) * dt;
}

// Compute y position based on velocity
double computeNewYPosition(double yi, double vx, double vy, double theta, double dt)
{
    return yi + (vx * sin(theta) + vy * sin(M_PI_2 + theta)) * dt;
}

// Compute theta position based on velocity
double computeNewThetaPosition(double thetai, double vth, double dt)
{
    return thetai + vth * dt;
}


/*
*
*
* Cost related functions start  
*
*
*/


double pointCost(int cx, int cy)
{
    int cost = costmap[cy][cx];
    if (cost == OBSTACLE_COST)
    {
        return -1;
    }
    return cost;
    
}

double lineCost(int cx0, int cx1, int cy0, int cy1)
{
    double line_cost = 0.0;
    double point_cost = -1.0;
    
    for (nsx::LineIterator line(cx0, cy0, cx1, cy1); line.isValid(); line.advance())
    {
        point_cost = pointCost(line.getX(), line.getY());

        if (point_cost < 0)
        {
            return point_cost;
        }

        if (line_cost < point_cost)
        {
            line_cost = point_cost;
        }
        return line_cost;
    }
    
    
}

// footprint in real coordiante aligned in robot pose, not in grid index
double footprintCost1(double x_i, double y_i, double theta_i, vector<Dpair>& footprint)
{
    // returns:
    //  -1 if footprint covers at least a lethal obstacle cell, or
    //  -2 if footprint covers at least a no-information cell, or
    //  -3 if footprint is [partially] outside of the map, or
    //  a positive value for traversable space

    int cell_x, cell_y;

    //get the cell coord of the center point of the robot
    if (!worldToMap(x_i, y_i, cell_x, cell_y))
    {
        return -3.0;
    }

    int x0, x1, y0, y1;
    double line_cost = 0.0;
    double footprint_cost = 0.0;

    for (int i = 0; i < footprint.size() - 1; i++)
    {
        //get the cell coord of the first point
        if (!worldToMap(footprint[i].second, footprint[i].first, x0, y0))
        {
            return -3.0;
        }
        //get the cell coord of the second point    
        if (!worldToMap(footprint[i+1].second, footprint[i+1].first, x1, y1))
        {
            return -3.0;
        }

        line_cost = lineCost(x0, x1, y0, y1);
        footprint_cost = max(line_cost, footprint_cost);

        //if there is an obstacle that hits the line... we know that we can return false right away
        if(line_cost < 0)
        {
            return line_cost;
        }
    }
    //we also need to connect the first point in the footprint to the last point
    //get the cell coord of the last point
    if (!worldToMap(footprint.back().second, footprint.back().first, x0, y0))
    {
        return -3.0;
    }
    //get the cell coord of the first point    
    if (!worldToMap(footprint.front().second, footprint.front().first, x1, y1))
    {
        return -3.0;
    }

    line_cost = lineCost(x0, x1, y0, y1);
    footprint_cost = max(line_cost, footprint_cost);

    //if there is an obstacle that hits the line... we know that we can return false right away
    if(line_cost < 0)
    {
        return line_cost;
    }

    return footprint_cost;
}

// x_i, y_i, theta_i are in world coordiante
double footprintCost(double x_i, double y_i, double theta_i)
{
    vector<Dpair> oriented_footprint;
    for (int i = 0; i < footprint_.size(); i++)
    {
        Dpair new_pt;
        new_pt.second = x_i + (footprint_[i].second*cos(theta_i) - footprint_[i].first*sin(theta_i));
        new_pt.first =  y_i + (footprint_[i].second*sin(theta_i) + footprint_[i].first*cos(theta_i));
        oriented_footprint.push_back(new_pt);
    }
    return footprintCost1(x_i, y_i, theta_i, oriented_footprint);
}



/*
*
*
* Cost related functions end  
*
*
*/

// print the grid in the terminal
void printGrid(vector<vector<GridCell>> &grid)
{
    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            GridCell &c = grid[i][j];

            printf("|(%2d,%2d) %3d %s|", c.cy, c.cx, c.target_dist, (c.target_mark ? "T" : "F"));
        }
        printf("\n");
    }
    printf("\n\n");
}

//visualize trajectory and footprint on top of the costmap
void visualize_traj(double x_i, double y_i, double theta_i, nsx::Trajectory &traj){
    char imgGrid[ROW][COL][3];
    char imgGridScaled[ROW * SCALE][COL * SCALE][3];

    int x0, x1, y0, y1;
    //orient the footprint
    vector<Dpair> oriented_footprint;

    // color
    // obs & inflation : blue
    // freespace : white
    // footprint : black
    // trajectory: blue (red if collision)

    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            int c = costmap[i][j];

            // free space
            if (c == 0)
            {
                imgGrid[i][j][0] = 255;
                imgGrid[i][j][1] = 255;
                imgGrid[i][j][2] = 255;
            }
            // obs & inflation
            else
            {
                imgGrid[i][j][0] = 255;
                imgGrid[i][j][1] = 255 - c * (int)(255/OBSTACLE_COST);
                imgGrid[i][j][2] = 255 - c * (int)(255/OBSTACLE_COST);
            }
        }
        
    }

    // fill footprint
    for (int i = 0; i < footprint_.size(); i++)
    {
        Dpair new_pt;
        new_pt.second = x_i + (footprint_[i].second*cos(theta_i) - footprint_[i].first*sin(theta_i));
        new_pt.first =  y_i + (footprint_[i].second*sin(theta_i) + footprint_[i].first*cos(theta_i));
        oriented_footprint.push_back(new_pt);
    }

        //loop through the footprint and mark it in the grid
    for (int i = 0; i < oriented_footprint.size() - 1; i++)
    {
        x0 = oriented_footprint[i].second/resolution_;
        y0 = oriented_footprint[i].first/resolution_;

        x1 = oriented_footprint[i+1].second/resolution_;
        y1 = oriented_footprint[i+1].first/resolution_;

        for (nsx::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance())
        {
            imgGrid[line.getY()][line.getX()][0] = 0;
            imgGrid[line.getY()][line.getX()][1] = 0;
            imgGrid[line.getY()][line.getX()][2] = 0;
        }
    }

    x0 = oriented_footprint.back().second/resolution_;
    y0 = oriented_footprint.back().first/resolution_;

    x1 = oriented_footprint.front().second/resolution_;
    y1 = oriented_footprint.front().first/resolution_;

    for (nsx::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance())
    {
        imgGrid[line.getY()][line.getX()][0] = 0;
        imgGrid[line.getY()][line.getX()][1] = 0;
        imgGrid[line.getY()][line.getX()][2] = 0;
    }

    // fill trajectory
    double x, y, th;
    int x_id, y_id;
    for (unsigned int i = 0; i < traj.getPointsSize(); i++)
    {
        traj.getPoint(i, x, y, th);
        x_id = (int)(x/resolution_);
        y_id = (int)(y/resolution_);
        
        imgGrid[y_id][x_id][0] = 255;
        imgGrid[y_id][x_id][1] = 0;
        imgGrid[y_id][x_id][2] = 0;

    }
    
    

    //scaling
    for (int i = 0; i < ROW; i++)
    {
        for (int i_s = 0; i_s < SCALE; i_s++)
        {
            for (int j = 0; j < COL; j++)
            {
                for (int j_s = 0; j_s < SCALE; j_s++)
                {
                    imgGridScaled[i * SCALE + i_s][j * SCALE + j_s][0] = imgGrid[i][j][0];
                    imgGridScaled[i * SCALE + i_s][j * SCALE + j_s][1] = imgGrid[i][j][1];
                    imgGridScaled[i * SCALE + i_s][j * SCALE + j_s][2] = imgGrid[i][j][2];
                }
            }
        }
    }

    Mat inflatedGridImg(ROW * SCALE, COL * SCALE, CV_8UC3, imgGridScaled);
    imshow("PIC", inflatedGridImg);
    waitKey(0);
}

void visualize_footprint(int row, int col, double x_i, double y_i, double theta_i)
{
    char imgGrid[row][col][3];
    int x0, x1, y0, y1;
    //orient the footprint
    vector<Dpair> oriented_footprint;

    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            imgGrid[i][j][0] = 255;
            imgGrid[i][j][1] = 255;
            imgGrid[i][j][2] = 255; 
        }
    }
    

    for (int i = 0; i < footprint_.size(); i++)
    {
        Dpair new_pt;
        new_pt.second = x_i + (footprint_[i].second*cos(theta_i) - footprint_[i].first*sin(theta_i));
        new_pt.first =  y_i + (footprint_[i].second*sin(theta_i) + footprint_[i].first*cos(theta_i));
        oriented_footprint.push_back(new_pt);
    }

    //loop through the footprint and mark it in the grid
    for (int i = 0; i < oriented_footprint.size() - 1; i++)
    {
        x0 = oriented_footprint[i].second/resolution_;
        y0 = oriented_footprint[i].first/resolution_;

        x1 = oriented_footprint[i+1].second/resolution_;
        y1 = oriented_footprint[i+1].first/resolution_;

        for (nsx::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance())
        {
            imgGrid[line.getY()][line.getX()][0] = 0;
            imgGrid[line.getY()][line.getX()][1] = 0;
            imgGrid[line.getY()][line.getX()][2] = 255;
        }
    }

    x0 = oriented_footprint.back().second/resolution_;
    y0 = oriented_footprint.back().first/resolution_;

    x1 = oriented_footprint.front().second/resolution_;
    y1 = oriented_footprint.front().first/resolution_;

    for (nsx::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance())
    {
        imgGrid[line.getY()][line.getX()][0] = 0;
        imgGrid[line.getY()][line.getX()][1] = 0;
        imgGrid[line.getY()][line.getX()][2] = 255;
    }

    //visualize
    Mat GridImgMAT(row, col, CV_8UC3, imgGrid);
    imshow("PIC", GridImgMAT);
    waitKey(0);

}

void visualize(vector<vector<GridCell>> &grid)
{
    char imgGrid[ROW][COL][3];
    char imgGridScaled[ROW * SCALE][COL * SCALE][3];
    double scaled_dist;

    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            GridCell &c = grid[i][j];
            if (c.target_dist == UNREACHABLE_COST)
            {
                imgGrid[i][j][0] = 0;
                imgGrid[i][j][1] = 0;
                imgGrid[i][j][2] = 0;
            }
            else if (c.target_dist == ROW * COL)
            {
                imgGrid[i][j][0] = 0;
                imgGrid[i][j][1] = 0;
                imgGrid[i][j][2] = 255;
            }
            else if (c.target_dist == 0)
            {
                imgGrid[i][j][0] = 255;
                imgGrid[i][j][1] = 0;
                imgGrid[i][j][2] = 0;
            }
            else
            {
                scaled_dist = (255.0 / (17.0)) * c.target_dist;
                imgGrid[i][j][0] = (char)scaled_dist;
                imgGrid[i][j][1] = 255;
                imgGrid[i][j][2] = (char)scaled_dist;
            }
        }
    }

    //scaling
    for (int i = 0; i < ROW; i++)
    {
        for (int i_s = 0; i_s < SCALE; i_s++)
        {
            for (int j = 0; j < COL; j++)
            {
                for (int j_s = 0; j_s < SCALE; j_s++)
                {
                    imgGridScaled[i * SCALE + i_s][j * SCALE + j_s][0] = imgGrid[i][j][0];
                    imgGridScaled[i * SCALE + i_s][j * SCALE + j_s][1] = imgGrid[i][j][1];
                    imgGridScaled[i * SCALE + i_s][j * SCALE + j_s][2] = imgGrid[i][j][2];
                }
            }
        }
    }

    Mat inflatedGridImg(ROW * SCALE, COL * SCALE, CV_8UC3, imgGridScaled);
    // Mat inflatedGridImgResized;

    // resize(inflatedGridImg, inflatedGridImgResized, Size(), 10, 10);

    imshow("PIC", inflatedGridImg);
    waitKey(0);
}

void initializeGrid(vector<vector<GridCell>> &grid)
{
    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            grid[i][j].cx = j;
            grid[i][j].cy = i;
            grid[i][j].target_dist = UNREACHABLE_COST;
            grid[i][j].target_mark = false;
        }
    }
}

bool updatePathCell(GridCell *current_cell, GridCell *check_cell, vector<vector<int>> &costmap)
{
    int cost = costmap[check_cell->cy][check_cell->cx];
    if (cost == OBSTACLE_COST)
    {
        check_cell->target_dist = ROW * COL;
        return false;
    }

    int new_target_dist = current_cell->target_dist + 1;
    if (new_target_dist < check_cell->target_dist)
    {
        check_cell->target_dist = new_target_dist;
    }
    return true;
}

void computeTargetDistance(queue<GridCell *> &dist_queue, vector<vector<GridCell>> &grid,
                           vector<vector<int>> &costmap)
{
    GridCell *current_cell;
    GridCell *check_cell;

    while (!dist_queue.empty())
    {
        current_cell = dist_queue.front();
        dist_queue.pop();

        if (current_cell->cx > 0)
        {
            check_cell = &grid[current_cell->cy][current_cell->cx - 1];
            if (!check_cell->target_mark)
            {
                check_cell->target_mark = true;
                if (updatePathCell(current_cell, check_cell, costmap))
                {
                    dist_queue.push(check_cell);
                }
            }
        }

        if (current_cell->cx < (COL - 1))
        {
            check_cell = &grid[current_cell->cy][current_cell->cx + 1];
            if (!check_cell->target_mark)
            {
                check_cell->target_mark = true;
                if (updatePathCell(current_cell, check_cell, costmap))
                {
                    dist_queue.push(check_cell);
                }
            }
        }

        if (current_cell->cy > 0)
        {
            check_cell = &grid[current_cell->cy - 1][current_cell->cx];
            if (!check_cell->target_mark)
            {
                check_cell->target_mark = true;
                if (updatePathCell(current_cell, check_cell, costmap))
                {
                    dist_queue.push(check_cell);
                }
            }
        }

        if (current_cell->cy < (ROW - 1))
        {
            check_cell = &grid[current_cell->cy + 1][current_cell->cx];
            if (!check_cell->target_mark)
            {
                check_cell->target_mark = true;
                if (updatePathCell(current_cell, check_cell, costmap))
                {
                    dist_queue.push(check_cell);
                }
            }
        }
    }
}

void setTargetCells(vector<vector<int>> &costmap, vector<vector<GridCell>> &grid, vector<Pair> &global_plan)
{
    bool started_path = false;
    queue<GridCell *> path_dist_queue;

    // Adjust global plan for resolution, if needed

    for (unsigned int i = 0; i < global_plan.size(); i++)
    {
        int g_x = global_plan[i].second;
        int g_y = global_plan[i].first;
        /*
        In the following if.. 
        ** need to convert world to grid and check each point is explored
        */
        if (isValid(g_y, g_x))
        {
            GridCell &current = grid[g_y][g_x];
            current.target_dist = 0;
            current.target_mark = true;
            path_dist_queue.push(&current);
            started_path = true;
        }
        else if (started_path)
        {
            break;
        }
    }
    if (!started_path)
    {
        return;
    }

    computeTargetDistance(path_dist_queue, grid, costmap);
}

void setLocalGoal(vector<vector<int>> &costmap, vector<vector<GridCell>> &grid, vector<Pair> &global_plan)
{
    int local_goal_x = -1;
    int local_goal_y = -1;
    bool started_path = false;
    queue<GridCell *> path_dist_queue;

    // Adjust global plan for resolution, if needed

    for (unsigned int i = 0; i < global_plan.size(); i++)
    {
        int g_x = global_plan[i].second;
        int g_y = global_plan[i].first;
        /*
        In the following if.. 
        ** need to convert world to grid and check each point is explored
        */
        if (isValid(g_y, g_x))
        {
            local_goal_x = g_x;
            local_goal_y = g_y;
            started_path = true;
        }
        else if (started_path)
        {
            break;
        }
    }
    if (!started_path)
    {
        return;
    }

    if (local_goal_x >= 0 && local_goal_y >= 0)
    {
        GridCell &current = grid[local_goal_y][local_goal_x];
        current.target_dist = 0;
        current.target_mark = true;
        path_dist_queue.push(&current);
    }

    computeTargetDistance(path_dist_queue, grid, costmap);
}

void generateTrajectory(
    double x, double y, double theta,
    double vx, double vy, double vtheta,
    double vx_samp, double vy_samp, double vtheta_samp,
    double acc_x, double acc_y, double acc_theta,
    double impossible_cost,
    nsx::Trajectory &traj)
{

    double x_i = x;
    double y_i = y;
    double theta_i = theta;

    double vx_i, vy_i, vtheta_i;

    vx_i = vx;
    vy_i = vy;
    vtheta_i = vtheta;

    //compute the magnitude of the velocities
    double vmag = hypot(vx_samp, vy_samp);

    //compute the number of steps we must take along this trajectory to be "safe"
    int num_steps;
    num_steps = int(max((vmag * sim_time_) / sim_granularity_, fabs(vtheta_samp) / angular_sim_granularity_) + 0.5);

    //we at least want to take one step... even if we won't move, we want to score our current position
    if (num_steps == 0)
    {
        num_steps = 1;
    }

    double dt = sim_time_ / num_steps;
    double time = 0.0;

    //create a potential trajectory
    traj.resetPoints();
    traj.xv_ = vx_samp;
    traj.yv_ = vy_samp;
    traj.thetav_ = vtheta_samp;
    traj.cost_ = -1.0;

    //initialize the costs for the trajectory
    double path_dist = 0.0;
    double goal_dist = 0.0;
    double occ_cost = 0.0;

    for (int i = 0; i < num_steps; i++)
    {
        int cell_x, cell_y;

        if (!worldToMap(x_i, y_i, cell_x, cell_y))
        {
            traj.cost_ = -1.0;
            return;
        }

        double footprint_cost = footprintCost(x_i, y_i, theta_i);

        //if the footprint hits an obstacle this trajectory is invalid
        if (footprint_cost < 0)
        {
            traj.cost_ = -1.0;
            return;
        }

        occ_cost = max(max(occ_cost, footprint_cost), (double)costmap[cell_y][cell_x]);
        path_dist = pathmap[cell_y][cell_x].target_dist;
        goal_dist = goalmap[cell_y][cell_x].target_dist;

        if(impossible_cost <= goal_dist || impossible_cost <= path_dist)
        {
            traj.cost_ = -2.0;
            return;
        }

        //the point is legal... add it to the trajectory
        traj.addPoint(x_i, y_i, theta_i);

        //calculate velocities
        vx_i = computeNewVelocity(vx_samp, vx_i, acc_x, dt);
        vy_i = computeNewVelocity(vy_samp, vy_i, acc_y, dt);
        vtheta_i = computeNewVelocity(vtheta_samp, vtheta_i, acc_theta, dt);

        //calculate positions
        x_i = computeNewXPosition(x_i, vx_i, vy_i, theta_i, dt);
        y_i = computeNewYPosition(y_i, vx_i, vy_i, theta_i, dt);
        theta_i = computeNewThetaPosition(theta_i, vtheta_i, dt);

        time += dt;
    }

    traj.cost_ = path_distance_bias_ * path_dist + goal_dist * goal_distance_bias_ + occdist_scale_ * occ_cost;
}

int main()
{
    // costmap[5][5] = OBSTACLE_COST;
    // costmap[5][6] = OBSTACLE_COST;
    // costmap[6][5] = OBSTACLE_COST;
    // costmap[5][7] = OBSTACLE_COST;
    // costmap[7][5] = OBSTACLE_COST;
    // costmap[5][8] = OBSTACLE_COST;

    // initializeGrid(goalmap);
    // initializeGrid(pathmap);

    // int ix = 3, iy = 4;

    // // //for goalmap
    // // GridCell& c_cell = goalmap[iy][ix];
    // // c_cell.target_dist = 0;
    // // c_cell.target_mark = true;

    // // queue<GridCell*> goal_dist_queue;
    // // goal_dist_queue.push(&c_cell);
    // // computeTargetDistance(goal_dist_queue, goalmap, costmap);

    // // printGrid(goalmap);
    // // visualize(goalmap);

    // //for pathmap
    // vector<Pair> global_path;
    // global_path.push_back(make_pair(iy, ix));
    // global_path.push_back(make_pair(iy - 1, ix + 1));
    // global_path.push_back(make_pair(iy - 2, ix + 2));
    // global_path.push_back(make_pair(iy - 3, ix + 3));
    // global_path.push_back(make_pair(iy - 4, ix + 4));
    // global_path.push_back(make_pair(iy - 5, ix + 5));

    // setTargetCells(costmap, pathmap, global_path);
    // printGrid(pathmap);
    // visualize(pathmap);

    // //for goalmap
    // setLocalGoal(costmap, goalmap, global_path);
    // printGrid(goalmap);
    // visualize(goalmap);


    fillFootprint();
    initializeGrid(goalmap);
    initializeGrid(pathmap);

    //for pathmap
    vector<Pair> global_path;
    nsx::Trajectory traj;

    int ix = 150;
    int iy = 150;
    for(int i = 0 ; i < 155 ; i++){
        global_path.push_back(make_pair(iy - i, ix + i));
    }

    setTargetCells(costmap, pathmap, global_path);
    setLocalGoal(costmap, goalmap, global_path);

    double x = 1.5;
    double y = 1.5;
    double th = M_PI_4;
    double vx = 0.2;
    double vy = 0;
    double vth = 0;
    double vsx = 1;
    double vsy = 0;
    double vsth = -1;
    double ax = 1;
    double ay = 0;
    double ath = 1;

    generateTrajectory(x, y, th, vx, vy, vth, vsx, vsy, vsth, ax, ay, ath, UNREACHABLE_COST-1, traj);

    visualize_traj(x, y, th, traj);


    

    return 0;
}