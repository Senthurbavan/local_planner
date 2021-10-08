#include <vector>
#include <queue>
#include <iostream>

#include <opencv2/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

static const unsigned int ROW = 10; //y
static const unsigned int COL = 10; //x
static const unsigned int SCALE = 20; //xvvv
static const unsigned int UNREACHABLE_COST = ROW*COL + 1;
static const unsigned char OBSTACLE_COST = 127;


struct GridCell
{
    int cx, cy;
    int target_dist;
    bool target_mark;
};

vector<vector<GridCell>> pathmap(ROW, vector<GridCell>(COL));
vector<vector<GridCell>> goalmap(ROW, vector<GridCell>(COL));
vector<vector<int>> costmap(ROW, vector<int>(COL, 0));   


void printGrid(vector<vector<GridCell>>& grid)
{
    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            GridCell& c = grid[i][j];

            printf("|(%2d,%2d) %3d %s|", c.cy, c.cx, c.target_dist, (c.target_mark?"T":"F")); 

        }
        cout << endl;
    }
}




void visualize(vector<vector<GridCell>>& grid)
{
    char imgGrid[ROW][COL][3];
    char imgGridScaled[ROW*SCALE][COL*SCALE][3];
    double scaled_dist;

    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            GridCell& c = grid[i][j];
            if (c.target_dist == UNREACHABLE_COST)
            {
                imgGrid[i][j][0] = 0;
                imgGrid[i][j][1] = 0;
                imgGrid[i][j][2] = 0;
            }
            else if (c.target_dist==ROW*COL)
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
                scaled_dist = (255.0/(12.0))*c.target_dist;
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
                    imgGridScaled[i*SCALE + i_s][j*SCALE + j_s][0] = imgGrid[i][j][0];
                    imgGridScaled[i*SCALE + i_s][j*SCALE + j_s][1] = imgGrid[i][j][1];
                    imgGridScaled[i*SCALE + i_s][j*SCALE + j_s][2] = imgGrid[i][j][2];
                }
                
            }
            
        }
        
    }
    

    Mat inflatedGridImg(ROW*SCALE, COL*SCALE, CV_8UC3, imgGridScaled);
    // Mat inflatedGridImgResized;

    // resize(inflatedGridImg, inflatedGridImgResized, Size(), 10, 10);

    imshow("PIC",inflatedGridImg);
    waitKey(0);
}

void initializeGrid(vector<vector<GridCell>>& grid)
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


bool updatePathCell(GridCell* current_cell, GridCell* check_cell, vector<vector<int>>& costmap)
{
    int cost = costmap[check_cell->cy][check_cell->cx];
    if(cost == OBSTACLE_COST)
    {
        check_cell->target_dist = ROW*COL;
        return false;
    }

    int new_target_dist = current_cell->target_dist + 1;
    if (new_target_dist < check_cell->target_dist)
    {       
        check_cell->target_dist = new_target_dist;
    }
    return true;
    
}

void computeTargetDistance(queue<GridCell*>& dist_queue, vector<vector<GridCell>>& grid,
                            vector<vector<int>>& costmap)
{
    GridCell* current_cell;
    GridCell* check_cell;

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
                if(updatePathCell(current_cell, check_cell, costmap))
                {
                    dist_queue.push(check_cell);
                }
            }
            
        }

        if (current_cell->cx < (COL-1))
        {
            check_cell = &grid[current_cell->cy][current_cell->cx + 1];
            if (!check_cell->target_mark)
            {
                check_cell->target_mark = true;
                if(updatePathCell(current_cell, check_cell, costmap))
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
                if(updatePathCell(current_cell, check_cell, costmap))
                {
                    dist_queue.push(check_cell);
                }
            }
            
        }

        if (current_cell->cy < (ROW-1))
        {
            check_cell = &grid[current_cell->cy + 1][current_cell->cx];
            if (!check_cell->target_mark)
            {
                check_cell->target_mark = true;
                if(updatePathCell(current_cell, check_cell, costmap))
                {
                    dist_queue.push(check_cell);
                }
            }
            
        }
        
    }
    
}



int main()
{   
    costmap[5][5] = OBSTACLE_COST;
    costmap[5][6] = OBSTACLE_COST;
    costmap[6][5] = OBSTACLE_COST;
    costmap[5][7] = OBSTACLE_COST;
    costmap[7][5] = OBSTACLE_COST;
    costmap[5][8] = OBSTACLE_COST;
    costmap[8][5] = OBSTACLE_COST;

    initializeGrid(goalmap);

    int ix = 3, iy = 4;

    GridCell& c_cell = goalmap[iy][ix];
    c_cell.target_dist = 0;
    c_cell.target_mark = true;

    queue<GridCell*> goal_dist_queue;

    goal_dist_queue.push(&c_cell);

    computeTargetDistance(goal_dist_queue, goalmap, costmap);

    printGrid(goalmap);
    visualize(goalmap);

    return 0;
}